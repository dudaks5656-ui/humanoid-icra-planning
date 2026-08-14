#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

// Reuse the already-audited scene, FCL, attachment, and reporting primitives without
// changing the random-reference source or its preserved output files. The include is
// translation-unit local and the original executable remains independently built.
#define private public
#define main preserved_random_reference_main
#include "reference_trajectory_generator.cpp"
#undef main
#undef private

namespace stage_constrained
{
struct RankedIk
{
  moveit::core::RobotState state;
  double joint_margin{};
  double self_clearance{};
  double environment_clearance{};
  int seed_id{};
  explicit RankedIk(const moveit::core::RobotModelConstPtr& model) : state(model) {}
};

struct Segment
{
  std::string stage;
  std::string method;
  std::vector<moveit::core::RobotState> states;
  bool object_attached{ false };
};

struct Trial
{
  int trial_id{};
  double initial_lift{};
  int branch_rank{};
  int ik_seeds_tested{};
  int collision_free_branches{};
  bool success{ false };
  std::string failure_stage;
  int failure_waypoint{ -1 };
  std::string failure_reason;
  std::string collision_pairs;
  double cartesian_fraction{ 0.0 };
  double min_joint_margin{ std::numeric_limits<double>::infinity() };
  double min_active_revolute_margin{ std::numeric_limits<double>::infinity() };
  double min_self_clearance{ std::numeric_limits<double>::infinity() };
  double min_environment_clearance{ std::numeric_limits<double>::infinity() };
  double max_orientation_error{ 0.0 };
  double max_position_error{ 0.0 };
  double planning_time_ms{ 0.0 };
  std::vector<Segment> segments;
  Eigen::Isometry3d attached_target_in_tcp{ Eigen::Isometry3d::Identity() };
};

class Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node) : node_(node), core_(node)
  {
    trials_csv_ = node_->get_parameter("stage_trials_csv").as_string();
    trajectory_csv_ = node_->get_parameter("stage_trajectory_csv").as_string();
    waypoints_yaml_ = node_->get_parameter("stage_waypoints_yaml").as_string();
    audit_md_ = node_->get_parameter("stage_audit_md").as_string();
    hold_for_rviz_ = node_->get_parameter("hold_for_rviz").as_bool();
    group_names_ = core_.left_arm_with_torso_group_->getVariableNames();
    initializeTrials();
  }

  bool run()
  {
    const std::vector<double> lift_order{ 0.25, 0.35, 0.30, 0.20, 0.40,
                                          0.25, 0.35, 0.30, 0.20, 0.40 };
    std::vector<Trial> trials;
    for (std::size_t index = 0; index < lift_order.size(); ++index)
    {
      const int rank = index < 5 ? 0 : 1;
      Trial trial = runTrial(static_cast<int>(index + 1), lift_order[index], rank);
      appendTrial(trial);
      trials.push_back(trial);
      RCLCPP_INFO(node_->get_logger(),
                  "STAGE_CONSTRAINED_PROGRESS trial=%d lift=%.2f branch=%d success=%s stage=%s waypoint=%d",
                  trial.trial_id, trial.initial_lift, rank, trial.success ? "true" : "false",
                  trial.failure_stage.c_str(), trial.failure_waypoint);
      if (trial.success)
      {
        selected_ = std::make_unique<Trial>(std::move(trials.back()));
        writeTrajectory(*selected_);
        writeWaypoints(*selected_);
        writeAudit(trials, selected_.get());
        publish(*selected_);
        return true;
      }
    }
    writeNotSelectedArtifacts();
    writeAudit(trials, nullptr);
    return false;
  }

  bool holdForRviz() const { return hold_for_rviz_; }

private:
  double jointMargin(const moveit::core::RobotState& state) const
  {
    double margin = std::numeric_limits<double>::infinity();
    std::vector<std::string> names = core_.left_arm_group_->getVariableNames();
    names.push_back("lift_joint");
    for (const auto& name : names)
    {
      const auto& bounds = core_.robot_model_->getVariableBounds(name);
      if (bounds.position_bounded_)
      {
        const double value = state.getVariablePosition(name);
        margin = std::min(margin, std::min(value - bounds.min_position_, bounds.max_position_ - value));
      }
    }
    return margin;
  }

  double activeRevoluteMargin(const moveit::core::RobotState& state) const
  {
    double margin = std::numeric_limits<double>::infinity();
    for (const auto& name : core_.left_arm_group_->getVariableNames())
    {
      const auto& bounds = core_.robot_model_->getVariableBounds(name);
      if (bounds.position_bounded_)
      {
        const double value = state.getVariablePosition(name);
        margin = std::min(margin, std::min(value - bounds.min_position_, bounds.max_position_ - value));
      }
    }
    return margin;
  }

  bool evaluate(moveit::core::RobotState& state, Trial& trial, std::string& reason, std::string& pairs)
  {
    state.update();
    const CollisionStatus status = core_.checkState(state);
    const auto clearance = core_.stateClearances(state);
    trial.min_joint_margin = std::min(trial.min_joint_margin, jointMargin(state));
    trial.min_active_revolute_margin = std::min(trial.min_active_revolute_margin, activeRevoluteMargin(state));
    trial.min_environment_clearance = std::min(trial.min_environment_clearance, clearance.first);
    trial.min_self_clearance = std::min(trial.min_self_clearance, clearance.second);
    pairs = pairString(status.pairs);
    if (!status.joint_limit_valid)
      reason = "JOINT_LIMIT_VIOLATION";
    else if (status.self_collision || status.environment_collision)
      reason = core_.collisionFailure(status);
    return status.joint_limit_valid && !status.self_collision && !status.environment_collision;
  }

  std::vector<RankedIk> rankIk(const geometry_msgs::msg::Pose& pose, double lift,
                               const moveit::core::RobotState& base, Trial& trial, int requested = 100)
  {
    std::vector<RankedIk> valid;
    for (int seed_id = 0; seed_id < requested; ++seed_id)
    {
      moveit::core::RobotState state = base;
      state.setVariablePosition("lift_joint", lift);
      state.setVariablePosition("waist_yaw_joint", 0.0);
      state.setVariablePosition("waist_pitch_joint", 0.0);
      if (core_.object_phase_ != ObjectPhase::ATTACHED)
        state.setVariablePosition("openarm_left_finger_joint1", core_.scene_config_.left_finger);
      if (seed_id > 0)
      {
        std::mt19937_64 rng(2026081200ULL + static_cast<std::uint64_t>(seed_id) +
                            static_cast<std::uint64_t>(std::llround(lift * 1e6)));
        for (const auto& variable : core_.left_arm_group_->getVariableNames())
        {
          const auto& bounds = core_.robot_model_->getVariableBounds(variable);
          std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
          state.setVariablePosition(variable, distribution(rng));
        }
        state.update();
      }
      ++trial.ik_seeds_tested;
      if (!state.setFromIK(core_.left_arm_group_, pose, core_.left_tcp_link_, core_.scene_config_.ik_timeout))
        continue;
      std::string reason, pairs;
      Trial scratch;
      if (!evaluate(state, scratch, reason, pairs))
        continue;
      const double margin = activeRevoluteMargin(state);
      if (!(margin > 1e-9))
        continue;
      RankedIk ranked(core_.robot_model_);
      ranked.state = state;
      ranked.joint_margin = margin;
      ranked.environment_clearance = scratch.min_environment_clearance;
      ranked.self_clearance = scratch.min_self_clearance;
      ranked.seed_id = seed_id;
      valid.push_back(ranked);
    }
    trial.collision_free_branches = static_cast<int>(valid.size());
    std::stable_sort(valid.begin(), valid.end(), [](const RankedIk& a, const RankedIk& b) {
      if (std::abs(a.joint_margin - b.joint_margin) > 1e-12)
        return a.joint_margin > b.joint_margin;
      if (std::abs(a.self_clearance - b.self_clearance) > 1e-12)
        return a.self_clearance > b.self_clearance;
      if (std::abs(a.environment_clearance - b.environment_clearance) > 1e-12)
        return a.environment_clearance > b.environment_clearance;
      return a.seed_id < b.seed_id;
    });
    return valid;
  }

  bool planOmpl(const std::string& stage, const moveit::core::RobotState& start,
                const moveit::core::RobotState& goal, const geometry_msgs::msg::Pose& desired,
                Trial& trial, Segment& segment)
  {
    core_.move_group_->clearPoseTargets();
    core_.move_group_->setStartState(start);
    core_.move_group_->setJointValueTarget(goal);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto before = std::chrono::steady_clock::now();
    const auto code = core_.move_group_->plan(plan);
    trial.planning_time_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - before).count();
    if (code != moveit::core::MoveItErrorCode::SUCCESS || plan.trajectory_.joint_trajectory.points.empty())
    {
      trial.failure_reason = "MOTION_PLANNING_FAILURE:CODE_" + std::to_string(code.val);
      return false;
    }
    segment.stage = stage;
    segment.method = "OMPL_RRTConnect";
    segment.object_attached = core_.object_phase_ == ObjectPhase::ATTACHED;
    segment.states.push_back(start);
    moveit::core::RobotState state = start;
    for (const auto& point : plan.trajectory_.joint_trajectory.points)
    {
      state.setVariablePositions(plan.trajectory_.joint_trajectory.joint_names, point.positions);
      std::string reason, pairs;
      if (!evaluate(state, trial, reason, pairs))
      {
        trial.failure_reason = reason;
        trial.collision_pairs = pairs;
        trial.failure_waypoint = static_cast<int>(segment.states.size());
        return false;
      }
      segment.states.push_back(state);
    }
    double position_error{}, orientation_error{};
    core_.poseError(state, desired, position_error, orientation_error);
    trial.max_position_error = std::max(trial.max_position_error, position_error);
    trial.max_orientation_error = std::max(trial.max_orientation_error, orientation_error);
    if (position_error > core_.scene_config_.position_tolerance + 1e-12 ||
        orientation_error > core_.scene_config_.orientation_tolerance + 1e-12)
    {
      trial.failure_reason = orientation_error > core_.scene_config_.orientation_tolerance ?
        "ENDPOINT_ORIENTATION_TOLERANCE_EXCEEDED" : "ENDPOINT_POSITION_TOLERANCE_EXCEEDED";
      return false;
    }
    return true;
  }

  bool cartesian(const std::string& stage, const moveit::core::RobotState& start,
                 const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to,
                 Trial& trial, Segment& segment)
  {
    const Eigen::Vector3d a(from.position.x, from.position.y, from.position.z);
    const Eigen::Vector3d b(to.position.x, to.position.y, to.position.z);
    const int intervals = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.005)));
    segment.stage = stage;
    segment.method = "CARTESIAN_CONTINUOUS_IK_MAX_0.005M";
    segment.object_attached = core_.object_phase_ == ObjectPhase::ATTACHED;
    segment.states.push_back(start);
    moveit::core::RobotState previous = start;
    int completed = 0;
    for (int i = 1; i <= intervals; ++i)
    {
      const double ratio = static_cast<double>(i) / static_cast<double>(intervals);
      geometry_msgs::msg::Pose pose = from;
      pose.position.x = from.position.x + ratio * (to.position.x - from.position.x);
      pose.position.y = from.position.y + ratio * (to.position.y - from.position.y);
      pose.position.z = from.position.z + ratio * (to.position.z - from.position.z);
      moveit::core::RobotState next = previous;
      if (!next.setFromIK(core_.left_arm_group_, pose, core_.left_tcp_link_, core_.scene_config_.ik_timeout))
      {
        trial.failure_reason = "CONTINUOUS_IK_FAILURE";
        trial.failure_waypoint = i;
        trial.cartesian_fraction = static_cast<double>(completed) / intervals;
        return false;
      }
      next.setVariablePosition("waist_yaw_joint", 0.0);
      next.setVariablePosition("waist_pitch_joint", 0.0);
      std::string reason, pairs;
      if (!evaluate(next, trial, reason, pairs))
      {
        trial.failure_reason = reason;
        trial.collision_pairs = pairs;
        trial.failure_waypoint = i;
        trial.cartesian_fraction = static_cast<double>(completed) / intervals;
        return false;
      }
      segment.states.push_back(next);
      previous = next;
      ++completed;
    }
    trial.cartesian_fraction = 1.0;
    double position_error{}, orientation_error{};
    core_.poseError(previous, to, position_error, orientation_error);
    trial.max_position_error = std::max(trial.max_position_error, position_error);
    trial.max_orientation_error = std::max(trial.max_orientation_error, orientation_error);
    return true;
  }

  bool graspClosure(const moveit::core::RobotState& start, Trial& trial, Segment& segment)
  {
    core_.setFingerTargetContactAllowed(true);
    segment.stage = "GRASP";
    segment.method = "FINGER_ONLY_10_STEP_TASK_SCOPED_TOUCH";
    segment.object_attached = false;
    segment.states.push_back(start);
    const double q0 = start.getVariablePosition("openarm_left_finger_joint1");
    for (int i = 1; i <= 10; ++i)
    {
      moveit::core::RobotState state = start;
      state.setVariablePosition("openarm_left_finger_joint1",
                                q0 + static_cast<double>(i) / 10.0 * (core_.scene_config_.q_contact - q0));
      std::string reason, pairs;
      if (!evaluate(state, trial, reason, pairs))
      {
        trial.failure_reason = reason;
        trial.collision_pairs = pairs;
        trial.failure_waypoint = i;
        return false;
      }
      segment.states.push_back(state);
    }
    return true;
  }

  Trial runTrial(int trial_id, double lift, int branch_rank)
  {
    Trial trial;
    trial.trial_id = trial_id;
    trial.initial_lift = lift;
    trial.branch_rank = branch_rank;
    Candidate candidate;
    candidate.id = "stage_constrained_" + std::to_string(trial_id);
    candidate.lift = lift;
    candidate.yaw = candidate.pitch = 0.0;
    core_.resetSceneForCandidate();
    moveit::core::RobotState initial = core_.initialState(candidate);

    const auto top_pose = core_.approachPose(core_.scene_config_.rim_clearance);
    const auto grasp_pose = core_.graspPose();
    auto branches = rankIk(top_pose, lift, initial, trial, 100);
    if (branches.size() <= static_cast<std::size_t>(branch_rank))
    {
      trial.failure_stage = "TOP_APPROACH";
      trial.failure_reason = "INSUFFICIENT_COLLISION_FREE_IK_BRANCHES";
      return trial;
    }

    Segment top;
    if (!planOmpl("TOP_APPROACH", initial, branches[branch_rank].state, top_pose, trial, top))
    {
      trial.failure_stage = "TOP_APPROACH";
      return trial;
    }
    trial.segments.push_back(top);
    moveit::core::RobotState current = top.states.back();

    Segment descent;
    if (!cartesian("VERTICAL_DESCENT", current, top_pose, grasp_pose, trial, descent))
    {
      trial.failure_stage = "VERTICAL_DESCENT";
      return trial;
    }
    trial.segments.push_back(descent);
    current = descent.states.back();

    Segment grasp;
    if (!graspClosure(current, trial, grasp))
    {
      trial.failure_stage = "GRASP";
      return trial;
    }
    trial.segments.push_back(grasp);
    current = grasp.states.back();
    core_.attachTargetAtomically(current);
    trial.attached_target_in_tcp = core_.attached_target_in_tcp_;

    const auto lift_pose = core_.liftClearPose();
    Segment lift_segment;
    if (!cartesian("LIFT_CLEAR", current, grasp_pose, lift_pose, trial, lift_segment))
    {
      trial.failure_stage = "LIFT_CLEAR";
      return trial;
    }
    trial.segments.push_back(lift_segment);
    current = lift_segment.states.back();

    const auto transfer_pose = core_.transferOutsidePose(trial.attached_target_in_tcp);
    auto transfer_branches = rankIk(transfer_pose, current.getVariablePosition("lift_joint"), current, trial, 100);
    if (transfer_branches.empty())
    {
      trial.failure_stage = "TRANSFER_OUTSIDE";
      trial.failure_reason = "NO_COLLISION_FREE_TRANSFER_IK";
      return trial;
    }
    Segment transfer;
    if (!planOmpl("TRANSFER_OUTSIDE", current, transfer_branches.front().state,
                  transfer_pose, trial, transfer))
    {
      trial.failure_stage = "TRANSFER_OUTSIDE";
      return trial;
    }
    trial.segments.push_back(transfer);
    trial.success = denseValidate(trial);
    if (!trial.success && trial.failure_stage.empty())
      trial.failure_stage = "DENSE_VALIDATION";
    return trial;
  }

  bool denseValidate(Trial& trial)
  {
    core_.resetSceneForCandidate();
    moveit::core::RobotState current = trial.segments.front().states.front();
    for (auto& segment : trial.segments)
    {
      if (segment.stage == "GRASP")
        core_.setFingerTargetContactAllowed(true);
      double discontinuity = 0.0;
      for (const auto& name : core_.whole_body_group_->getVariableNames())
        discontinuity = std::max(discontinuity,
          std::abs(current.getVariablePosition(name) - segment.states.front().getVariablePosition(name)));
      if (discontinuity > 1e-6)
      {
        trial.failure_reason = "STAGE_START_DISCONTINUITY";
        return false;
      }
      for (std::size_t point = 1; point < segment.states.size(); ++point)
      {
        const auto from = current;
        const auto& to = segment.states[point];
        std::size_t steps = 1;
        for (const auto& name : core_.whole_body_group_->getVariableNames())
        {
          const auto* joint = core_.robot_model_->getJointOfVariable(name);
          const double resolution = joint->getType() == moveit::core::JointModel::PRISMATIC ? 0.005 : 0.01;
          steps = std::max(steps, static_cast<std::size_t>(std::ceil(
            std::abs(to.getVariablePosition(name) - from.getVariablePosition(name)) / resolution)));
        }
        for (std::size_t i = 1; i <= steps; ++i)
        {
          current = from;
          const double ratio = static_cast<double>(i) / steps;
          for (const auto& name : core_.whole_body_group_->getVariableNames())
            current.setVariablePosition(name, from.getVariablePosition(name) + ratio *
              (to.getVariablePosition(name) - from.getVariablePosition(name)));
          std::string reason, pairs;
          if (!evaluate(current, trial, reason, pairs))
          {
            trial.failure_reason = reason;
            trial.collision_pairs = pairs;
            trial.failure_waypoint = static_cast<int>(point);
            return false;
          }
        }
      }
      if (segment.stage == "GRASP")
      {
        core_.attachTargetAtomically(current);
        trial.attached_target_in_tcp = core_.attached_target_in_tcp_;
      }
    }
    if (!(trial.min_active_revolute_margin > 1e-9))
    {
      trial.failure_reason = "ACTIVE_REVOLUTE_JOINT_AT_BOUND";
      return false;
    }
    if (trial.max_orientation_error > core_.scene_config_.orientation_tolerance + 1e-12)
    {
      trial.failure_reason = "ENDPOINT_ORIENTATION_TOLERANCE_EXCEEDED";
      return false;
    }
    return true;
  }

  void initializeTrials() const
  {
    std::ofstream out(trials_csv_, std::ios::trunc);
    out << "timestamp,trial_id,initial_lift,branch_rank,ik_seeds_tested,collision_free_ik_branches,success,"
           "failure_stage,failure_waypoint,failure_reason,collision_pairs,cartesian_fraction,min_joint_margin,"
           "min_active_revolute_margin,min_self_clearance,min_environment_clearance,max_position_error,"
           "max_orientation_error,planning_time_ms\n";
  }

  void appendTrial(const Trial& t) const
  {
    std::ofstream out(trials_csv_, std::ios::app);
    out << csvEscape(timestampNow()) << ',' << t.trial_id << ',' << std::setprecision(15) << t.initial_lift << ','
        << t.branch_rank << ',' << t.ik_seeds_tested << ',' << t.collision_free_branches << ','
        << (t.success ? 1 : 0) << ',' << csvEscape(t.failure_stage) << ',' << t.failure_waypoint << ','
        << csvEscape(t.failure_reason) << ',' << csvEscape(t.collision_pairs) << ',' << t.cartesian_fraction << ','
        << t.min_joint_margin << ',' << t.min_active_revolute_margin << ',' << t.min_self_clearance << ','
        << t.min_environment_clearance << ',' << t.max_position_error << ',' << t.max_orientation_error << ','
        << t.planning_time_ms << '\n';
  }

  void writeTrajectory(const Trial& trial)
  {
    std::ofstream out(trajectory_csv_, std::ios::trunc);
    out << "stage,method,point_index";
    for (const auto& name : core_.whole_body_group_->getVariableNames()) out << ',' << name;
    out << ",tcp_x,tcp_y,tcp_z,tcp_qx,tcp_qy,tcp_qz,tcp_qw,lift,yaw,pitch,object_attached,"
           "joint_limit_margin,self_collision_clearance,environment_clearance,validity\n";
    core_.resetSceneForCandidate();
    for (const auto& segment : trial.segments)
    {
      if (segment.stage == "GRASP") core_.setFingerTargetContactAllowed(true);
      for (std::size_t index = 0; index < segment.states.size(); ++index)
      {
        moveit::core::RobotState state = segment.states[index];
        const auto status = core_.checkState(state);
        const auto clearance = core_.stateClearances(state);
        const auto& tcp = state.getGlobalLinkTransform(core_.left_tcp_link_);
        Eigen::Quaterniond q(tcp.rotation()); q.normalize();
        out << segment.stage << ',' << segment.method << ',' << index;
        for (const auto& name : core_.whole_body_group_->getVariableNames())
          out << ',' << std::setprecision(15) << state.getVariablePosition(name);
        out << ',' << tcp.translation().x() << ',' << tcp.translation().y() << ',' << tcp.translation().z()
            << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
            << state.getVariablePosition("lift_joint") << ',' << state.getVariablePosition("waist_yaw_joint")
            << ',' << state.getVariablePosition("waist_pitch_joint") << ',' << (segment.object_attached ? 1 : 0)
            << ',' << jointMargin(state) << ',' << clearance.second << ',' << clearance.first << ','
            << (status.joint_limit_valid && !status.self_collision && !status.environment_collision ? 1 : 0) << '\n';
      }
      if (segment.stage == "GRASP") core_.attachTargetAtomically(segment.states.back());
    }
  }

  void writeWaypoints(const Trial& trial) const
  {
    std::ofstream out(waypoints_yaml_, std::ios::trunc);
    out << "status: STAGE_CONSTRAINED_OFFLINE_REFERENCE\n"
           "classification: " << (std::min(trial.min_self_clearance, trial.min_environment_clearance) < 0.001 ?
             "VALID_BUT_LOW_CLEARANCE_REFERENCE" : "VALID_REFERENCE") << "\n"
           "reference_grasp: KINEMATIC_REFERENCE_GRASP_50MM\n"
           "trajectory_execution_performed: false\n"
           "initial_lift: " << trial.initial_lift << "\n"
           "yaw: 0.0\npitch: 0.0\n"
           "min_joint_limit_margin: " << trial.min_joint_margin << "\n"
           "min_active_revolute_margin: " << trial.min_active_revolute_margin << "\n"
           "min_self_collision_clearance: " << trial.min_self_clearance << "\n"
           "min_environment_clearance: " << trial.min_environment_clearance << "\n"
           "stages:\n";
    for (const auto& segment : trial.segments)
    {
      const auto& first = segment.states.front();
      const auto& last = segment.states.back();
      const auto& a = first.getGlobalLinkTransform(core_.left_tcp_link_);
      const auto& b = last.getGlobalLinkTransform(core_.left_tcp_link_);
      out << "  " << segment.stage << ":\n    method: " << segment.method
          << "\n    waypoint_count: " << segment.states.size()
          << "\n    start_tcp_xyz: [" << a.translation().x() << ", " << a.translation().y() << ", " << a.translation().z()
          << "]\n    end_tcp_xyz: [" << b.translation().x() << ", " << b.translation().y() << ", " << b.translation().z()
          << "]\n    start_lift: " << first.getVariablePosition("lift_joint")
          << "\n    end_lift: " << last.getVariablePosition("lift_joint") << "\n";
    }
    out << "tcp_to_target_attached_transform:\n  xyz: [" << trial.attached_target_in_tcp.translation().x() << ", "
        << trial.attached_target_in_tcp.translation().y() << ", " << trial.attached_target_in_tcp.translation().z()
        << "]\n";
  }

  void writeAudit(const std::vector<Trial>& trials, const Trial* selected) const
  {
    std::ofstream out(audit_md_, std::ios::trunc);
    out << "# Stage-constrained top-open reference audit\n\n"
           "- Preserved random-reference classification: `FEASIBLE_BUT_NOT_ROBUST_RANDOM_REFERENCE`\n"
           "- Methods: OMPL TOP_APPROACH; Cartesian VERTICAL_DESCENT; 10-step finger-only GRASP; "
           "Cartesian LIFT_CLEAR; OMPL TRANSFER_OUTSIDE.\n"
           "- Cartesian TCP spacing: <= 0.005 m; dense validation: revolute <= 0.01 rad, prismatic <= 0.005 m.\n"
           "- Fixed grasp: `KINEMATIC_REFERENCE_GRASP_50MM`; force closure and hardware stability not validated.\n"
           "- Global SRDF ACM was not changed. Only finger-target contact was task-scoped during GRASP.\n"
           "- Trials executed: " << trials.size() << "/10\n\n";
    out << "|trial|lift|branch|IK seeds|CF branches|result|failure stage|waypoint|fraction|joint margin|self clearance|environment clearance|pairs|\n"
           "|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---|\n";
    for (const auto& t : trials)
      out << '|' << t.trial_id << '|' << t.initial_lift << '|' << t.branch_rank << '|' << t.ik_seeds_tested
          << '|' << t.collision_free_branches << '|' << (t.success ? "PASS" : t.failure_reason) << '|'
          << t.failure_stage << '|' << t.failure_waypoint << '|' << t.cartesian_fraction << '|'
          << t.min_joint_margin << '|' << t.min_self_clearance << '|' << t.min_environment_clearance << '|'
          << t.collision_pairs << "|\n";
    if (selected)
    {
      const bool low = std::min(selected->min_self_clearance, selected->min_environment_clearance) < 0.001;
      out << "\n## Result\n\n- Status: `" << (low ? "VALID_BUT_LOW_CLEARANCE_REFERENCE" : "VALID_REFERENCE")
          << "`\n- Selected trial: " << selected->trial_id << "\n- Initial Lift: " << selected->initial_lift
          << " m\n- Minimum joint-limit margin: " << selected->min_joint_margin
          << "\n- Minimum active-revolute margin: " << selected->min_active_revolute_margin
          << "\n- Minimum self-collision clearance: " << selected->min_self_clearance
          << " m\n- Minimum environment clearance: " << selected->min_environment_clearance
          << " m\n- Maximum position error: " << selected->max_position_error
          << " m\n- Maximum orientation error: " << selected->max_orientation_error
          << " rad (limit 0.03 rad)\n";
    }
    else
      out << "\n## Result\n\n`NO_APPROVABLE_STAGE_CONSTRAINED_REFERENCE`\n";
    out << "\nNo trajectory execution, controller, ros2_control, or hardware node was used.\n";
  }

  void writeNotSelectedArtifacts() const
  {
    std::ofstream trajectory(trajectory_csv_, std::ios::trunc);
    trajectory << "status,reason\nNOT_SELECTED,NO_APPROVABLE_STAGE_CONSTRAINED_REFERENCE\n";
    std::ofstream waypoints(waypoints_yaml_, std::ios::trunc);
    waypoints << "status: NOT_SELECTED\nreason: NO_APPROVABLE_STAGE_CONSTRAINED_REFERENCE\n";
  }

  void publish(const Trial& trial)
  {
    CandidateResult result(core_.robot_model_);
    result.success = true;
    result.candidate.lift = trial.initial_lift;
    result.candidate.yaw = result.candidate.pitch = 0.0;
    result.attached_target_in_tcp = trial.attached_target_in_tcp;
    moveit::core::robotStateToRobotStateMsg(trial.segments.front().states.front(), result.start_state);
    for (const auto& segment : trial.segments)
    {
      moveit_msgs::msg::RobotTrajectory trajectory;
      trajectory.joint_trajectory.joint_names = core_.whole_body_group_->getVariableNames();
      for (std::size_t i = 0; i < segment.states.size(); ++i)
      {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (const auto& name : trajectory.joint_trajectory.joint_names)
          point.positions.push_back(segment.states[i].getVariablePosition(name));
        point.time_from_start.sec = static_cast<int32_t>(i / 10);
        point.time_from_start.nanosec = static_cast<uint32_t>((i % 10) * 100000000);
        trajectory.joint_trajectory.points.push_back(point);
      }
      result.trajectories.push_back(trajectory);
      result.trajectory_stages.push_back(segment.stage);
    }
    core_.publishResult(result);
    core_.publishReferenceMarkers(result);
    core_.publishState(trial.segments.front().states.front());
  }

  rclcpp::Node::SharedPtr node_;
  ReferenceTrajectoryGenerator core_;
  std::vector<std::string> group_names_;
  std::string trials_csv_, trajectory_csv_, waypoints_yaml_, audit_md_;
  bool hold_for_rviz_{ false };
  std::unique_ptr<Trial> selected_;
};
}  // namespace stage_constrained

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("stage_constrained_reference_generator", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  int exit_code = 1;
  try
  {
    stage_constrained::Runner runner(node);
    const bool success = runner.run();
    if (success && runner.holdForRviz())
    {
      RCLCPP_INFO(node->get_logger(), "Stage-constrained reference ready; planning-only RViz hold active.");
      while (rclcpp::ok()) rclcpp::sleep_for(std::chrono::seconds(1));
    }
    exit_code = success ? 0 : 2;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Stage-constrained generator failed: %s", error.what());
  }
  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
