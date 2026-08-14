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

// Reuse only the already-audited scene/object-lifecycle implementation.  The
// previous random and stage-constrained generators remain immutable files.
#define private public
#define main preserved_random_reference_main
#include "reference_trajectory_generator.cpp"
#undef main
#undef private

namespace stage_constrained_v2
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
  double cartesian_fraction{};
  double min_joint_margin{ std::numeric_limits<double>::infinity() };
  double min_active_revolute_margin{ std::numeric_limits<double>::infinity() };
  double min_self_clearance{ std::numeric_limits<double>::infinity() };
  double min_environment_clearance{ std::numeric_limits<double>::infinity() };
  double max_orientation_error{};
  double max_position_error{};
  double planning_time_ms{};
  std::vector<Segment> segments;
  Eigen::Isometry3d attached_target_in_tcp{ Eigen::Isometry3d::Identity() };
};

struct TrialRecord
{
  Trial trial;
  int grasp_ik_branches{};
  double accepted_spacing{};
  int reverse_failure_waypoint{ -1 };
  double reverse_fraction{};
  int branch_seed_id{ -1 };
};

class Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node)
    : node_(node), core_(node)
  {
    trials_csv_ = node_->get_parameter("stage_v2_trials_csv").as_string();
    trajectory_csv_ = node_->get_parameter("stage_v2_trajectory_csv").as_string();
    waypoints_yaml_ = node_->get_parameter("stage_v2_waypoints_yaml").as_string();
    audit_md_ = node_->get_parameter("stage_v2_audit_md").as_string();
    hold_for_rviz_ = node_->get_parameter("hold_for_rviz").as_bool();
    initializeTrials();
  }

  bool run()
  {
    // v1 tested only ranks 0 and 1 even though the successful discovery lifts
    // exposed 18--24 branches.  Exercise every reproducibly ranked branch at
    // those lifts instead of treating one local IK branch as representative.
    std::vector<std::pair<double, int>> candidates;
    for (int rank = 0; rank < 24; ++rank)
      candidates.emplace_back(0.35, rank);
    for (int rank = 0; rank < 24; ++rank)
      candidates.emplace_back(0.40, rank);
    std::vector<TrialRecord> records;
    std::vector<TrialRecord> successes;
    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
      const auto [lift, branch_rank] = candidates[index];
      TrialRecord record = runTrial(static_cast<int>(index + 1), lift, branch_rank);
      appendTrial(record);
      records.push_back(record);
      RCLCPP_INFO(node_->get_logger(),
                  "STAGE_V2_PROGRESS trial=%d lift=%.2f branch=%d success=%s stage=%s waypoint=%d fraction=%.6f spacing=%.4f",
                  record.trial.trial_id, record.trial.initial_lift, branch_rank,
                  record.trial.success ? "true" : "false", record.trial.failure_stage.c_str(),
                  record.trial.failure_waypoint, record.reverse_fraction, record.accepted_spacing);
      if (record.trial.success)
        successes.push_back(record);
    }
    // A single stochastic success is not a stable reference set.  Preserve all
    // trial evidence and select only after five complete dense-valid candidates.
    if (successes.size() >= 5)
    {
      std::stable_sort(successes.begin(), successes.end(), [](const TrialRecord& a, const TrialRecord& b) {
        const Trial& x = a.trial;
        const Trial& y = b.trial;
        if (std::abs(x.min_active_revolute_margin - y.min_active_revolute_margin) > 1e-12)
          return x.min_active_revolute_margin > y.min_active_revolute_margin;
        if (std::abs(x.min_environment_clearance - y.min_environment_clearance) > 1e-12)
          return x.min_environment_clearance > y.min_environment_clearance;
        if (std::abs(x.min_self_clearance - y.min_self_clearance) > 1e-12)
          return x.min_self_clearance > y.min_self_clearance;
        return x.planning_time_ms < y.planning_time_ms;
      });
      selected_ = std::make_unique<Trial>(successes.front().trial);
      writeTrajectory(*selected_);
      writeWaypoints(*selected_);
      writeAudit(records, &successes.front());
      publish(*selected_);
      return true;
    }
    writeNotSelectedArtifacts();
    writeAudit(records, nullptr);
    return false;
  }

  bool holdForRviz() const { return hold_for_rviz_; }

private:
  static void mergeMetrics(Trial& destination, const Trial& source)
  {
    destination.min_joint_margin = std::min(destination.min_joint_margin, source.min_joint_margin);
    destination.min_active_revolute_margin =
      std::min(destination.min_active_revolute_margin, source.min_active_revolute_margin);
    destination.min_self_clearance = std::min(destination.min_self_clearance, source.min_self_clearance);
    destination.min_environment_clearance =
      std::min(destination.min_environment_clearance, source.min_environment_clearance);
    destination.max_position_error = std::max(destination.max_position_error, source.max_position_error);
    destination.max_orientation_error = std::max(destination.max_orientation_error, source.max_orientation_error);
    destination.planning_time_ms += source.planning_time_ms;
  }

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

  bool evaluate(moveit::core::RobotState& state, Trial& trial,
                std::string& reason, std::string& pairs)
  {
    state.update();
    const CollisionStatus status = core_.checkState(state);
    const auto clearance = core_.stateClearances(state);
    trial.min_joint_margin = std::min(trial.min_joint_margin, jointMargin(state));
    trial.min_active_revolute_margin =
      std::min(trial.min_active_revolute_margin, activeRevoluteMargin(state));
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
                               const moveit::core::RobotState& base, Trial& trial,
                               int requested = 100)
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
        std::mt19937_64 rng(2026081300ULL + static_cast<std::uint64_t>(seed_id) +
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
      Trial scratch;
      std::string reason, pairs;
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

  bool planOmpl(const std::string& stage,
                const moveit::core::RobotState& start,
                const moveit::core::RobotState& goal,
                const geometry_msgs::msg::Pose& desired,
                Trial& trial,
                Segment& segment)
  {
    core_.move_group_->clearPoseTargets();
    core_.move_group_->setStartState(start);
    core_.move_group_->setJointValueTarget(goal);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto before = std::chrono::steady_clock::now();
    const auto code = core_.move_group_->plan(plan);
    trial.planning_time_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - before).count();
    if (code != moveit::core::MoveItErrorCode::SUCCESS ||
        plan.trajectory_.joint_trajectory.points.empty())
    {
      trial.failure_reason = "MOTION_PLANNING_FAILURE:CODE_" + std::to_string(code.val);
      return false;
    }
    segment.stage = stage;
    segment.method = "OMPL_RRTConnect_YAW_PITCH_LOCKED";
    segment.object_attached = core_.object_phase_ == ObjectPhase::ATTACHED;
    segment.states = { start };
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

  bool followCartesian(const moveit::core::RobotState& start,
                       const geometry_msgs::msg::Pose& from,
                       const geometry_msgs::msg::Pose& to,
                       double spacing,
                       bool attached,
                       Trial& metrics,
                       Segment& segment,
                       int& failure_waypoint,
                       double& fraction,
                       std::string& failure_reason,
                       std::string& collision_pairs)
  {
    const Eigen::Vector3d a(from.position.x, from.position.y, from.position.z);
    const Eigen::Vector3d b(to.position.x, to.position.y, to.position.z);
    const int intervals = std::max(1, static_cast<int>(std::ceil((b - a).norm() / spacing)));
    segment.method = "DETERMINISTIC_MULTISTART_SEQUENTIAL_IK_MAX_" + std::to_string(spacing) + "M";
    segment.object_attached = attached;
    segment.states = { start };
    moveit::core::RobotState previous = start;
    int completed = 0;
    for (int i = 1; i <= intervals; ++i)
    {
      const double ratio = static_cast<double>(i) / static_cast<double>(intervals);
      geometry_msgs::msg::Pose pose = from;
      pose.position.x += ratio * (to.position.x - from.position.x);
      pose.position.y += ratio * (to.position.y - from.position.y);
      pose.position.z += ratio * (to.position.z - from.position.z);
      moveit::core::RobotState next = previous;
      bool solved = next.setFromIK(core_.left_arm_group_, pose, core_.left_tcp_link_,
                                   core_.scene_config_.ik_timeout);
      if (solved)
      {
        next.setVariablePosition("waist_yaw_joint", 0.0);
        next.setVariablePosition("waist_pitch_joint", 0.0);
        Trial scratch;
        std::string reason, pairs;
        solved = evaluate(next, scratch, reason, pairs);
      }
      if (!solved)
      {
        double best_distance = std::numeric_limits<double>::infinity();
        // Deterministic local perturbations allow a branch switch at the narrow
        // 14 mm v1 gap without relaxing pose, torso, limits, or collision checks.
        for (int attempt = 0; attempt < 32; ++attempt)
        {
          moveit::core::RobotState seed = previous;
          std::mt19937_64 rng(2026081400ULL + static_cast<std::uint64_t>(i * 101 + attempt));
          for (const auto& variable : core_.left_arm_group_->getVariableNames())
          {
            const auto& bounds = core_.robot_model_->getVariableBounds(variable);
            std::uniform_real_distribution<double> sample(bounds.min_position_, bounds.max_position_);
            seed.setVariablePosition(variable, sample(rng));
          }
          seed.update();
          if (!seed.setFromIK(core_.left_arm_group_, pose, core_.left_tcp_link_,
                              core_.scene_config_.ik_timeout))
            continue;
          seed.setVariablePosition("waist_yaw_joint", 0.0);
          seed.setVariablePosition("waist_pitch_joint", 0.0);
          Trial scratch;
          std::string reason, pairs;
          if (!evaluate(seed, scratch, reason, pairs))
            continue;
          double distance = 0.0;
          for (const auto& variable : core_.left_arm_group_->getVariableNames())
          {
            const double delta = seed.getVariablePosition(variable) - previous.getVariablePosition(variable);
            distance += delta * delta;
          }
          if (distance < best_distance)
          {
            next = seed;
            best_distance = distance;
            solved = true;
          }
        }
      }
      if (!solved)
      {
        failure_waypoint = i;
        fraction = static_cast<double>(completed) / intervals;
        failure_reason = "CONTINUOUS_IK_FAILURE";
        return false;
      }
      next.setVariablePosition("waist_yaw_joint", 0.0);
      next.setVariablePosition("waist_pitch_joint", 0.0);
      std::string reason, pairs;
      if (!evaluate(next, metrics, reason, pairs))
      {
        failure_waypoint = i;
        fraction = static_cast<double>(completed) / intervals;
        failure_reason = reason;
        collision_pairs = pairs;
        return false;
      }
      segment.states.push_back(next);
      previous = next;
      ++completed;
    }
    fraction = 1.0;
    double position_error{}, orientation_error{};
    core_.poseError(previous, to, position_error, orientation_error);
    metrics.max_position_error = std::max(metrics.max_position_error, position_error);
    metrics.max_orientation_error = std::max(metrics.max_orientation_error, orientation_error);
    if (position_error > core_.scene_config_.position_tolerance + 1e-12 ||
        orientation_error > core_.scene_config_.orientation_tolerance + 1e-12)
    {
      failure_waypoint = intervals;
      failure_reason = orientation_error > core_.scene_config_.orientation_tolerance ?
        "ENDPOINT_ORIENTATION_TOLERANCE_EXCEEDED" : "ENDPOINT_POSITION_TOLERANCE_EXCEEDED";
      return false;
    }
    return true;
  }

  bool adaptiveContinuation(const moveit::core::RobotState& start,
                            const geometry_msgs::msg::Pose& from,
                            const geometry_msgs::msg::Pose& to,
                            bool attached,
                            const std::string& stage,
                            Trial& trial,
                            Segment& accepted,
                            double& accepted_spacing,
                            int& failure_waypoint,
                            double& fraction)
  {
    static const std::vector<double> spacings{ 0.005, 0.0025, 0.001 };
    double best_fraction = -1.0;
    int best_waypoint = -1;
    double best_spacing = 0.0;
    Trial best_metrics;
    std::string best_reason = "CONTINUOUS_IK_FAILURE";
    std::string best_pairs;
    for (const double spacing : spacings)
    {
      Trial attempt;
      Segment segment;
      segment.stage = stage;
      int failed_at = -1;
      double completed_fraction = 0.0;
      std::string reason, pairs;
      if (followCartesian(start, from, to, spacing, attached, attempt, segment,
                          failed_at, completed_fraction, reason, pairs))
      {
        mergeMetrics(trial, attempt);
        accepted = std::move(segment);
        accepted_spacing = spacing;
        failure_waypoint = -1;
        fraction = 1.0;
        return true;
      }
      if (completed_fraction > best_fraction)
      {
        best_fraction = completed_fraction;
        best_waypoint = failed_at;
        best_spacing = spacing;
        best_metrics = attempt;
        best_reason = reason;
        best_pairs = pairs;
      }
    }
    mergeMetrics(trial, best_metrics);
    accepted_spacing = best_spacing;
    failure_waypoint = best_waypoint;
    fraction = std::max(0.0, best_fraction);
    trial.failure_reason = best_reason;
    trial.collision_pairs = best_pairs;
    return false;
  }

  bool planOmplLocked(const std::string& stage,
                      const moveit::core::RobotState& start,
                      const moveit::core::RobotState& goal,
                      const geometry_msgs::msg::Pose& desired,
                      Trial& trial,
                      Segment& segment)
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "yaw_pitch_locked_zero";
    for (const std::string& joint : { "waist_yaw_joint", "waist_pitch_joint" })
    {
      moveit_msgs::msg::JointConstraint constraint;
      constraint.joint_name = joint;
      constraint.position = 0.0;
      constraint.tolerance_above = 1e-6;
      constraint.tolerance_below = 1e-6;
      constraint.weight = 1.0;
      constraints.joint_constraints.push_back(constraint);
    }
    core_.move_group_->setPathConstraints(constraints);
    const bool result = planOmpl(stage, start, goal, desired, trial, segment);
    core_.move_group_->clearPathConstraints();
    if (!result)
      return false;
    for (const auto& state : segment.states)
    {
      if (std::abs(state.getVariablePosition("waist_yaw_joint")) > 1e-6 ||
          std::abs(state.getVariablePosition("waist_pitch_joint")) > 1e-6)
      {
        trial.failure_reason = "TORSO_LOCK_CONSTRAINT_VIOLATION";
        return false;
      }
    }
    return true;
  }

  bool graspClosure(const moveit::core::RobotState& start, Trial& trial, Segment& segment)
  {
    core_.setFingerTargetContactAllowed(true);
    segment.stage = "GRASP";
    segment.method = "FINGER_ONLY_10_STEP_TASK_SCOPED_TOUCH";
    segment.object_attached = false;
    segment.states = { start };
    const double q0 = start.getVariablePosition("openarm_left_finger_joint1");
    for (int i = 1; i <= 10; ++i)
    {
      moveit::core::RobotState state = start;
      state.setVariablePosition("openarm_left_finger_joint1",
                                q0 + static_cast<double>(i) / 10.0 *
                                  (core_.scene_config_.q_contact - q0));
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
          std::abs(current.getVariablePosition(name) -
                   segment.states.front().getVariablePosition(name)));
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
          if (std::abs(current.getVariablePosition("waist_yaw_joint")) > 1e-6 ||
              std::abs(current.getVariablePosition("waist_pitch_joint")) > 1e-6)
          {
            trial.failure_reason = "TORSO_LOCK_CONSTRAINT_VIOLATION";
            trial.failure_waypoint = static_cast<int>(point);
            return false;
          }
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

  TrialRecord runTrial(int trial_id, double lift, int branch_rank)
  {
    TrialRecord record;
    Trial& trial = record.trial;
    trial.trial_id = trial_id;
    trial.initial_lift = lift;
    trial.branch_rank = branch_rank;
    Candidate candidate;
    candidate.id = "stage_v2_" + std::to_string(trial_id);
    candidate.lift = lift;
    candidate.yaw = candidate.pitch = 0.0;
    core_.resetSceneForCandidate();
    moveit::core::RobotState initial = core_.initialState(candidate);
    const auto grasp_pose = core_.graspPose();
    const auto top_pose = core_.approachPose(core_.scene_config_.rim_clearance);

    // The key change: solve GRASP first, rank the valid solutions, and continue
    // upward.  The accepted state sequence is reversed for the actual descent.
    auto grasp_branches = rankIk(grasp_pose, lift, initial, trial, 100);
    record.grasp_ik_branches = static_cast<int>(grasp_branches.size());
    if (grasp_branches.size() <= static_cast<std::size_t>(branch_rank))
    {
      trial.failure_stage = "GRASP_BRANCH_DISCOVERY";
      trial.failure_reason = "INSUFFICIENT_COLLISION_FREE_GRASP_IK_BRANCHES";
      return record;
    }
    record.branch_seed_id = grasp_branches[branch_rank].seed_id;
    const moveit::core::RobotState grasp_open = grasp_branches[branch_rank].state;

    Trial continuation_metrics;
    Segment upward;
    if (!adaptiveContinuation(grasp_open, grasp_pose, top_pose, false, "REVERSE_IK_DISCOVERY",
                              continuation_metrics, upward, record.accepted_spacing,
                              record.reverse_failure_waypoint, record.reverse_fraction))
    {
      trial.failure_stage = "REVERSE_GRASP_TO_TOP_IK";
      trial.failure_waypoint = record.reverse_failure_waypoint;
      trial.failure_reason = continuation_metrics.failure_reason;
      trial.collision_pairs = continuation_metrics.collision_pairs;
      return record;
    }
    mergeMetrics(trial, continuation_metrics);
    moveit::core::RobotState top_goal = upward.states.back();

    Segment top;
    if (!planOmplLocked("TOP_APPROACH", initial, top_goal, top_pose, trial, top))
    {
      trial.failure_stage = "TOP_APPROACH";
      return record;
    }
    trial.segments.push_back(top);

    Segment descent;
    descent.stage = "VERTICAL_DESCENT";
    descent.method = "REVERSED_DETERMINISTIC_MULTISTART_SEQUENTIAL_IK_MAX_" +
                     std::to_string(record.accepted_spacing) + "M";
    descent.object_attached = false;
    descent.states.assign(upward.states.rbegin(), upward.states.rend());
    // Preserve exact stage continuity with the verified OMPL endpoint.
    descent.states.front() = top.states.back();
    trial.segments.push_back(descent);
    moveit::core::RobotState current = descent.states.back();

    Segment grasp;
    if (!graspClosure(current, trial, grasp))
    {
      trial.failure_stage = "GRASP";
      return record;
    }
    trial.segments.push_back(grasp);
    current = grasp.states.back();
    core_.attachTargetAtomically(current);
    trial.attached_target_in_tcp = core_.attached_target_in_tcp_;

    Segment lift_segment;
    double lift_spacing{};
    int lift_failure_waypoint{};
    double lift_fraction{};
    const auto lift_pose = core_.liftClearPose();
    if (!adaptiveContinuation(current, grasp_pose, lift_pose, true, "LIFT_CLEAR", trial,
                              lift_segment, lift_spacing, lift_failure_waypoint, lift_fraction))
    {
      trial.failure_stage = "LIFT_CLEAR";
      trial.failure_waypoint = lift_failure_waypoint;
      trial.cartesian_fraction = lift_fraction;
      return record;
    }
    trial.segments.push_back(lift_segment);
    current = lift_segment.states.back();

    const auto transfer_pose = core_.transferOutsidePose(trial.attached_target_in_tcp);
    auto transfer_branches = rankIk(transfer_pose, current.getVariablePosition("lift_joint"),
                                    current, trial, 100);
    if (transfer_branches.empty())
    {
      trial.failure_stage = "TRANSFER_OUTSIDE";
      trial.failure_reason = "NO_COLLISION_FREE_TRANSFER_IK";
      return record;
    }
    Segment transfer;
    if (!planOmplLocked("TRANSFER_OUTSIDE", current, transfer_branches.front().state,
                        transfer_pose, trial, transfer))
    {
      trial.failure_stage = "TRANSFER_OUTSIDE";
      return record;
    }
    trial.segments.push_back(transfer);
    trial.cartesian_fraction = 1.0;
    trial.success = denseValidate(trial);
    if (!trial.success && trial.failure_stage.empty())
      trial.failure_stage = "DENSE_VALIDATION";
    return record;
  }

  void initializeTrials() const
  {
    std::ofstream out(trials_csv_, std::ios::trunc);
    out << "timestamp,trial_id,initial_lift,branch_rank,branch_seed_id,ik_seeds_tested,grasp_ik_branches,success,"
           "failure_stage,failure_waypoint,failure_reason,collision_pairs,reverse_fraction,accepted_spacing_m,"
           "min_joint_margin,min_active_revolute_margin,min_self_clearance,min_environment_clearance,"
           "max_position_error,max_orientation_error,planning_time_ms\n";
  }

  void appendTrial(const TrialRecord& r) const
  {
    const Trial& t = r.trial;
    std::ofstream out(trials_csv_, std::ios::app);
    out << csvEscape(timestampNow()) << ',' << t.trial_id << ',' << std::setprecision(15) << t.initial_lift << ','
        << t.branch_rank << ',' << r.branch_seed_id << ',' << t.ik_seeds_tested << ',' << r.grasp_ik_branches << ','
        << (t.success ? 1 : 0) << ',' << csvEscape(t.failure_stage) << ',' << t.failure_waypoint << ','
        << csvEscape(t.failure_reason) << ',' << csvEscape(t.collision_pairs) << ',' << r.reverse_fraction << ','
        << r.accepted_spacing << ',' << t.min_joint_margin << ',' << t.min_active_revolute_margin << ','
        << t.min_self_clearance << ',' << t.min_environment_clearance << ',' << t.max_position_error << ','
        << t.max_orientation_error << ',' << t.planning_time_ms << '\n';
  }

  void writeTrajectory(const Trial& trial)
  {
    std::ofstream out(trajectory_csv_, std::ios::trunc);
    out << "stage,method,point_index";
    for (const auto& name : core_.whole_body_group_->getVariableNames())
      out << ',' << name;
    out << ",tcp_x,tcp_y,tcp_z,tcp_qx,tcp_qy,tcp_qz,tcp_qw,lift,yaw,pitch,object_attached,"
           "joint_limit_margin,self_collision_clearance,environment_clearance,validity\n";
    core_.resetSceneForCandidate();
    for (const auto& segment : trial.segments)
    {
      if (segment.stage == "GRASP")
        core_.setFingerTargetContactAllowed(true);
      for (std::size_t index = 0; index < segment.states.size(); ++index)
      {
        moveit::core::RobotState state = segment.states[index];
        const auto status = core_.checkState(state);
        const auto clearance = core_.stateClearances(state);
        const auto& tcp = state.getGlobalLinkTransform(core_.left_tcp_link_);
        Eigen::Quaterniond q(tcp.rotation());
        q.normalize();
        out << segment.stage << ',' << segment.method << ',' << index;
        for (const auto& name : core_.whole_body_group_->getVariableNames())
          out << ',' << std::setprecision(15) << state.getVariablePosition(name);
        out << ',' << tcp.translation().x() << ',' << tcp.translation().y() << ',' << tcp.translation().z()
            << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
            << state.getVariablePosition("lift_joint") << ','
            << state.getVariablePosition("waist_yaw_joint") << ','
            << state.getVariablePosition("waist_pitch_joint") << ','
            << (segment.object_attached ? 1 : 0) << ',' << jointMargin(state) << ','
            << clearance.second << ',' << clearance.first << ','
            << (status.joint_limit_valid && !status.self_collision && !status.environment_collision ? 1 : 0)
            << '\n';
      }
      if (segment.stage == "GRASP")
        core_.attachTargetAtomically(segment.states.back());
    }
  }

  void writeWaypoints(const Trial& trial) const
  {
    std::ofstream out(waypoints_yaml_, std::ios::trunc);
    const bool low = std::min(trial.min_self_clearance, trial.min_environment_clearance) < 0.001;
    out << "status: STAGE_V2_REVERSE_CONTINUATION_REFERENCE\n"
        << "classification: " << (low ? "VALID_BUT_LOW_CLEARANCE_REFERENCE" : "VALID_REFERENCE") << "\n"
        << "reference_grasp: KINEMATIC_REFERENCE_GRASP_50MM\n"
        << "trajectory_execution_performed: false\n"
        << "initial_lift: " << trial.initial_lift << "\n"
        << "yaw: 0.0\npitch: 0.0\n"
        << "min_joint_limit_margin: " << trial.min_joint_margin << "\n"
        << "min_active_revolute_margin: " << trial.min_active_revolute_margin << "\n"
        << "min_self_collision_clearance: " << trial.min_self_clearance << "\n"
        << "min_environment_clearance: " << trial.min_environment_clearance << "\n"
        << "stages:\n";
    for (const auto& segment : trial.segments)
    {
      const auto& first = segment.states.front();
      const auto& last = segment.states.back();
      const auto& a = first.getGlobalLinkTransform(core_.left_tcp_link_);
      const auto& b = last.getGlobalLinkTransform(core_.left_tcp_link_);
      out << "  " << segment.stage << ":\n    method: " << segment.method
          << "\n    waypoint_count: " << segment.states.size()
          << "\n    start_tcp_xyz: [" << a.translation().x() << ", " << a.translation().y() << ", "
          << a.translation().z() << "]\n    end_tcp_xyz: [" << b.translation().x() << ", "
          << b.translation().y() << ", " << b.translation().z() << "]\n";
    }
    out << "tcp_to_target_attached_transform:\n  xyz: ["
        << trial.attached_target_in_tcp.translation().x() << ", "
        << trial.attached_target_in_tcp.translation().y() << ", "
        << trial.attached_target_in_tcp.translation().z() << "]\n";
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

  void writeNotSelectedArtifacts() const
  {
    std::ofstream trajectory(trajectory_csv_, std::ios::trunc);
    trajectory << "status,reason\nNOT_SELECTED,NO_APPROVABLE_STAGE_V2_REFERENCE\n";
    std::ofstream waypoints(waypoints_yaml_, std::ios::trunc);
    waypoints << "status: NOT_SELECTED\nreason: NO_APPROVABLE_STAGE_V2_REFERENCE\n";
  }

  void writeAudit(const std::vector<TrialRecord>& records, const TrialRecord* selected) const
  {
    std::ofstream out(audit_md_, std::ios::trunc);
    out << "# Stage-constrained reference v2 reference audit\n\n"
           "- Previous stage-constrained and random-reference artifacts were preserved.\n"
           "- Discovery direction: GRASP to TOP_APPROACH; accepted states are reversed for VERTICAL_DESCENT.\n"
           "- Adaptive TCP intervals tested: 0.005, 0.0025, 0.001 m.\n"
           "- Each Cartesian waypoint uses the previous state, followed by 32 deterministic alternate seeds on failure.\n"
           "- Selection requires at least five complete dense-valid candidates.\n"
           "- Yaw and Pitch remain zero, including OMPL path constraints.\n"
           "- No Xacro, SRDF/ACM, joint limit, collision mesh, kinematics, or OMPL configuration was changed.\n\n"
           "|trial|Lift|branch|seed|GRASP branches|result|failure stage|waypoint|reverse fraction|spacing|joint margin|self clearance|environment clearance|pairs|\n"
           "|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---|\n";
    for (const auto& r : records)
    {
      const Trial& t = r.trial;
      out << '|' << t.trial_id << '|' << t.initial_lift << '|' << t.branch_rank << '|'
          << r.branch_seed_id << '|' << r.grasp_ik_branches << '|'
          << (t.success ? "PASS" : t.failure_reason) << '|' << t.failure_stage << '|'
          << t.failure_waypoint << '|' << r.reverse_fraction << '|' << r.accepted_spacing << '|'
          << t.min_active_revolute_margin << '|' << t.min_self_clearance << '|'
          << t.min_environment_clearance << '|' << t.collision_pairs << "|\n";
    }
    out << "\n## Result\n\n";
    if (selected)
    {
      const auto& t = selected->trial;
      const bool low = std::min(t.min_self_clearance, t.min_environment_clearance) < 0.001;
      out << "- Status: `" << (low ? "VALID_BUT_LOW_CLEARANCE_REFERENCE" : "VALID_REFERENCE") << "`\n"
          << "- Selected trial: " << t.trial_id << "\n- Lift seed: " << t.initial_lift
          << " m\n- Minimum active-revolute margin: " << t.min_active_revolute_margin
          << " rad\n- Minimum self-collision clearance: " << t.min_self_clearance
          << " m\n- Minimum environment clearance: " << t.min_environment_clearance
          << " m\n- Maximum orientation error: " << t.max_orientation_error << " rad\n";
    }
    else
    {
      out << "`NO_APPROVABLE_STAGE_V2_REFERENCE`\n\n"
             "No synthetic or partially completed trajectory was promoted as a reference.\n";
    }
    out << "\nTrajectory execution, controllers, ros2_control, and hardware nodes were not used.\n";
  }

  rclcpp::Node::SharedPtr node_;
  ReferenceTrajectoryGenerator core_;
  std::string trials_csv_, trajectory_csv_, waypoints_yaml_, audit_md_;
  bool hold_for_rviz_{ false };
  std::unique_ptr<Trial> selected_;
};
}  // namespace stage_constrained_v2

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("stage_v2_reference_generator", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  int exit_code = 1;
  try
  {
    stage_constrained_v2::Runner runner(node);
    const bool success = runner.run();
    if (success && runner.holdForRviz())
    {
      RCLCPP_INFO(node->get_logger(), "Grasp-seeded reference ready; planning-only hold active.");
      while (rclcpp::ok()) rclcpp::sleep_for(std::chrono::seconds(1));
    }
    exit_code = success ? 0 : 2;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Grasp-seeded generator failed: %s", error.what());
  }
  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
