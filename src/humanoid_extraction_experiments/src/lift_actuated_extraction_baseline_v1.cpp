#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
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
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

// Reuse the audited robot model, scene construction, task-scoped ACM, attachment,
// clearance, and grasp-pose definitions. Existing generators are not edited.
#define private public
#define main preserved_reference_generator_main
#include "reference_trajectory_generator.cpp"
#undef main
#undef private

namespace lift_actuated_baseline_v1
{
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct StateMetrics
{
  bool bounds_valid{ false };
  bool self_collision{ false };
  bool environment_collision{ false };
  double active_margin{ std::numeric_limits<double>::quiet_NaN() };
  double joint3_margin{ std::numeric_limits<double>::quiet_NaN() };
  double joint5_margin{ std::numeric_limits<double>::quiet_NaN() };
  double environment_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double self_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double tcp_x{};
  double tcp_y{};
  double tcp_z{};
  double tcp_qx{};
  double tcp_qy{};
  double tcp_qz{};
  double tcp_qw{ 1.0 };
  double expected_z_error{};
  double xy_error{};
  double orientation_error{};
  double attached_object_min_z{ std::numeric_limits<double>::quiet_NaN() };
  double object_box_top_clearance{ std::numeric_limits<double>::quiet_NaN() };
  std::string collision_pairs;
};

struct GraspCandidate
{
  moveit::core::RobotState state;
  StateMetrics metrics;
  int seed_id{ -1 };
  double score{};

  explicit GraspCandidate(const moveit::core::RobotModelConstPtr& model) : state(model) {}
};

struct Trial
{
  double grasp_lift{};
  int candidate_rank{ -1 };
  int seed_id{ -1 };
  bool success{ false };
  int stage_progress{};
  std::string failure_label;
  std::string failure_stage;
  int failure_waypoint{ -1 };
  std::string collision_pairs;
  double descent_distance{};
  double ascent_distance{};
  double maximum_arm_delta{};
  double min_joint3_margin{ kInfinity };
  double min_joint5_margin{ kInfinity };
  double min_active_margin{ kInfinity };
  double min_environment_clearance{ kInfinity };
  double min_self_clearance{ kInfinity };
  double final_object_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double max_expected_z_error{};
  double max_xy_error{};
  double max_orientation_error{};
};

struct CaseResult
{
  double grasp_lift{};
  int raw_ik_count{};
  int collision_free_ik_count{};
  int retained_ik_count{};
  Trial selected;
  std::vector<Trial> trials;
};

class Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node)
    : node_(node), core_(node)
  {
    const YAML::Node config = YAML::LoadFile(node_->get_parameter("lift_baseline_config").as_string());
    for (const auto& value : config["grasp_lift_candidates_m"])
      lift_candidates_.push_back(value.as<double>());
    ik_seeds_ = config["grasp_search"]["deterministic_ik_seeds"].as<int>();
    duplicate_tolerance_ = config["grasp_search"]["duplicate_joint_distance_rad"].as<double>();
    sample_spacing_ = config["lift_motion"]["maximum_sample_spacing_m"].as<double>();
    safety_clearance_ = config["lift_motion"]["object_bottom_clearance_above_box_top_m"].as<double>();
    arm_lock_tolerance_ = config["strict_arm_locked"]["arm_lock_tolerance_rad"].as<double>();
    tcp_xy_tolerance_ = config["strict_arm_locked"]["tcp_xy_tolerance_m"].as<double>();
    tcp_z_tolerance_ = config["strict_arm_locked"]["tcp_z_translation_tolerance_m"].as<double>();
    tcp_orientation_tolerance_ =
      config["strict_arm_locked"]["tcp_orientation_tolerance_rad"].as<double>();
    maximum_candidates_to_validate_ = config["grasp_search"]["maximum_candidates_to_validate"].as<int>();

    samples_csv_ = node_->get_parameter("lift_baseline_samples_csv").as_string();
    grasp_candidates_csv_ = node_->get_parameter("lift_baseline_grasp_candidates_csv").as_string();
    trials_csv_ = node_->get_parameter("lift_baseline_trials_csv").as_string();
    summary_csv_ = node_->get_parameter("lift_baseline_summary_csv").as_string();
    result_yaml_ = node_->get_parameter("lift_baseline_result_yaml").as_string();
    audit_md_ = node_->get_parameter("lift_baseline_audit_md").as_string();
    initializeOutputs();
  }

  bool run()
  {
    validateModelAndDirection();
    std::vector<CaseResult> results;
    for (const double lift : lift_candidates_)
    {
      RCLCPP_INFO(node_->get_logger(), "LIFT_BASELINE grasp_lift=%.2f STRICT_ARM_LOCKED starting", lift);
      results.push_back(runCase(lift));
      appendSummary(results.back());
      RCLCPP_INFO(node_->get_logger(), "LIFT_BASELINE grasp_lift=%.2f label=%s success=%s",
                  lift, results.back().selected.failure_label.c_str(),
                  results.back().selected.success ? "true" : "false");
    }
    writeResultYaml(results);
    writeAudit(results);
    return true;  // Diagnostic completion is distinct from extraction success.
  }

private:
  double variableMargin(const moveit::core::RobotState& state, const std::string& name) const
  {
    const auto& bounds = core_.robot_model_->getVariableBounds(name);
    if (!bounds.position_bounded_)
      return kInfinity;
    const double value = state.getVariablePosition(name);
    return std::min(value - bounds.min_position_, bounds.max_position_ - value);
  }

  double armDistance(const moveit::core::RobotState& first,
                     const moveit::core::RobotState& second) const
  {
    double squared = 0.0;
    for (const auto& name : core_.left_arm_group_->getVariableNames())
    {
      const double delta = second.getVariablePosition(name) - first.getVariablePosition(name);
      squared += delta * delta;
    }
    return std::sqrt(squared);
  }

  double attachedObjectMinimumZ(const moveit::core::RobotState& state) const
  {
    const Eigen::Isometry3d object_world = state.getGlobalLinkTransform(core_.left_tcp_link_) *
                                           core_.attached_target_in_tcp_;
    const Eigen::Vector3d half(core_.scene_config_.target_size[0] / 2.0,
                               core_.scene_config_.target_size[1] / 2.0,
                               core_.scene_config_.target_size[2] / 2.0);
    double minimum = kInfinity;
    for (const double x : { -half.x(), half.x() })
      for (const double y : { -half.y(), half.y() })
        for (const double z : { -half.z(), half.z() })
          minimum = std::min(minimum, (object_world * Eigen::Vector3d(x, y, z)).z());
    return minimum;
  }

  StateMetrics evaluate(moveit::core::RobotState& state, const Eigen::Isometry3d& expected_tcp,
                        bool attached) const
  {
    state.update();
    const CollisionStatus status = core_.checkState(state);
    const auto clearance = core_.stateClearances(state);
    const Eigen::Isometry3d& tcp = state.getGlobalLinkTransform(core_.left_tcp_link_);
    StateMetrics metrics;
    metrics.bounds_valid = status.joint_limit_valid;
    metrics.self_collision = status.self_collision;
    metrics.environment_collision = status.environment_collision;
    metrics.active_margin = kInfinity;
    for (const auto& name : core_.left_arm_with_torso_group_->getVariableNames())
      metrics.active_margin = std::min(metrics.active_margin, variableMargin(state, name));
    metrics.joint3_margin = variableMargin(state, "openarm_left_joint3");
    metrics.joint5_margin = variableMargin(state, "openarm_left_joint5");
    metrics.environment_clearance = clearance.first;
    metrics.self_clearance = clearance.second;
    metrics.tcp_x = tcp.translation().x();
    metrics.tcp_y = tcp.translation().y();
    metrics.tcp_z = tcp.translation().z();
    const Eigen::Quaterniond quaternion(tcp.rotation());
    metrics.tcp_qx = quaternion.x();
    metrics.tcp_qy = quaternion.y();
    metrics.tcp_qz = quaternion.z();
    metrics.tcp_qw = quaternion.w();
    metrics.expected_z_error = metrics.tcp_z - expected_tcp.translation().z();
    metrics.xy_error = (tcp.translation().head<2>() - expected_tcp.translation().head<2>()).norm();
    metrics.orientation_error = quaternion.angularDistance(Eigen::Quaterniond(expected_tcp.rotation()));
    metrics.collision_pairs = pairString(status.pairs);
    if (attached)
    {
      metrics.attached_object_min_z = attachedObjectMinimumZ(state);
      const double box_top = core_.scene_config_.box_center[2] + core_.scene_config_.box_height / 2.0;
      metrics.object_box_top_clearance = metrics.attached_object_min_z - box_top;
    }
    return metrics;
  }

  bool stateValid(const StateMetrics& metrics) const
  {
    return metrics.bounds_valid && !metrics.self_collision && !metrics.environment_collision;
  }

  void updateTrial(Trial& trial, const StateMetrics& metrics, double arm_delta)
  {
    trial.maximum_arm_delta = std::max(trial.maximum_arm_delta, arm_delta);
    trial.min_joint3_margin = std::min(trial.min_joint3_margin, metrics.joint3_margin);
    trial.min_joint5_margin = std::min(trial.min_joint5_margin, metrics.joint5_margin);
    trial.min_active_margin = std::min(trial.min_active_margin, metrics.active_margin);
    trial.min_environment_clearance = std::min(trial.min_environment_clearance,
                                               metrics.environment_clearance);
    trial.min_self_clearance = std::min(trial.min_self_clearance, metrics.self_clearance);
    trial.max_expected_z_error = std::max(trial.max_expected_z_error,
                                          std::abs(metrics.expected_z_error));
    trial.max_xy_error = std::max(trial.max_xy_error, metrics.xy_error);
    trial.max_orientation_error = std::max(trial.max_orientation_error, metrics.orientation_error);
    if (std::isfinite(metrics.object_box_top_clearance))
      trial.final_object_clearance = metrics.object_box_top_clearance;
  }

  void writeSample(double lift_candidate, int candidate_rank, const std::string& stage,
                   int waypoint, double stage_start_lift, const moveit::core::RobotState& state,
                   const moveit::core::RobotState& previous, const StateMetrics& metrics,
                   const std::string& failure_label) const
  {
    std::ofstream out(samples_csv_, std::ios::app);
    const double maximum_arm_delta = armDistance(previous, state);
    out << std::setprecision(15) << lift_candidate << ',' << candidate_rank << ',' << stage << ','
        << waypoint << ',' << state.getVariablePosition("lift_joint") << ','
        << state.getVariablePosition("lift_joint") - stage_start_lift << ','
        << state.getVariablePosition("waist_yaw_joint") << ','
        << state.getVariablePosition("waist_pitch_joint");
    for (const auto& name : core_.left_arm_group_->getVariableNames())
      out << ',' << state.getVariablePosition(name);
    for (const auto& name : core_.left_arm_group_->getVariableNames())
      out << ',' << state.getVariablePosition(name) - previous.getVariablePosition(name);
    out << ',' << maximum_arm_delta << ',' << metrics.tcp_x << ',' << metrics.tcp_y << ',' << metrics.tcp_z
        << ',' << metrics.tcp_qx << ',' << metrics.tcp_qy << ',' << metrics.tcp_qz << ',' << metrics.tcp_qw
        << ',' << metrics.expected_z_error << ',' << metrics.xy_error << ',' << metrics.orientation_error
        << ',' << metrics.joint3_margin << ',' << metrics.joint5_margin << ',' << metrics.active_margin
        << ',' << metrics.environment_clearance << ',' << metrics.self_clearance << ','
        << metrics.attached_object_min_z << ',' << metrics.object_box_top_clearance << ','
        << ((!metrics.bounds_valid || metrics.self_collision || metrics.environment_collision) ? 1 : 0)
        << ',' << csvEscape(metrics.collision_pairs) << ',' << csvEscape(failure_label) << '\n';
  }

  moveit::core::RobotState randomArmSeed(const moveit::core::RobotState& base,
                                         double lift, int seed_id) const
  {
    moveit::core::RobotState state = base;
    if (seed_id > 0)
    {
      std::mt19937_64 rng(202608160000ULL +
                          static_cast<std::uint64_t>(std::llround(lift * 1e6)) * 1009ULL +
                          static_cast<std::uint64_t>(seed_id));
      for (const auto& name : core_.left_arm_group_->getVariableNames())
      {
        const auto& bounds = core_.robot_model_->getVariableBounds(name);
        std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
        state.setVariablePosition(name, distribution(rng));
      }
    }
    state.setVariablePosition("lift_joint", lift);
    state.setVariablePosition("waist_yaw_joint", 0.0);
    state.setVariablePosition("waist_pitch_joint", 0.0);
    state.setVariablePosition("openarm_left_finger_joint1", core_.scene_config_.left_finger);
    state.update();
    return state;
  }

  std::vector<GraspCandidate> searchGrasp(double lift, CaseResult& result)
  {
    core_.resetSceneForCandidate();
    Candidate candidate;
    candidate.id = "lift_actuated_grasp_search";
    candidate.lift = lift;
    candidate.yaw = candidate.pitch = 0.0;
    const moveit::core::RobotState base = core_.initialState(candidate);
    const geometry_msgs::msg::Pose grasp_pose = core_.graspPose();
    Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
    expected.translation() = Eigen::Vector3d(grasp_pose.position.x, grasp_pose.position.y,
                                              grasp_pose.position.z);
    expected.linear() = Eigen::Quaterniond(grasp_pose.orientation.w, grasp_pose.orientation.x,
                                            grasp_pose.orientation.y, grasp_pose.orientation.z).toRotationMatrix();
    std::vector<GraspCandidate> retained;
    for (int seed_id = 0; seed_id < ik_seeds_; ++seed_id)
    {
      moveit::core::RobotState state = randomArmSeed(base, lift, seed_id);
      const bool ik = state.setFromIK(core_.left_arm_group_, grasp_pose, core_.left_tcp_link_,
                                      core_.scene_config_.ik_timeout);
      if (!ik)
      {
        writeGraspCandidate(lift, seed_id, false, false, false, nullptr, nullptr, 0.0);
        continue;
      }
      ++result.raw_ik_count;
      StateMetrics metrics = evaluate(state, expected, false);
      const bool collision_free = stateValid(metrics);
      if (collision_free)
        ++result.collision_free_ik_count;
      const bool positive_margin = collision_free && metrics.active_margin > 1e-9 &&
        metrics.joint3_margin > 1e-9 && metrics.joint5_margin > 1e-9;
      GraspCandidate grasp(core_.robot_model_);
      grasp.state = state;
      grasp.metrics = metrics;
      grasp.seed_id = seed_id;
      const double clearance = std::min(metrics.environment_clearance, metrics.self_clearance);
      grasp.score = std::min(metrics.joint3_margin,
                             std::min(metrics.joint5_margin, metrics.active_margin)) +
                    0.05 * std::max(0.0, clearance);
      bool duplicate = false;
      if (positive_margin)
        duplicate = std::any_of(retained.begin(), retained.end(), [&](const auto& existing) {
          return armDistance(existing.state, grasp.state) <= duplicate_tolerance_;
        });
      if (positive_margin && !duplicate)
        retained.push_back(grasp);
      writeGraspCandidate(lift, seed_id, true, collision_free,
                          positive_margin && !duplicate, &state, &metrics, grasp.score);
    }
    std::stable_sort(retained.begin(), retained.end(), [](const auto& first, const auto& second) {
      if (std::abs(first.metrics.active_margin - second.metrics.active_margin) > 1e-12)
        return first.metrics.active_margin > second.metrics.active_margin;
      if (std::abs(first.metrics.environment_clearance - second.metrics.environment_clearance) > 1e-12)
        return first.metrics.environment_clearance > second.metrics.environment_clearance;
      if (std::abs(first.metrics.self_clearance - second.metrics.self_clearance) > 1e-12)
        return first.metrics.self_clearance > second.metrics.self_clearance;
      return first.seed_id < second.seed_id;
    });
    if (maximum_candidates_to_validate_ > 0 &&
        retained.size() > static_cast<std::size_t>(maximum_candidates_to_validate_))
      retained.erase(retained.begin() + maximum_candidates_to_validate_, retained.end());
    result.retained_ik_count = static_cast<int>(retained.size());
    return retained;
  }

  void writeGraspCandidate(double lift, int seed_id, bool raw_ik, bool collision_free,
                           bool retained, const moveit::core::RobotState* state,
                           const StateMetrics* metrics, double score) const
  {
    std::ofstream out(grasp_candidates_csv_, std::ios::app);
    out << std::setprecision(15) << lift << ',' << seed_id << ',' << (raw_ik ? 1 : 0) << ','
        << (collision_free ? 1 : 0) << ',' << (retained ? 1 : 0) << ',' << score;
    if (metrics)
      out << ',' << metrics->joint3_margin << ',' << metrics->joint5_margin << ','
          << metrics->active_margin << ',' << metrics->environment_clearance << ','
          << metrics->self_clearance << ',' << metrics->xy_error << ','
          << metrics->expected_z_error << ',' << metrics->orientation_error << ','
          << csvEscape(metrics->collision_pairs);
    else
      out << ",nan,nan,nan,nan,nan,nan,nan,nan,\"\"";
    for (const auto& name : core_.left_arm_group_->getVariableNames())
      out << ',' << (state ? std::to_string(state->getVariablePosition(name)) : std::string());
    out << '\n';
  }

  bool checkStrictInvariants(const moveit::core::RobotState& locked_arm,
                             const moveit::core::RobotState& state,
                             const StateMetrics& metrics, Trial& trial,
                             const std::string& stage, int waypoint)
  {
    const double arm_delta = armDistance(locked_arm, state);
    updateTrial(trial, metrics, arm_delta);
    if (arm_delta > arm_lock_tolerance_ ||
        std::abs(state.getVariablePosition("waist_yaw_joint")) > 1e-12 ||
        std::abs(state.getVariablePosition("waist_pitch_joint")) > 1e-12 ||
        metrics.xy_error > tcp_xy_tolerance_ ||
        std::abs(metrics.expected_z_error) > tcp_z_tolerance_ ||
        metrics.orientation_error > tcp_orientation_tolerance_)
    {
      trial.failure_label = "ARM_COMPENSATION_CONNECTIVITY_FAILURE";
      trial.failure_stage = stage;
      trial.failure_waypoint = waypoint;
      return false;
    }
    return true;
  }

  bool validateLiftStage(const std::string& stage, const std::string& limit_label,
                         const std::string& collision_label, double lift_candidate,
                         int candidate_rank, const moveit::core::RobotState& locked_arm,
                         const moveit::core::RobotState& initial, double target_lift,
                         bool attached, Trial& trial, moveit::core::RobotState& final)
  {
    const double start_lift = initial.getVariablePosition("lift_joint");
    const auto& lift_bounds = core_.robot_model_->getVariableBounds("lift_joint");
    if (target_lift < lift_bounds.min_position_ - 1e-12 ||
        target_lift > lift_bounds.max_position_ + 1e-12)
    {
      trial.failure_label = limit_label;
      trial.failure_stage = stage;
      trial.failure_waypoint = 0;
      return false;
    }
    const int intervals = std::max(1, static_cast<int>(std::ceil(
      std::abs(target_lift - start_lift) / sample_spacing_)));
    moveit::core::RobotState previous = initial;
    const Eigen::Isometry3d reference_tcp = initial.getGlobalLinkTransform(core_.left_tcp_link_);
    for (int waypoint = 0; waypoint <= intervals; ++waypoint)
    {
      const double ratio = static_cast<double>(waypoint) / intervals;
      moveit::core::RobotState state = initial;
      const double lift = start_lift + ratio * (target_lift - start_lift);
      state.setVariablePosition("lift_joint", lift);
      state.setVariablePosition("waist_yaw_joint", 0.0);
      state.setVariablePosition("waist_pitch_joint", 0.0);
      state.update();
      Eigen::Isometry3d expected_tcp = reference_tcp;
      expected_tcp.translation().z() = reference_tcp.translation().z() - (lift - start_lift);
      StateMetrics metrics = evaluate(state, expected_tcp, attached);
      const bool strict_valid = checkStrictInvariants(locked_arm, state, metrics, trial, stage, waypoint);
      std::string label;
      if (!metrics.bounds_valid)
        label = limit_label;
      else if (metrics.self_collision || metrics.environment_collision)
        label = collision_label;
      else if (!strict_valid)
        label = trial.failure_label;
      writeSample(lift_candidate, candidate_rank, stage, waypoint, start_lift,
                  state, waypoint == 0 ? state : previous, metrics, label);
      if (!label.empty())
      {
        trial.failure_label = label;
        trial.failure_stage = stage;
        trial.failure_waypoint = waypoint;
        trial.collision_pairs = metrics.collision_pairs;
        return false;
      }
      previous = state;
      final = state;
    }
    return true;
  }

  bool createGrasp(double lift_candidate, int candidate_rank,
                   const moveit::core::RobotState& locked_arm,
                   const moveit::core::RobotState& initial, Trial& trial,
                   moveit::core::RobotState& grasped)
  {
    core_.setFingerTargetContactAllowed(true);
    const double start_finger = initial.getVariablePosition("openarm_left_finger_joint1");
    moveit::core::RobotState previous = initial;
    const Eigen::Isometry3d expected_tcp = initial.getGlobalLinkTransform(core_.left_tcp_link_);
    for (int waypoint = 0; waypoint <= 10; ++waypoint)
    {
      moveit::core::RobotState state = initial;
      state.setVariablePosition("openarm_left_finger_joint1", start_finger +
        static_cast<double>(waypoint) / 10.0 * (core_.scene_config_.q_contact - start_finger));
      state.update();
      StateMetrics metrics = evaluate(state, expected_tcp, false);
      const bool strict_valid = checkStrictInvariants(locked_arm, state, metrics, trial, "GRASP", waypoint);
      std::string label;
      if (!stateValid(metrics))
        label = "GRASP_GEOMETRY_FAILURE";
      else if (!strict_valid)
        label = trial.failure_label;
      writeSample(lift_candidate, candidate_rank, "GRASP", waypoint,
                  initial.getVariablePosition("lift_joint"), state,
                  waypoint == 0 ? state : previous, metrics, label);
      if (!label.empty())
      {
        trial.failure_label = label;
        trial.failure_stage = "GRASP";
        trial.failure_waypoint = waypoint;
        trial.collision_pairs = metrics.collision_pairs;
        return false;
      }
      previous = state;
      grasped = state;
    }
    if (std::abs(grasped.getVariablePosition("openarm_left_finger_joint1") -
                 core_.scene_config_.q_contact) > 1e-12)
    {
      trial.failure_label = "GRASP_GEOMETRY_FAILURE";
      trial.failure_stage = "GRASP";
      return false;
    }
    core_.attachTargetAtomically(grasped);
    return true;
  }

  Trial runLockedTrial(double grasp_lift, int candidate_rank,
                       const GraspCandidate& grasp_candidate)
  {
    Trial trial;
    trial.grasp_lift = grasp_lift;
    trial.candidate_rank = candidate_rank;
    trial.seed_id = grasp_candidate.seed_id;
    core_.resetSceneForCandidate();

    const double box_top = core_.scene_config_.box_center[2] + core_.scene_config_.box_height / 2.0;
    const double object_bottom = core_.scene_config_.target_position[2] -
                                 core_.scene_config_.target_size[2] / 2.0;
    const double required_rise = box_top + safety_clearance_ - object_bottom;
    const double upper_lift = grasp_lift - required_rise;
    trial.descent_distance = required_rise;
    trial.ascent_distance = required_rise;

    const auto& bounds = core_.robot_model_->getVariableBounds("lift_joint");
    if (upper_lift < bounds.min_position_ - 1e-12 || upper_lift > bounds.max_position_ + 1e-12)
    {
      trial.failure_label = "LIFT_DESCENT_LIMIT_FAILURE";
      trial.failure_stage = "LIFT_VERTICAL_DESCENT";
      return trial;
    }

    moveit::core::RobotState locked_arm = grasp_candidate.state;
    locked_arm.setVariablePosition("lift_joint", grasp_lift);
    locked_arm.setVariablePosition("waist_yaw_joint", 0.0);
    locked_arm.setVariablePosition("waist_pitch_joint", 0.0);
    locked_arm.setVariablePosition("openarm_left_finger_joint1", core_.scene_config_.left_finger);
    locked_arm.update();

    moveit::core::RobotState descent_start = locked_arm;
    descent_start.setVariablePosition("lift_joint", upper_lift);
    descent_start.update();
    moveit::core::RobotState at_grasp = descent_start;
    if (!validateLiftStage("LIFT_VERTICAL_DESCENT", "LIFT_DESCENT_LIMIT_FAILURE",
                           "LIFT_DESCENT_COLLISION_FAILURE", grasp_lift, candidate_rank,
                           locked_arm, descent_start, grasp_lift, false, trial, at_grasp))
    {
      trial.stage_progress = 1;
      return trial;
    }
    trial.stage_progress = 2;

    moveit::core::RobotState grasped = at_grasp;
    if (!createGrasp(grasp_lift, candidate_rank, locked_arm, at_grasp, trial, grasped))
    {
      trial.stage_progress = 2;
      return trial;
    }
    trial.stage_progress = 3;

    moveit::core::RobotState cleared = grasped;
    if (!validateLiftStage("LIFT_ACTUATED_CLEARANCE", "LIFT_ASCENT_LIMIT_FAILURE",
                           "LIFT_ASCENT_COLLISION_FAILURE", grasp_lift, candidate_rank,
                           locked_arm, grasped, upper_lift, true, trial, cleared))
    {
      trial.stage_progress = 3;
      return trial;
    }
    trial.stage_progress = 4;
    const double final_min_z = attachedObjectMinimumZ(cleared);
    trial.final_object_clearance = final_min_z - box_top;
    if (trial.final_object_clearance + 1e-9 < safety_clearance_)
    {
      trial.failure_label = "ATTACHED_OBJECT_CLEARANCE_FAILURE";
      trial.failure_stage = "LIFT_ACTUATED_CLEARANCE";
      return trial;
    }
    trial.success = true;
    trial.failure_label = "LIFT_ACTUATED_EXTRACTION_SUCCESS";
    trial.failure_stage.clear();
    trial.failure_waypoint = -1;
    trial.stage_progress = 5;
    return trial;
  }

  CaseResult runCase(double lift)
  {
    CaseResult result;
    result.grasp_lift = lift;
    const auto candidates = searchGrasp(lift, result);
    if (candidates.empty())
    {
      result.selected.grasp_lift = lift;
      result.selected.failure_label = "GRASP_CONFIGURATION_IK_FAILURE";
      result.selected.failure_stage = "GRASP_CONFIGURATION_SEARCH";
      return result;
    }
    const int count = static_cast<int>(candidates.size());
    for (int rank = 0; rank < count; ++rank)
    {
      Trial trial = runLockedTrial(lift, rank, candidates[rank]);
      appendTrial(trial);
      result.trials.push_back(trial);
      if (result.trials.size() == 1 || trial.stage_progress > result.selected.stage_progress)
        result.selected = trial;
      if (trial.success)
      {
        result.selected = trial;
        break;
      }
    }
    return result;
  }

  void validateModelAndDirection()
  {
    const auto& bounds = core_.robot_model_->getVariableBounds("lift_joint");
    if (!bounds.position_bounded_ || std::abs(bounds.min_position_) > 1e-12 ||
        std::abs(bounds.max_position_ - 0.7) > 1e-12)
      throw std::runtime_error("Unexpected lift_joint bounds; expected [0.0, 0.7] m");
    Candidate candidate;
    candidate.id = "lift_axis_audit";
    candidate.lift = 0.35;
    moveit::core::RobotState first = core_.initialState(candidate);
    moveit::core::RobotState second = first;
    second.setVariablePosition("lift_joint", 0.351);
    second.update();
    const Eigen::Vector3d displacement = second.getGlobalLinkTransform(core_.left_tcp_link_).translation() -
                                         first.getGlobalLinkTransform(core_.left_tcp_link_).translation();
    measured_lift_dx_ = displacement.x();
    measured_lift_dy_ = displacement.y();
    measured_lift_dz_ = displacement.z();
    if (std::abs(measured_lift_dx_) > 1e-9 || std::abs(measured_lift_dy_) > 1e-9 ||
        std::abs(measured_lift_dz_ + 0.001) > 1e-9)
      throw std::runtime_error("lift_joint direction audit failed: +1 mm did not move TCP world Z by -1 mm");
  }

  void initializeOutputs() const
  {
    {
      std::ofstream out(samples_csv_, std::ios::trunc);
      out << "lift_candidate,candidate_rank,stage,waypoint_index,lift_joint,lift_displacement,yaw,pitch";
      for (const auto& name : core_.left_arm_group_->getVariableNames())
        out << ',' << name;
      for (const auto& name : core_.left_arm_group_->getVariableNames())
        out << ",delta_" << name;
      out << ",max_arm_delta,tcp_x,tcp_y,tcp_z,tcp_qx,tcp_qy,tcp_qz,tcp_qw,expected_tcp_z_error,"
             "tcp_xy_error,tcp_orientation_error,joint3_margin,joint5_margin,active_joint_min_margin,"
             "environment_clearance,self_clearance,attached_object_min_z,object_clearance_above_box_top,"
             "collision,collision_pairs,failure_label\n";
    }
    {
      std::ofstream out(grasp_candidates_csv_, std::ios::trunc);
      out << "lift,seed_id,raw_ik,collision_free,retained,score,joint3_margin,joint5_margin,"
             "active_joint_min_margin,environment_clearance,self_clearance,tcp_xy_error,tcp_z_error,"
             "tcp_orientation_error,collision_pairs";
      for (const auto& name : core_.left_arm_group_->getVariableNames())
        out << ',' << name;
      out << '\n';
    }
    {
      std::ofstream out(trials_csv_, std::ios::trunc);
      out << "lift,candidate_rank,seed_id,success,failure_label,failure_stage,failure_waypoint,"
             "descent_distance,ascent_distance,max_arm_delta,min_joint3_margin,min_joint5_margin,"
             "min_active_margin,min_environment_clearance,min_self_clearance,final_object_clearance,"
             "max_expected_z_error,max_xy_error,max_orientation_error,collision_pairs\n";
    }
    {
      std::ofstream out(summary_csv_, std::ios::trunc);
      out << "lift,raw_ik_count,collision_free_ik_count,retained_ik_count,trials_run,strict_success,"
             "selected_candidate_rank,selected_seed_id,failure_label,failure_stage,failure_waypoint,"
             "descent_distance,ascent_distance,max_arm_delta,min_joint3_margin,min_joint5_margin,"
             "min_active_margin,min_environment_clearance,min_self_clearance,final_object_clearance,"
             "max_expected_z_error,max_xy_error,max_orientation_error,collision_pairs\n";
    }
  }

  void appendTrial(const Trial& trial) const
  {
    std::ofstream out(trials_csv_, std::ios::app);
    out << std::setprecision(15) << trial.grasp_lift << ',' << trial.candidate_rank << ',' << trial.seed_id << ','
        << (trial.success ? 1 : 0) << ',' << trial.failure_label << ',' << trial.failure_stage << ','
        << trial.failure_waypoint << ',' << trial.descent_distance << ',' << trial.ascent_distance << ','
        << trial.maximum_arm_delta << ',' << trial.min_joint3_margin << ',' << trial.min_joint5_margin << ','
        << trial.min_active_margin << ',' << trial.min_environment_clearance << ',' << trial.min_self_clearance
        << ',' << trial.final_object_clearance << ',' << trial.max_expected_z_error << ',' << trial.max_xy_error
        << ',' << trial.max_orientation_error << ',' << csvEscape(trial.collision_pairs) << '\n';
  }

  void appendSummary(const CaseResult& result) const
  {
    const Trial& trial = result.selected;
    std::ofstream out(summary_csv_, std::ios::app);
    out << std::setprecision(15) << result.grasp_lift << ',' << result.raw_ik_count << ','
        << result.collision_free_ik_count << ',' << result.retained_ik_count << ',' << result.trials.size() << ','
        << (trial.success ? 1 : 0) << ',' << trial.candidate_rank << ',' << trial.seed_id << ','
        << trial.failure_label << ',' << trial.failure_stage << ',' << trial.failure_waypoint << ','
        << trial.descent_distance << ',' << trial.ascent_distance << ',' << trial.maximum_arm_delta << ','
        << trial.min_joint3_margin << ',' << trial.min_joint5_margin << ',' << trial.min_active_margin << ','
        << trial.min_environment_clearance << ',' << trial.min_self_clearance << ','
        << trial.final_object_clearance << ',' << trial.max_expected_z_error << ',' << trial.max_xy_error << ','
        << trial.max_orientation_error << ',' << csvEscape(trial.collision_pairs) << '\n';
  }

  void writeResultYaml(const std::vector<CaseResult>& results) const
  {
    std::ofstream out(result_yaml_, std::ios::trunc);
    out << "protocol: LIFT_ACTUATED_EXTRACTION_BASELINE_V1\nmode: STRICT_ARM_LOCKED\n"
           "planning_only: true\ntrajectory_execution_performed: false\nrviz_started: false\n"
           "yaw_rad: 0.0\npitch_rad: 0.0\nresults:\n";
    for (const auto& result : results)
    {
      const Trial& trial = result.selected;
      out << "  - grasp_lift_m: " << result.grasp_lift << "\n    label: " << trial.failure_label
          << "\n    success: " << (trial.success ? "true" : "false")
          << "\n    descent_distance_m: " << trial.descent_distance
          << "\n    ascent_distance_m: " << trial.ascent_distance
          << "\n    maximum_arm_delta_rad: " << trial.maximum_arm_delta
          << "\n    final_object_clearance_m: " << trial.final_object_clearance << '\n';
    }
  }

  void writeAudit(const std::vector<CaseResult>& results) const
  {
    std::ofstream out(audit_md_, std::ios::trunc);
    out << "# Lift-actuated extraction baseline v1 audit\n\nGenerated: " << timestampNow() << "\n\n"
           "## Existing implementation audit\n\n"
           "- v1 `cartesian()` held the trial Lift value and called `setFromIK(left_arm_group_)` for every "
           "VERTICAL_DESCENT and LIFT_CLEAR Cartesian Z waypoint.\n"
           "- v2 `followCartesian()` / `adaptiveContinuation()` likewise kept Lift at the grasp candidate and "
           "used sequential Arm IK for reverse descent discovery and attached LIFT_CLEAR.\n"
           "- v3 set Lift to 0.35 or 0.40 at every layer and generated each Cartesian Z layer with Arm IK.\n"
           "- torso recovery kept Lift fixed and added Yaw/Pitch while Arm IK still generated Cartesian Z layers.\n"
           "- The focused boundary audit identified openarm_left_joint3 and openarm_left_joint5 upper-limit "
           "termination during that Arm-actuated vertical motion.\n\n"
           "## Corrected baseline\n\n"
           "Only grasp configuration uses Arm IK. STRICT_ARM_LOCKED then changes only lift_joint at <=1 mm "
           "spacing for descent and attached ascent. No OMPL planning request, controller, ros2_control, RViz, "
           "hardware, or trajectory execution was used. Force closure is not claimed.\n\n"
           "Lift direction audit for +0.001 m: TCP delta xyz = " << measured_lift_dx_ << ' '
        << measured_lift_dy_ << ' ' << measured_lift_dz_ << " m.\n\n"
           "|Grasp Lift|STRICT result|Descent / ascent (m)|Max Arm delta|j3 / j5 minimum margin|"
           "Environment / self minimum clearance|Object-bottom clearance|First failure|Pairs|\n"
           "|---:|---|---:|---:|---:|---:|---:|---|---|\n";
    for (const auto& result : results)
    {
      const Trial& trial = result.selected;
      out << '|' << result.grasp_lift << '|' << trial.failure_label << '|'
          << trial.descent_distance << " / " << trial.ascent_distance << '|'
          << trial.maximum_arm_delta << '|' << trial.min_joint3_margin << " / "
          << trial.min_joint5_margin << '|' << trial.min_environment_clearance << " / "
          << trial.min_self_clearance << '|' << trial.final_object_clearance << '|'
          << trial.failure_stage << ':' << trial.failure_waypoint << '|'
          << trial.collision_pairs << "|\n";
    }
    out << "\nValidation stops when the attached object clears the box top; TRANSFER_OUTSIDE and the complete "
           "five-stage experiment are intentionally excluded.\n";
  }

  rclcpp::Node::SharedPtr node_;
  ReferenceTrajectoryGenerator core_;
  std::vector<double> lift_candidates_;
  int ik_seeds_{};
  int maximum_candidates_to_validate_{};
  double duplicate_tolerance_{};
  double sample_spacing_{};
  double safety_clearance_{};
  double arm_lock_tolerance_{};
  double tcp_xy_tolerance_{};
  double tcp_z_tolerance_{};
  double tcp_orientation_tolerance_{};
  double measured_lift_dx_{};
  double measured_lift_dy_{};
  double measured_lift_dz_{};
  std::string samples_csv_;
  std::string grasp_candidates_csv_;
  std::string trials_csv_;
  std::string summary_csv_;
  std::string result_yaml_;
  std::string audit_md_;
};
}  // namespace lift_actuated_baseline_v1

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("lift_actuated_extraction_baseline_v1", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  int exit_code = 1;
  std::unique_ptr<lift_actuated_baseline_v1::Runner> runner;
  try
  {
    runner = std::make_unique<lift_actuated_baseline_v1::Runner>(node);
    exit_code = runner->run() ? 0 : 2;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Lift-actuated baseline failed: %s", error.what());
  }
  executor.cancel();
  if (spin_thread.joinable())
    spin_thread.join();
  runner.reset();
  rclcpp::shutdown();
  return exit_code;
}
