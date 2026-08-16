#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

// Reuse the audited scene, robot state, collision, clearance, and TCP pose code.
// The reference generators and the focused v3 boundary audit remain untouched.
#define private public
#define main preserved_reference_generator_main
#include "reference_trajectory_generator.cpp"
#undef main
#undef private

namespace torso_recovery_v1
{
constexpr double kPiLocal = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

double radians(double degrees)
{
  return degrees * kPiLocal / 180.0;
}

double degrees(double radians_value)
{
  return radians_value * 180.0 / kPiLocal;
}

struct BoundaryCase
{
  double lift{};
  int previous_waypoint{};
  int failed_waypoint{};
  double previous_z{};
  double failed_z{};
  double refined_lower_z{};
  double refined_upper_z{};
  std::vector<double> arm_start;
};

struct Metrics
{
  bool bounds_valid{ false };
  bool collision_free{ false };
  double joint3_margin{ std::numeric_limits<double>::quiet_NaN() };
  double joint5_margin{ std::numeric_limits<double>::quiet_NaN() };
  double active_margin{ std::numeric_limits<double>::quiet_NaN() };
  double environment_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double self_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double position_error{ std::numeric_limits<double>::quiet_NaN() };
  double orientation_error{ std::numeric_limits<double>::quiet_NaN() };
  std::string failure;
  std::string collision_pairs;
};

struct StaticCandidate
{
  moveit::core::RobotState state;
  Metrics metrics;
  double yaw{};
  double pitch{};
  double cost{ kInf };
  bool refined{ false };
  int seed_id{ -1 };

  explicit StaticCandidate(const moveit::core::RobotModelConstPtr& model) : state(model) {}
};

struct GraphNode
{
  moveit::core::RobotState state;
  Metrics metrics;
  double cost{ kInf };
  int parent{ -1 };
  int schedule_id{ -1 };
  bool reachable{ false };
  double edge_cost{};

  explicit GraphNode(const moveit::core::RobotModelConstPtr& model) : state(model) {}
};

struct EdgeAudit
{
  bool valid{ false };
  double displacement{};
  double edge_cost{};
  double min_active_margin{ kInf };
  double min_environment_clearance{ kInf };
  double min_self_clearance{ kInf };
  double max_position_error{};
  double max_orientation_error{};
  std::string failure;
  std::string collision_pairs;
};

struct ModeResult
{
  std::string mode;
  std::string label;
  bool success{ false };
  int static_raw_ik{};
  int static_collision_free{};
  int static_margin_positive{};
  int first_failed_layer{ -1 };
  int first_recovery_layer{ -1 };
  int reconnect_layer{ -1 };
  int evaluated_edges{};
  int connected_edges{};
  double selected_yaw{};
  double selected_pitch{};
  double path_length{};
  double min_joint3_margin{ kInf };
  double min_joint5_margin{ kInf };
  double min_active_margin{ kInf };
  double min_environment_clearance{ kInf };
  double min_self_clearance{ kInf };
  std::string failure_reason;
  std::string collision_pairs;
  std::vector<std::vector<GraphNode>> layers;
  std::vector<int> path_indices;
};

class Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node)
    : node_(node), core_(node)
  {
    const YAML::Node config = YAML::LoadFile(node_->get_parameter("torso_recovery_config").as_string());
    yaw_min_ = radians(config["search"]["yaw_min_deg"].as<double>());
    yaw_max_ = radians(config["search"]["yaw_max_deg"].as<double>());
    pitch_min_ = radians(config["search"]["pitch_min_deg"].as<double>());
    pitch_max_ = radians(config["search"]["pitch_max_deg"].as<double>());
    coarse_step_ = radians(config["search"]["coarse_step_deg"].as<double>());
    fine_step_ = radians(config["search"]["fine_step_deg"].as<double>());
    refine_radius_ = radians(config["search"]["refine_radius_deg"].as<double>());
    static_seeds_ = config["search"]["static_ik_seeds"].as<int>();
    max_refine_centers_ = config["search"]["max_refine_centers"].as<int>();
    max_schedules_ = config["graph"]["max_static_schedules"].as<int>();
    max_nodes_per_schedule_ = config["graph"]["max_nodes_per_schedule_per_layer"].as<int>();
    cartesian_spacing_ = config["graph"]["cartesian_spacing_m"].as<double>();
    max_revolute_step_ = config["graph"]["max_revolute_step_rad"].as<double>();
    max_prismatic_step_ = config["graph"]["max_prismatic_step_m"].as<double>();
    dense_revolute_step_ = config["graph"]["dense_revolute_resolution_rad"].as<double>();
    dense_prismatic_step_ = config["graph"]["dense_prismatic_resolution_m"].as<double>();
    duplicate_tolerance_ = config["graph"]["duplicate_joint_distance"].as<double>();
    positive_margin_epsilon_ = config["graph"]["positive_margin_epsilon_rad"].as<double>();
    position_tolerance_ = config["graph"]["tcp_position_tolerance_m"].as<double>();
    orientation_tolerance_ = config["graph"]["tcp_orientation_tolerance_rad"].as<double>();

    for (const auto& entry : config["boundaries"])
    {
      BoundaryCase boundary;
      boundary.lift = entry["lift_m"].as<double>();
      boundary.previous_waypoint = entry["previous_waypoint"].as<int>();
      boundary.failed_waypoint = entry["failed_waypoint"].as<int>();
      boundary.previous_z = entry["previous_z_m"].as<double>();
      boundary.failed_z = entry["failed_z_m"].as<double>();
      boundary.refined_lower_z = entry["refined_lower_z_m"].as<double>();
      boundary.refined_upper_z = entry["refined_upper_z_m"].as<double>();
      for (const auto& value : entry["arm_start_rad"])
        boundary.arm_start.push_back(value.as<double>());
      if (boundary.arm_start.size() != core_.left_arm_group_->getVariableNames().size())
        throw std::runtime_error("Boundary arm_start_rad does not match left_arm variables");
      boundaries_.push_back(boundary);
    }

    static_csv_ = node_->get_parameter("torso_recovery_static_csv").as_string();
    nodes_csv_ = node_->get_parameter("torso_recovery_nodes_csv").as_string();
    edges_csv_ = node_->get_parameter("torso_recovery_edges_csv").as_string();
    paths_csv_ = node_->get_parameter("torso_recovery_paths_csv").as_string();
    summary_csv_ = node_->get_parameter("torso_recovery_summary_csv").as_string();
    result_yaml_ = node_->get_parameter("torso_recovery_result_yaml").as_string();
    audit_md_ = node_->get_parameter("torso_recovery_audit_md").as_string();
    initializeOutputs();
  }

  bool run()
  {
    std::vector<std::pair<BoundaryCase, std::vector<ModeResult>>> all_results;
    for (const auto& boundary : boundaries_)
    {
      core_.resetSceneForCandidate();
      std::vector<ModeResult> results;
      for (const std::string mode : { "LOCKED_BASELINE", "YAW_ONLY_RECOVERY",
                                      "PITCH_ONLY_RECOVERY", "YAW_PITCH_RECOVERY" })
      {
        RCLCPP_INFO(node_->get_logger(), "TORSO_RECOVERY lift=%.2f mode=%s starting",
                    boundary.lift, mode.c_str());
        results.push_back(runMode(boundary, mode));
        appendSummary(boundary, results.back());
        RCLCPP_INFO(node_->get_logger(), "TORSO_RECOVERY lift=%.2f mode=%s label=%s success=%s",
                    boundary.lift, mode.c_str(), results.back().label.c_str(),
                    results.back().success ? "true" : "false");
      }
      all_results.emplace_back(boundary, std::move(results));
    }
    writeResultYaml(all_results);
    writeAudit(all_results);
    return true;  // The diagnostic completed; a recovery path is not required for process success.
  }

private:
  double variableMargin(const moveit::core::RobotState& state, const std::string& name) const
  {
    const auto& bounds = core_.robot_model_->getVariableBounds(name);
    if (!bounds.position_bounded_)
      return kInf;
    const double value = state.getVariablePosition(name);
    return std::min(value - bounds.min_position_, bounds.max_position_ - value);
  }

  Metrics evaluate(moveit::core::RobotState& state, const geometry_msgs::msg::Pose& pose) const
  {
    state.update();
    const CollisionStatus status = core_.checkState(state);
    const auto clearance = core_.stateClearances(state);
    Metrics metrics;
    metrics.bounds_valid = status.joint_limit_valid;
    metrics.collision_free = status.joint_limit_valid && !status.self_collision && !status.environment_collision;
    metrics.joint3_margin = variableMargin(state, "openarm_left_joint3");
    metrics.joint5_margin = variableMargin(state, "openarm_left_joint5");
    metrics.active_margin = kInf;
    for (const auto& name : core_.left_arm_with_torso_group_->getVariableNames())
      metrics.active_margin = std::min(metrics.active_margin, variableMargin(state, name));
    metrics.environment_clearance = clearance.first;
    metrics.self_clearance = clearance.second;
    core_.poseError(state, pose, metrics.position_error, metrics.orientation_error);
    metrics.collision_pairs = pairString(status.pairs);
    if (!status.joint_limit_valid)
      metrics.failure = "JOINT_LIMIT_VIOLATION";
    else if (status.self_collision || status.environment_collision)
      metrics.failure = core_.collisionFailure(status);
    else if (metrics.position_error > position_tolerance_ || metrics.orientation_error > orientation_tolerance_)
      metrics.failure = "TCP_PATH_TOLERANCE_EXCEEDED";
    return metrics;
  }

  moveit::core::RobotState startState(const BoundaryCase& boundary) const
  {
    Candidate candidate;
    candidate.id = "torso_recovery_start";
    candidate.lift = boundary.lift;
    candidate.yaw = 0.0;
    candidate.pitch = 0.0;
    moveit::core::RobotState state = core_.initialState(candidate);
    state.setVariablePositions(core_.left_arm_group_->getVariableNames(), boundary.arm_start);
    state.update();
    return state;
  }

  geometry_msgs::msg::Pose boundaryPose(double z) const
  {
    auto pose = core_.graspPose();
    pose.position.z = z;
    return pose;
  }

  bool withinBounds(const std::string& name, double value) const
  {
    const auto& bounds = core_.robot_model_->getVariableBounds(name);
    return !bounds.position_bounded_ ||
           (value >= bounds.min_position_ - 1e-12 && value <= bounds.max_position_ + 1e-12);
  }

  double jointDelta(const std::string& name, double from, double to) const
  {
    const auto* joint = core_.robot_model_->getJointOfVariable(name);
    if (joint && joint->getType() == moveit::core::JointModel::REVOLUTE)
    {
      const auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint);
      if (revolute && revolute->isContinuous())
        return std::atan2(std::sin(to - from), std::cos(to - from));
    }
    return to - from;
  }

  double armDistance(const moveit::core::RobotState& from,
                     const moveit::core::RobotState& to) const
  {
    double squared = 0.0;
    for (const auto& name : core_.left_arm_group_->getVariableNames())
    {
      const double delta = jointDelta(name, from.getVariablePosition(name), to.getVariablePosition(name));
      squared += delta * delta;
    }
    return std::sqrt(squared);
  }

  double activeDistance(const moveit::core::RobotState& from,
                        const moveit::core::RobotState& to) const
  {
    double squared = 0.0;
    for (const auto& name : core_.left_arm_with_torso_group_->getVariableNames())
    {
      const double delta = jointDelta(name, from.getVariablePosition(name), to.getVariablePosition(name));
      squared += delta * delta;
    }
    return std::sqrt(squared);
  }

  std::vector<double> range(double minimum, double maximum, double step) const
  {
    std::vector<double> values;
    for (double value = minimum; value <= maximum + 1e-12; value += step)
      values.push_back(std::clamp(value, minimum, maximum));
    return values;
  }

  std::vector<std::pair<double, double>> coarseAngles(const std::string& mode) const
  {
    if (mode == "LOCKED_BASELINE")
      return { { 0.0, 0.0 } };
    const auto yaws = mode == "PITCH_ONLY_RECOVERY" ? std::vector<double>{ 0.0 } :
      range(yaw_min_, yaw_max_, coarse_step_);
    const auto pitches = mode == "YAW_ONLY_RECOVERY" ? std::vector<double>{ 0.0 } :
      range(pitch_min_, pitch_max_, coarse_step_);
    std::vector<std::pair<double, double>> values;
    for (const double yaw : yaws)
      for (const double pitch : pitches)
        values.emplace_back(yaw, pitch);
    return values;
  }

  moveit::core::RobotState randomArmSeed(const moveit::core::RobotState& base,
                                         std::uint64_t seed) const
  {
    moveit::core::RobotState state = base;
    std::mt19937_64 rng(seed);
    for (const auto& name : core_.left_arm_group_->getVariableNames())
    {
      const auto& bounds = core_.robot_model_->getVariableBounds(name);
      std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
      state.setVariablePosition(name, distribution(rng));
    }
    state.update();
    return state;
  }

  double staticCost(const moveit::core::RobotState& start, const StaticCandidate& candidate) const
  {
    const double torso = std::abs(candidate.yaw) + std::abs(candidate.pitch);
    const double arm = armDistance(start, candidate.state);
    const double margin = std::min(candidate.metrics.joint3_margin,
                                   std::min(candidate.metrics.joint5_margin, candidate.metrics.active_margin));
    const double clearance = std::min(candidate.metrics.environment_clearance, candidate.metrics.self_clearance);
    return torso + 0.25 * arm + 0.002 / std::max(1e-6, margin) +
           0.0001 / std::max(1e-6, clearance);
  }

  void writeStaticRow(const BoundaryCase& boundary, const std::string& mode,
                      const geometry_msgs::msg::Pose& pose, double yaw, double pitch,
                      bool refined, int seed_id, bool ik, const moveit::core::RobotState* state,
                      const Metrics* metrics, bool retained, double cost) const
  {
    std::ofstream out(static_csv_, std::ios::app);
    out << std::setprecision(15) << boundary.lift << ',' << mode << ',' << pose.position.z << ','
        << yaw << ',' << pitch << ',' << degrees(yaw) << ',' << degrees(pitch) << ','
        << (refined ? 1 : 0) << ',' << seed_id << ',' << (ik ? 1 : 0) << ','
        << (metrics && metrics->collision_free ? 1 : 0) << ',' << (retained ? 1 : 0) << ',' << cost << ',';
    if (metrics)
      out << metrics->joint3_margin << ',' << metrics->joint5_margin << ',' << metrics->active_margin << ','
          << metrics->environment_clearance << ',' << metrics->self_clearance << ','
          << metrics->position_error << ',' << metrics->orientation_error << ','
          << csvEscape(metrics->failure) << ',' << csvEscape(metrics->collision_pairs);
    else
      out << "nan,nan,nan,nan,nan,nan,nan,\"NO_IK_SOLUTION\",\"\"";
    for (const auto& name : core_.left_arm_group_->getVariableNames())
      out << ',' << (state ? std::to_string(state->getVariablePosition(name)) : std::string());
    out << '\n';
  }

  std::vector<StaticCandidate> evaluateAngles(const BoundaryCase& boundary, const std::string& mode,
                                               const geometry_msgs::msg::Pose& pose,
                                               const std::vector<std::pair<double, double>>& angles,
                                               bool refined, const moveit::core::RobotState& start,
                                               ModeResult& result) const
  {
    std::vector<StaticCandidate> viable;
    for (std::size_t angle_id = 0; angle_id < angles.size(); ++angle_id)
    {
      const double yaw = angles[angle_id].first;
      const double pitch = angles[angle_id].second;
      if (!withinBounds("waist_yaw_joint", yaw) || !withinBounds("waist_pitch_joint", pitch))
        continue;
      std::vector<StaticCandidate> angle_candidates;
      for (int seed_id = 0; seed_id < static_seeds_; ++seed_id)
      {
        moveit::core::RobotState state = seed_id == 0 ? start : randomArmSeed(
          start, 202608150000ULL + static_cast<std::uint64_t>(std::llround(boundary.lift * 1e6)) * 10007ULL +
          static_cast<std::uint64_t>(angle_id) * 101ULL + static_cast<std::uint64_t>(seed_id) +
          (refined ? 900000001ULL : 0ULL));
        state.setVariablePosition("lift_joint", boundary.lift);
        state.setVariablePosition("waist_yaw_joint", yaw);
        state.setVariablePosition("waist_pitch_joint", pitch);
        state.update();
        const bool ik = state.setFromIK(core_.left_arm_group_, pose, core_.left_tcp_link_,
                                        core_.scene_config_.ik_timeout);
        if (!ik)
        {
          writeStaticRow(boundary, mode, pose, yaw, pitch, refined, seed_id, false,
                         nullptr, nullptr, false, kInf);
          continue;
        }
        ++result.static_raw_ik;
        Metrics metrics = evaluate(state, pose);
        if (metrics.collision_free)
          ++result.static_collision_free;
        const bool positive = metrics.collision_free &&
          metrics.joint3_margin > positive_margin_epsilon_ &&
          metrics.joint5_margin > positive_margin_epsilon_ &&
          metrics.active_margin > positive_margin_epsilon_;
        StaticCandidate candidate(core_.robot_model_);
        candidate.state = state;
        candidate.metrics = metrics;
        candidate.yaw = yaw;
        candidate.pitch = pitch;
        candidate.refined = refined;
        candidate.seed_id = seed_id;
        candidate.cost = staticCost(start, candidate);
        if (positive)
        {
          ++result.static_margin_positive;
          angle_candidates.push_back(candidate);
        }
        writeStaticRow(boundary, mode, pose, yaw, pitch, refined, seed_id, true,
                       &state, &metrics, positive, candidate.cost);
      }
      if (!angle_candidates.empty())
      {
        std::stable_sort(angle_candidates.begin(), angle_candidates.end(),
                         [](const auto& a, const auto& b) { return a.cost < b.cost; });
        viable.push_back(angle_candidates.front());
      }
    }
    return viable;
  }

  std::vector<StaticCandidate> staticSearch(const BoundaryCase& boundary, const std::string& mode,
                                             const moveit::core::RobotState& start,
                                             ModeResult& result) const
  {
    const auto pose = boundaryPose(boundary.refined_upper_z);
    auto candidates = evaluateAngles(boundary, mode, pose, coarseAngles(mode), false, start, result);
    if (mode != "LOCKED_BASELINE" && !candidates.empty())
    {
      std::stable_sort(candidates.begin(), candidates.end(),
                       [](const auto& a, const auto& b) { return a.cost < b.cost; });
      std::set<std::pair<int, int>> unique;
      std::vector<std::pair<double, double>> refined_angles;
      const int centers = std::min(max_refine_centers_, static_cast<int>(candidates.size()));
      for (int index = 0; index < centers; ++index)
      {
        const std::vector<double> refined_yaws = mode == "PITCH_ONLY_RECOVERY" ?
          std::vector<double>{ 0.0 } : range(
            std::max(yaw_min_, candidates[index].yaw - refine_radius_),
            std::min(yaw_max_, candidates[index].yaw + refine_radius_), fine_step_);
        const std::vector<double> refined_pitches = mode == "YAW_ONLY_RECOVERY" ?
          std::vector<double>{ 0.0 } : range(
            std::max(pitch_min_, candidates[index].pitch - refine_radius_),
            std::min(pitch_max_, candidates[index].pitch + refine_radius_), fine_step_);
        for (const double yaw : refined_yaws)
          for (const double pitch : refined_pitches)
          {
            const auto key = std::make_pair(static_cast<int>(std::llround(degrees(yaw))),
                                            static_cast<int>(std::llround(degrees(pitch))));
            if (unique.insert(key).second)
              refined_angles.emplace_back(radians(key.first), radians(key.second));
          }
      }
      auto refined = evaluateAngles(boundary, mode, pose, refined_angles, true, start, result);
      candidates.insert(candidates.end(), refined.begin(), refined.end());
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
      return std::tie(a.cost, a.yaw, a.pitch) < std::tie(b.cost, b.yaw, b.pitch);
    });
    std::set<std::pair<int, int>> used;
    std::vector<StaticCandidate> schedules;
    for (const auto& candidate : candidates)
    {
      const auto key = std::make_pair(static_cast<int>(std::llround(candidate.yaw * 1e9)),
                                      static_cast<int>(std::llround(candidate.pitch * 1e9)));
      if (used.insert(key).second)
        schedules.push_back(candidate);
      if (static_cast<int>(schedules.size()) >= max_schedules_)
        break;
    }
    return schedules;
  }

  bool duplicate(const GraphNode& candidate, const std::vector<GraphNode>& nodes) const
  {
    return std::any_of(nodes.begin(), nodes.end(), [&](const GraphNode& existing) {
      if (std::abs(existing.state.getVariablePosition("waist_yaw_joint") -
                   candidate.state.getVariablePosition("waist_yaw_joint")) > 1e-10 ||
          std::abs(existing.state.getVariablePosition("waist_pitch_joint") -
                   candidate.state.getVariablePosition("waist_pitch_joint")) > 1e-10)
        return false;
      return armDistance(existing.state, candidate.state) <= duplicate_tolerance_;
    });
  }

  EdgeAudit auditEdge(const GraphNode& from, const GraphNode& to,
                      const geometry_msgs::msg::Pose& from_pose,
                      const geometry_msgs::msg::Pose& to_pose) const
  {
    EdgeAudit audit;
    std::size_t steps = 1;
    for (const auto& name : core_.left_arm_with_torso_group_->getVariableNames())
    {
      const auto* joint = core_.robot_model_->getJointOfVariable(name);
      const double delta = std::abs(jointDelta(name, from.state.getVariablePosition(name),
                                               to.state.getVariablePosition(name)));
      const bool prismatic = joint && joint->getType() == moveit::core::JointModel::PRISMATIC;
      const double maximum = prismatic ? max_prismatic_step_ : max_revolute_step_;
      if (delta > maximum + 1e-12)
      {
        audit.failure = prismatic ? "EDGE_PRISMATIC_STEP_EXCEEDED" : "EDGE_REVOLUTE_STEP_EXCEEDED";
        return audit;
      }
      const double resolution = prismatic ? dense_prismatic_step_ : dense_revolute_step_;
      steps = std::max(steps, static_cast<std::size_t>(std::ceil(delta / resolution)));
    }
    audit.displacement = activeDistance(from.state, to.state);
    for (std::size_t index = 1; index <= steps; ++index)
    {
      const double ratio = static_cast<double>(index) / static_cast<double>(steps);
      moveit::core::RobotState state = from.state;
      for (const auto& name : core_.left_arm_with_torso_group_->getVariableNames())
      {
        const double delta = jointDelta(name, from.state.getVariablePosition(name),
                                        to.state.getVariablePosition(name));
        state.setVariablePosition(name, from.state.getVariablePosition(name) + ratio * delta);
      }
      geometry_msgs::msg::Pose desired = from_pose;
      desired.position.x += ratio * (to_pose.position.x - from_pose.position.x);
      desired.position.y += ratio * (to_pose.position.y - from_pose.position.y);
      desired.position.z += ratio * (to_pose.position.z - from_pose.position.z);
      Metrics metrics = evaluate(state, desired);
      audit.min_active_margin = std::min(audit.min_active_margin, metrics.active_margin);
      audit.min_environment_clearance = std::min(audit.min_environment_clearance,
                                                 metrics.environment_clearance);
      audit.min_self_clearance = std::min(audit.min_self_clearance, metrics.self_clearance);
      audit.max_position_error = std::max(audit.max_position_error, metrics.position_error);
      audit.max_orientation_error = std::max(audit.max_orientation_error, metrics.orientation_error);
      if (!metrics.bounds_valid || !metrics.collision_free || !metrics.failure.empty())
      {
        audit.failure = metrics.failure.empty() ? "DENSE_EDGE_INVALID" : metrics.failure;
        audit.collision_pairs = metrics.collision_pairs;
        return audit;
      }
    }
    const double margin_penalty = 0.002 / std::max(1e-6, audit.min_active_margin);
    const double clearance = std::min(audit.min_environment_clearance, audit.min_self_clearance);
    audit.edge_cost = audit.displacement + margin_penalty + 0.0001 / std::max(1e-6, clearance);
    audit.valid = true;
    return audit;
  }

  void writeNode(const BoundaryCase& boundary, const std::string& mode, int layer, int node_id,
                 const geometry_msgs::msg::Pose& pose, const GraphNode& node,
                 const std::string& label) const
  {
    std::ofstream out(nodes_csv_, std::ios::app);
    out << std::setprecision(15) << boundary.lift << ',' << mode << ',' << layer << ',' << node_id << ','
        << pose.position.z << ',' << node.state.getVariablePosition("waist_yaw_joint") << ','
        << node.state.getVariablePosition("waist_pitch_joint") << ',' << node.metrics.joint3_margin << ','
        << node.metrics.joint5_margin << ',' << node.metrics.active_margin << ','
        << node.metrics.environment_clearance << ',' << node.metrics.self_clearance << ','
        << node.metrics.position_error << ',' << node.metrics.orientation_error << ','
        << node.parent << ',' << node.edge_cost << ',' << node.cost << ',' << node.schedule_id << ','
        << (node.reachable ? 1 : 0) << ',' << csvEscape(label) << ',' << csvEscape(node.metrics.collision_pairs);
    for (const auto& name : core_.left_arm_group_->getVariableNames())
      out << ',' << node.state.getVariablePosition(name);
    out << '\n';
  }

  void writeEdge(const BoundaryCase& boundary, const std::string& mode, int from_layer,
                 int from_id, int to_id, const EdgeAudit& audit) const
  {
    std::ofstream out(edges_csv_, std::ios::app);
    out << std::setprecision(15) << boundary.lift << ',' << mode << ',' << from_layer << ',' << from_id << ','
        << from_layer + 1 << ',' << to_id << ',' << (audit.valid ? 1 : 0) << ',' << audit.displacement << ','
        << audit.edge_cost << ',' << audit.min_active_margin << ',' << audit.min_environment_clearance << ','
        << audit.min_self_clearance << ',' << audit.max_position_error << ',' << audit.max_orientation_error << ','
        << csvEscape(audit.failure) << ',' << csvEscape(audit.collision_pairs) << '\n';
  }

  ModeResult buildGraph(const BoundaryCase& boundary, const std::string& mode,
                        const moveit::core::RobotState& start,
                        std::vector<StaticCandidate> schedules, ModeResult result) const
  {
    if (mode == "LOCKED_BASELINE" && schedules.empty())
    {
      StaticCandidate locked(core_.robot_model_);
      locked.state = start;
      locked.yaw = locked.pitch = 0.0;
      schedules.push_back(locked);
    }
    if (schedules.empty())
    {
      result.label = result.static_collision_free == 0 ?
        (result.static_raw_ik == 0 ? "TORSO_RECOVERY_IK_FAILURE" : "TORSO_RECOVERY_COLLISION_FAILURE") :
        "TORSO_RECOVERY_IK_FAILURE";
      result.failure_reason = "NO_STATIC_MARGIN_POSITIVE_RECOVERY_CANDIDATE";
      return result;
    }

    const int intervals = std::max(1, static_cast<int>(std::ceil(
      std::abs(boundary.failed_z - boundary.previous_z) / cartesian_spacing_)));
    result.layers.resize(intervals + 1);
    GraphNode initial(core_.robot_model_);
    initial.state = start;
    initial.metrics = evaluate(initial.state, boundaryPose(boundary.previous_z));
    initial.cost = 0.0;
    initial.reachable = true;
    initial.schedule_id = -1;
    result.layers[0].push_back(initial);
    writeNode(boundary, mode, 0, 0, boundaryPose(boundary.previous_z), initial, "START_AT_VERIFIED_BOUNDARY");

    for (int layer = 1; layer <= intervals; ++layer)
    {
      const double ratio = static_cast<double>(layer) / intervals;
      const auto pose = boundaryPose(boundary.previous_z + ratio * (boundary.failed_z - boundary.previous_z));
      const auto previous_pose = boundaryPose(boundary.previous_z +
        static_cast<double>(layer - 1) / intervals * (boundary.failed_z - boundary.previous_z));
      int raw_ik = 0;
      int collision_free = 0;
      for (std::size_t schedule_id = 0; schedule_id < schedules.size(); ++schedule_id)
      {
        // Complete the gradual torso ramp at the audited refined failure boundary,
        // where the static recovery candidate was evaluated, then hold it through
        // the original first-failed waypoint to test actual reconnection beyond it.
        const double layer_z = pose.position.z;
        const double recovery_ratio = std::clamp(
          (layer_z - boundary.previous_z) /
            std::max(1e-12, boundary.refined_upper_z - boundary.previous_z),
          0.0, 1.0);
        const double yaw = recovery_ratio * schedules[schedule_id].yaw;
        const double pitch = recovery_ratio * schedules[schedule_id].pitch;
        if (!withinBounds("waist_yaw_joint", yaw) || !withinBounds("waist_pitch_joint", pitch))
          continue;
        for (std::size_t parent_seed = 0; parent_seed <= result.layers[layer - 1].size(); ++parent_seed)
        {
          if (parent_seed < result.layers[layer - 1].size() &&
              !result.layers[layer - 1][parent_seed].reachable)
            continue;
          moveit::core::RobotState state = parent_seed < result.layers[layer - 1].size() ?
            result.layers[layer - 1][parent_seed].state : schedules[schedule_id].state;
          state.setVariablePosition("lift_joint", boundary.lift);
          state.setVariablePosition("waist_yaw_joint", yaw);
          state.setVariablePosition("waist_pitch_joint", pitch);
          state.update();
          if (!state.setFromIK(core_.left_arm_group_, pose, core_.left_tcp_link_, core_.scene_config_.ik_timeout))
            continue;
          ++raw_ik;
          Metrics metrics = evaluate(state, pose);
          if (!metrics.collision_free || !metrics.failure.empty())
          {
            if (!metrics.collision_pairs.empty())
              result.collision_pairs = metrics.collision_pairs;
            continue;
          }
          ++collision_free;
          GraphNode candidate(core_.robot_model_);
          candidate.state = state;
          candidate.metrics = metrics;
          candidate.schedule_id = static_cast<int>(schedule_id);
          if (!duplicate(candidate, result.layers[layer]))
            result.layers[layer].push_back(std::move(candidate));
        }
      }

      // Keep a bounded, diverse frontier per torso schedule. Only candidates generated from
      // reachable parents enter this pool; ranking favors local continuation and finite margin.
      // The cap is explicit in the YAML and does not alter any physical constraint.
      if (max_nodes_per_schedule_ > 0)
      {
        const auto continuity_score = [&](const GraphNode& candidate) {
          double best = kInf;
          for (const auto& parent : result.layers[layer - 1])
            if (parent.reachable)
              best = std::min(best, activeDistance(parent.state, candidate.state));
          return best + 0.002 / std::max(1e-6, candidate.metrics.active_margin);
        };
        std::stable_sort(result.layers[layer].begin(), result.layers[layer].end(),
                         [&](const GraphNode& a, const GraphNode& b) {
          if (a.schedule_id != b.schedule_id)
            return a.schedule_id < b.schedule_id;
          return continuity_score(a) < continuity_score(b);
        });
        std::map<int, int> retained_per_schedule;
        std::vector<GraphNode> retained;
        retained.reserve(result.layers[layer].size());
        for (auto& candidate : result.layers[layer])
          if (retained_per_schedule[candidate.schedule_id]++ < max_nodes_per_schedule_)
            retained.push_back(std::move(candidate));
        result.layers[layer] = std::move(retained);
      }

      for (std::size_t target_id = 0; target_id < result.layers[layer].size(); ++target_id)
      {
        auto& target = result.layers[layer][target_id];
        for (std::size_t source_id = 0; source_id < result.layers[layer - 1].size(); ++source_id)
        {
          const auto& source = result.layers[layer - 1][source_id];
          if (!source.reachable)
            continue;
          ++result.evaluated_edges;
          const EdgeAudit edge = auditEdge(source, target, previous_pose, pose);
          writeEdge(boundary, mode, layer - 1, static_cast<int>(source_id),
                    static_cast<int>(target_id), edge);
          if (!edge.valid)
          {
            if (!edge.collision_pairs.empty())
              result.collision_pairs = edge.collision_pairs;
            continue;
          }
          ++result.connected_edges;
          const double torso_target_cost = 0.1 * (
            std::abs(target.state.getVariablePosition("waist_yaw_joint")) +
            std::abs(target.state.getVariablePosition("waist_pitch_joint")));
          const double candidate_cost = source.cost + edge.edge_cost + torso_target_cost;
          if (candidate_cost < target.cost)
          {
            target.cost = candidate_cost;
            target.parent = static_cast<int>(source_id);
            target.edge_cost = edge.edge_cost;
            target.reachable = true;
          }
        }
        writeNode(boundary, mode, layer, static_cast<int>(target_id), pose, target,
                  target.reachable ? "REACHABLE" : "UNREACHABLE");
      }
      const bool any_reachable = std::any_of(result.layers[layer].begin(), result.layers[layer].end(),
                                              [](const auto& node) { return node.reachable; });
      if (!any_reachable)
      {
        result.first_failed_layer = layer;
        if (mode == "LOCKED_BASELINE")
          result.label = "LOCKED_JOINT_LIMIT_FAILURE";
        else if (result.layers[layer].empty())
          result.label = raw_ik == 0 ? "TORSO_RECOVERY_IK_FAILURE" :
            (collision_free == 0 ? "TORSO_RECOVERY_COLLISION_FAILURE" :
                                   "TORSO_RECOVERY_CONNECTIVITY_FAILURE");
        else
          result.label = "TORSO_RECOVERY_CONNECTIVITY_FAILURE";
        result.failure_reason = result.layers[layer].empty() ?
          (raw_ik == 0 ? "NO_IK_AT_LOCAL_LAYER" : "NO_VALID_NODE_AT_LOCAL_LAYER") :
          "NO_DENSE_VALID_EDGE_TO_LOCAL_LAYER";
        break;
      }
    }

    if (result.first_failed_layer < 0)
    {
      int best = -1;
      const auto& final_layer = result.layers.back();
      for (std::size_t index = 0; index < final_layer.size(); ++index)
      {
        const auto& node = final_layer[index];
        if (node.reachable && node.metrics.joint3_margin > positive_margin_epsilon_ &&
            node.metrics.joint5_margin > positive_margin_epsilon_ &&
            (best < 0 || node.cost < final_layer[best].cost))
          best = static_cast<int>(index);
      }
      if (best >= 0)
      {
        result.success = mode != "LOCKED_BASELINE";
        result.label = mode == "YAW_ONLY_RECOVERY" ? "YAW_ONLY_RECOVERY_SUCCESS" :
          (mode == "PITCH_ONLY_RECOVERY" ? "PITCH_ONLY_RECOVERY_SUCCESS" :
           (mode == "YAW_PITCH_RECOVERY" ? "YAW_PITCH_RECOVERY_SUCCESS" :
                                            "LOCKED_JOINT_LIMIT_FAILURE"));
        result.reconnect_layer = intervals;
        result.path_indices.assign(intervals + 1, -1);
        int cursor = best;
        for (int layer = intervals; layer >= 0; --layer)
        {
          result.path_indices[layer] = cursor;
          cursor = result.layers[layer][cursor].parent;
        }
        analyzeAndWritePath(boundary, result);
      }
      else
      {
        result.label = mode == "LOCKED_BASELINE" ? "LOCKED_JOINT_LIMIT_FAILURE" :
                                                   "TORSO_RECOVERY_CONNECTIVITY_FAILURE";
        result.failure_reason = "FINAL_LAYER_HAS_NO_POSITIVE_JOINT3_JOINT5_MARGIN";
      }
    }
    return result;
  }

  void analyzeAndWritePath(const BoundaryCase& boundary, ModeResult& result) const
  {
    std::ofstream out(paths_csv_, std::ios::app);
    for (std::size_t layer = 0; layer < result.path_indices.size(); ++layer)
    {
      const auto& node = result.layers[layer][result.path_indices[layer]];
      if (layer > 0)
        result.path_length += activeDistance(
          result.layers[layer - 1][result.path_indices[layer - 1]].state, node.state);
      if (layer > 0 && result.first_recovery_layer < 0 &&
          node.metrics.joint3_margin > positive_margin_epsilon_ &&
          node.metrics.joint5_margin > positive_margin_epsilon_)
        result.first_recovery_layer = static_cast<int>(layer);
      if (layer > 0)
      {
        result.min_joint3_margin = std::min(result.min_joint3_margin, node.metrics.joint3_margin);
        result.min_joint5_margin = std::min(result.min_joint5_margin, node.metrics.joint5_margin);
        result.min_active_margin = std::min(result.min_active_margin, node.metrics.active_margin);
      }
      result.min_environment_clearance = std::min(result.min_environment_clearance,
                                                   node.metrics.environment_clearance);
      result.min_self_clearance = std::min(result.min_self_clearance, node.metrics.self_clearance);
      result.selected_yaw = node.state.getVariablePosition("waist_yaw_joint");
      result.selected_pitch = node.state.getVariablePosition("waist_pitch_joint");
      const double z = boundary.previous_z + static_cast<double>(layer) /
        (result.path_indices.size() - 1) * (boundary.failed_z - boundary.previous_z);
      out << std::setprecision(15) << boundary.lift << ',' << result.mode << ',' << layer << ',' << z << ','
          << result.selected_yaw << ',' << result.selected_pitch;
      for (const auto& name : core_.left_arm_group_->getVariableNames())
        out << ',' << node.state.getVariablePosition(name);
      out << ',' << node.metrics.joint3_margin << ',' << node.metrics.joint5_margin << ','
          << node.metrics.active_margin << ',' << node.metrics.environment_clearance << ','
          << node.metrics.self_clearance << ',' << node.metrics.position_error << ','
          << node.metrics.orientation_error << ',' << node.parent << ',' << node.edge_cost << ','
          << csvEscape(result.label) << '\n';
    }
  }

  ModeResult runMode(const BoundaryCase& boundary, const std::string& mode) const
  {
    ModeResult result;
    result.mode = mode;
    auto start = startState(boundary);
    const auto schedules = staticSearch(boundary, mode, start, result);
    return buildGraph(boundary, mode, start, schedules, std::move(result));
  }

  void initializeOutputs() const
  {
    {
      std::ofstream out(static_csv_, std::ios::trunc);
      out << "lift,mode,target_z,yaw_rad,pitch_rad,yaw_deg,pitch_deg,refined,seed_id,raw_ik,collision_free,"
             "margin_positive_retained,cost,joint3_margin,joint5_margin,active_joint_min_margin,"
             "environment_clearance,self_clearance,tcp_position_error,tcp_orientation_error,failure,"
             "collision_pairs";
      for (const auto& name : core_.left_arm_group_->getVariableNames())
        out << ',' << name;
      out << '\n';
    }
    {
      std::ofstream out(nodes_csv_, std::ios::trunc);
      out << "lift,mode,layer,node_id,tcp_z,yaw,pitch,joint3_margin,joint5_margin,active_joint_min_margin,"
             "environment_clearance,self_clearance,tcp_position_error,tcp_orientation_error,parent_node,"
             "edge_cost,total_cost,schedule_id,reachable,failure_label,collision_pairs";
      for (const auto& name : core_.left_arm_group_->getVariableNames())
        out << ',' << name;
      out << '\n';
    }
    {
      std::ofstream out(edges_csv_, std::ios::trunc);
      out << "lift,mode,from_layer,from_node,to_layer,to_node,valid,joint_displacement,edge_cost,"
             "min_active_margin,min_environment_clearance,min_self_clearance,max_tcp_position_error,"
             "max_tcp_orientation_error,failure,collision_pairs\n";
    }
    {
      std::ofstream out(paths_csv_, std::ios::trunc);
      out << "lift,mode,waypoint,tcp_z,yaw,pitch";
      for (const auto& name : core_.left_arm_group_->getVariableNames())
        out << ',' << name;
      out << ",joint3_margin,joint5_margin,active_joint_min_margin,environment_clearance,self_clearance,"
             "tcp_position_error,tcp_orientation_error,parent_node,edge_cost,failure_label\n";
    }
    {
      std::ofstream out(summary_csv_, std::ios::trunc);
      out << "lift,mode,success,failure_label,static_raw_ik,static_collision_free,static_margin_positive,"
             "evaluated_edges,connected_edges,first_failed_layer,first_recovery_layer,reconnect_layer,"
             "selected_yaw_rad,selected_pitch_rad,selected_yaw_deg,selected_pitch_deg,path_length,"
             "min_joint3_margin_after_start,min_joint5_margin_after_start,min_active_margin_after_start,"
             "min_environment_clearance,min_self_clearance,failure_reason,collision_pairs\n";
    }
  }

  void appendSummary(const BoundaryCase& boundary, const ModeResult& result) const
  {
    std::ofstream out(summary_csv_, std::ios::app);
    out << std::setprecision(15) << boundary.lift << ',' << result.mode << ',' << (result.success ? 1 : 0) << ','
        << result.label << ',' << result.static_raw_ik << ',' << result.static_collision_free << ','
        << result.static_margin_positive << ',' << result.evaluated_edges << ',' << result.connected_edges << ','
        << result.first_failed_layer << ',' << result.first_recovery_layer << ',' << result.reconnect_layer << ','
        << result.selected_yaw << ',' << result.selected_pitch << ',' << degrees(result.selected_yaw) << ','
        << degrees(result.selected_pitch) << ',' << result.path_length << ',' << result.min_joint3_margin << ','
        << result.min_joint5_margin << ',' << result.min_active_margin << ','
        << result.min_environment_clearance << ',' << result.min_self_clearance << ','
        << csvEscape(result.failure_reason) << ',' << csvEscape(result.collision_pairs) << '\n';
  }

  void writeResultYaml(const std::vector<std::pair<BoundaryCase, std::vector<ModeResult>>>& all) const
  {
    std::ofstream out(result_yaml_, std::ios::trunc);
    out << "protocol: STAGE_CONSTRAINED_TORSO_RECOVERY_V1\nplanning_only: true\n"
           "trajectory_execution_performed: false\nrviz_started: false\nresults:\n";
    for (const auto& group : all)
    {
      out << "  - lift_m: " << group.first.lift << "\n    modes:\n";
      for (const auto& result : group.second)
        out << "      - mode: " << result.mode << "\n        label: " << result.label
            << "\n        success: " << (result.success ? "true" : "false")
            << "\n        yaw_rad: " << result.selected_yaw
            << "\n        pitch_rad: " << result.selected_pitch
            << "\n        first_recovery_waypoint: " << result.first_recovery_layer
            << "\n        reconnect_waypoint: " << result.reconnect_layer << '\n';
    }
  }

  void writeAudit(const std::vector<std::pair<BoundaryCase, std::vector<ModeResult>>>& all) const
  {
    std::ofstream out(audit_md_, std::ios::trunc);
    out << "# Stage-constrained torso local recovery v1 audit\n\nGenerated: " << timestampNow()
        << "\n\nThis is a planning-only, approximately 1 mm boundary-local IK graph audit. It did not invoke OMPL "
           "planning, RViz, trajectory execution, controllers, ros2_control, or hardware. Lift was fixed per case; "
           "the TCP Cartesian positions and orientation were unchanged.\n\n"
           "|Lift|Mode|Label|Static raw / collision-free / positive|Edges valid / tested|Yaw / Pitch (deg)|"
           "First recovery / reconnect|j3 / j5 min margin after start|Environment / self min clearance|\n"
           "|---:|---|---|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& group : all)
      for (const auto& result : group.second)
        out << '|' << group.first.lift << '|' << result.mode << '|' << result.label << '|'
            << result.static_raw_ik << " / " << result.static_collision_free << " / "
            << result.static_margin_positive << '|' << result.connected_edges << " / "
            << result.evaluated_edges << '|' << degrees(result.selected_yaw) << " / "
            << degrees(result.selected_pitch) << '|' << result.first_recovery_layer << " / "
            << result.reconnect_layer << '|' << result.min_joint3_margin << " / "
            << result.min_joint5_margin << '|' << result.min_environment_clearance << " / "
            << result.min_self_clearance << "|\n";
    out << "\nA successful local label proves only continuity across the audited boundary. It does not authorize or imply "
           "a complete five-stage experiment. The fixed start has zero j3/j5 margin by construction; reported path "
           "minimum margins exclude layer 0 and must be finite and positive for success.\n";
  }

  rclcpp::Node::SharedPtr node_;
  ReferenceTrajectoryGenerator core_;
  std::vector<BoundaryCase> boundaries_;
  double yaw_min_{};
  double yaw_max_{};
  double pitch_min_{};
  double pitch_max_{};
  double coarse_step_{};
  double fine_step_{};
  double refine_radius_{};
  int static_seeds_{};
  int max_refine_centers_{};
  int max_schedules_{};
  int max_nodes_per_schedule_{};
  double cartesian_spacing_{};
  double max_revolute_step_{};
  double max_prismatic_step_{};
  double dense_revolute_step_{};
  double dense_prismatic_step_{};
  double duplicate_tolerance_{};
  double positive_margin_epsilon_{};
  double position_tolerance_{};
  double orientation_tolerance_{};
  std::string static_csv_;
  std::string nodes_csv_;
  std::string edges_csv_;
  std::string paths_csv_;
  std::string summary_csv_;
  std::string result_yaml_;
  std::string audit_md_;
};
}  // namespace torso_recovery_v1

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("stage_constrained_torso_recovery_v1", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  int exit_code = 1;
  try
  {
    torso_recovery_v1::Runner runner(node);
    exit_code = runner.run() ? 0 : 2;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Torso recovery v1 failed: %s", error.what());
  }
  executor.cancel();
  if (spin_thread.joinable())
    spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
