#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

// Reuse the audited scene, ACM, attachment, model, and pose construction code.
// The existing generators are included, not edited.
#define private public
#define main preserved_reference_generator_main
#include "reference_trajectory_generator.cpp"
#undef main
#undef private

namespace stage_constrained_v3
{
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Metrics
{
  double joint_margin{ std::numeric_limits<double>::quiet_NaN() };
  double active_joint_margin{ std::numeric_limits<double>::quiet_NaN() };
  double environment_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double self_clearance{ std::numeric_limits<double>::quiet_NaN() };
  bool joint_limit_valid{ false };
  bool collision_free{ false };
  std::string failure_reason;
  std::string collision_pairs;
};

struct Node
{
  moveit::core::RobotState state;
  Metrics metrics;
  int seed_id{ -1 };
  int raw_attempt_id{ -1 };
  bool continuation_from_reachable{ false };
  double cost{ kInfinity };
  int predecessor{ -1 };
  bool reachable{ false };

  explicit Node(const moveit::core::RobotModelConstPtr& model) : state(model) {}
};

struct RawRepresentative
{
  moveit::core::RobotState state;
  Metrics metrics;
  int seed_id{ -1 };
  bool available{ false };

  explicit RawRepresentative(const moveit::core::RobotModelConstPtr& model) : state(model) {}
};

struct Layer
{
  int waypoint{};
  geometry_msgs::msg::Pose pose;
  int ik_attempts{};
  int raw_ik_count{};
  int collision_free_count{};
  std::size_t evaluated_edges_from_reachable{};
  std::size_t connected_edges_from_reachable{};
  std::size_t reachable_nodes{};
  std::vector<Node> nodes;
  RawRepresentative raw_representative;

  explicit Layer(const moveit::core::RobotModelConstPtr& model) : raw_representative(model) {}
};

struct GraphResult
{
  double lift{};
  bool success{ false };
  int failure_waypoint{ -1 };
  int last_reachable_waypoint{ -1 };
  int first_node_empty_waypoint{ -1 };
  int first_edge_gap_waypoint{ -1 };
  std::string failure_label;
  std::string failure_reason;
  std::string collision_pairs;
  std::vector<Layer> layers;
  std::vector<moveit::core::RobotState> path;
};

struct AuditEdgeResult
{
  bool valid{ false };
  bool raw_step_exceeded{ false };
  bool corrected_step_exceeded{ false };
  double displacement{};
  double raw_displacement{};
  double max_corrected_delta{};
  std::string max_delta_joint;
  std::string failure_reason;
  std::string collision_pairs;
};

struct JacobianMetrics
{
  double minimum_singular_value{};
  double maximum_singular_value{};
  double condition_number{};
  double manipulability{};
  std::string singular_values;
};

struct BoundaryAuditResult
{
  double lift{};
  int previous_waypoint{};
  int target_waypoint{};
  std::size_t previous_reachable_candidates{};
  int target_raw_ik{};
  int target_collision_free_ik{};
  std::size_t target_uncapped_candidates{};
  std::size_t target_cap48_candidates{};
  std::size_t total_edges{};
  std::size_t connected_edges{};
  std::size_t connected_edges_removed_by_cap{};
  std::size_t representation_only_edges{};
  std::size_t refinement_paths{};
  double minimum_displacement{ kInfinity };
  int closest_previous{ -1 };
  int closest_target{ -1 };
  std::string previous_zero_margin_joints;
  std::string target_zero_margin_joints;
  JacobianMetrics previous_jacobian;
  JacobianMetrics target_jacobian;
  std::string classification{ "UNRESOLVED" };
};

class Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node)
    : node_(node), core_(node)
  {
    summary_csv_ = node_->get_parameter("stage_v3_summary_csv").as_string();
    candidates_csv_ = node_->get_parameter("stage_v3_candidates_csv").as_string();
    layers_csv_ = node_->get_parameter("stage_v3_layers_csv").as_string();
    edges_csv_ = node_->get_parameter("stage_v3_edges_csv").as_string();
    boundary_csv_ = node_->get_parameter("stage_v3_boundary_csv").as_string();
    graph_path_csv_ = node_->get_parameter("stage_v3_graph_path_csv").as_string();
    waypoints_yaml_ = node_->get_parameter("stage_v3_waypoints_yaml").as_string();
    audit_md_ = node_->get_parameter("stage_v3_audit_md").as_string();
    ik_seeds_per_waypoint_ = node_->get_parameter("stage_v3_ik_seeds_per_waypoint").as_int();
    max_candidates_per_waypoint_ = node_->get_parameter("stage_v3_max_candidates_per_waypoint").as_int();
    cartesian_spacing_ = node_->get_parameter("stage_v3_cartesian_spacing_m").as_double();
    duplicate_tolerance_ = node_->get_parameter("stage_v3_duplicate_tolerance_rad").as_double();
    max_revolute_edge_step_ = node_->get_parameter("stage_v3_max_edge_revolute_step_rad").as_double();
    max_prismatic_edge_step_ = node_->get_parameter("stage_v3_max_edge_prismatic_step_m").as_double();
    displacement_weight_ = node_->get_parameter("stage_v3_displacement_weight").as_double();
    limit_proximity_weight_ = node_->get_parameter("stage_v3_limit_proximity_weight").as_double();
    node_->get_parameter_or("stage_v3_boundary_audit_only", boundary_audit_only_, false);
    if (boundary_audit_only_)
    {
      boundary_audit_summary_csv_ = node_->get_parameter("stage_v3_boundary_audit_summary_csv").as_string();
      boundary_audit_edges_csv_ = node_->get_parameter("stage_v3_boundary_audit_edges_csv").as_string();
      boundary_audit_joints_csv_ = node_->get_parameter("stage_v3_boundary_audit_joints_csv").as_string();
      boundary_audit_closest_csv_ = node_->get_parameter("stage_v3_boundary_audit_closest_csv").as_string();
      boundary_audit_jacobian_csv_ = node_->get_parameter("stage_v3_boundary_audit_jacobian_csv").as_string();
      boundary_audit_continuation_csv_ =
        node_->get_parameter("stage_v3_boundary_audit_continuation_csv").as_string();
      boundary_audit_refinement_csv_ =
        node_->get_parameter("stage_v3_boundary_audit_refinement_csv").as_string();
      boundary_audit_md_ = node_->get_parameter("stage_v3_boundary_audit_md").as_string();
    }
    initializeOutputs();
  }

  bool run()
  {
    if (boundary_audit_only_)
      return runBoundaryAudit();
    std::vector<GraphResult> results;
    for (const double lift : { 0.35, 0.40 })
    {
      core_.resetSceneForCandidate();
      GraphResult result = buildGraph(lift, core_.graspPose(),
                                      core_.approachPose(core_.scene_config_.rim_clearance), false);
      writeSummary(result);
      writeBoundary(result);
      RCLCPP_INFO(node_->get_logger(),
                  "STAGE_V3_GRAPH lift=%.2f success=%s last_reachable=%d failure_waypoint=%d label=%s",
                  lift, result.success ? "true" : "false", result.last_reachable_waypoint,
                  result.failure_waypoint, result.failure_label.c_str());
      results.push_back(std::move(result));
    }

    const GraphResult* selected = nullptr;
    for (const auto& result : results)
      if (result.success && (!selected || pathMinimumMargin(result) > pathMinimumMargin(*selected)))
        selected = &result;

    if (selected)
    {
      writeGraphPath(*selected);
      writeWaypoints(*selected);
    }
    else
      writeNotSelectedArtifacts();
    writeAudit(results, selected);
    return selected != nullptr;
  }

private:
  double jointMargin(const moveit::core::RobotState& state,
                     const std::vector<std::string>& names) const
  {
    double margin = kInfinity;
    for (const auto& name : names)
    {
      const auto& bounds = core_.robot_model_->getVariableBounds(name);
      if (!bounds.position_bounded_)
        continue;
      const double value = state.getVariablePosition(name);
      margin = std::min(margin, std::min(value - bounds.min_position_, bounds.max_position_ - value));
    }
    return margin;
  }

  Metrics evaluate(moveit::core::RobotState& state) const
  {
    state.setVariablePosition("waist_yaw_joint", 0.0);
    state.setVariablePosition("waist_pitch_joint", 0.0);
    state.update();
    const CollisionStatus status = core_.checkState(state);
    const auto clearance = core_.stateClearances(state);
    Metrics metrics;
    metrics.joint_margin = jointMargin(state, core_.whole_body_group_->getVariableNames());
    metrics.active_joint_margin = jointMargin(state, core_.left_arm_group_->getVariableNames());
    metrics.environment_clearance = clearance.first;
    metrics.self_clearance = clearance.second;
    metrics.joint_limit_valid = status.joint_limit_valid;
    metrics.collision_free = status.joint_limit_valid && !status.self_collision && !status.environment_collision;
    metrics.collision_pairs = pairString(status.pairs);
    if (!status.joint_limit_valid)
      metrics.failure_reason = "JOINT_LIMIT_VIOLATION";
    else if (status.self_collision || status.environment_collision)
      metrics.failure_reason = core_.collisionFailure(status);
    return metrics;
  }

  geometry_msgs::msg::Pose interpolatePose(const geometry_msgs::msg::Pose& from,
                                           const geometry_msgs::msg::Pose& to,
                                           double ratio) const
  {
    geometry_msgs::msg::Pose pose = from;
    pose.position.x += ratio * (to.position.x - from.position.x);
    pose.position.y += ratio * (to.position.y - from.position.y);
    pose.position.z += ratio * (to.position.z - from.position.z);
    // Orientation is deliberately copied from `from`; the audited grasp and
    // top poses have the same fixed orientation and no recovery rotation is allowed.
    return pose;
  }

  double jointDistance(const moveit::core::RobotState& a,
                       const moveit::core::RobotState& b) const
  {
    double squared = 0.0;
    for (const auto& name : core_.left_arm_group_->getVariableNames())
    {
      const double delta = a.getVariablePosition(name) - b.getVariablePosition(name);
      squared += delta * delta;
    }
    return std::sqrt(squared);
  }

  bool duplicate(const Node& candidate, const std::vector<Node>& nodes) const
  {
    return std::any_of(nodes.begin(), nodes.end(), [&](const Node& node) {
      return jointDistance(candidate.state, node.state) <= duplicate_tolerance_;
    });
  }

  void writeCandidateAttempt(double lift, const Layer& layer, int attempt_id, int seed_id,
                             const moveit::core::RobotState* state, const Metrics* metrics,
                             bool continuation_from_reachable, bool duplicate_state, bool retained) const
  {
    std::ofstream out(candidates_csv_, std::ios::app);
    out << std::setprecision(15) << lift << ',' << layer.waypoint << ','
        << layer.pose.position.x << ',' << layer.pose.position.y << ',' << layer.pose.position.z << ','
        << layer.pose.orientation.x << ',' << layer.pose.orientation.y << ','
        << layer.pose.orientation.z << ',' << layer.pose.orientation.w << ','
        << attempt_id << ',' << seed_id << ',' << (continuation_from_reachable ? 1 : 0) << ','
        << (state ? 1 : 0) << ',';
    if (metrics)
      out << (metrics->collision_free ? 1 : 0) << ',' << (duplicate_state ? 1 : 0) << ','
          << (retained ? 1 : 0) << ',' << csvEscape(metrics->failure_reason) << ','
          << csvEscape(metrics->collision_pairs) << ',' << metrics->joint_margin << ','
          << metrics->active_joint_margin << ',' << metrics->environment_clearance << ','
          << metrics->self_clearance;
    else
      out << "0,0,0,NO_IK_SOLUTION,,nan,nan,nan,nan";
    for (const auto& name : core_.whole_body_group_->getVariableNames())
      out << ',' << (state ? std::to_string(state->getVariablePosition(name)) : std::string());
    out << '\n';
  }

  Layer generateLayer(double lift, int waypoint, const geometry_msgs::msg::Pose& pose,
                      const moveit::core::RobotState& base, const std::vector<Node>* previous,
                      bool attached, bool apply_cap = true, bool deduplicate_nodes = true)
  {
    (void)attached;
    Layer layer(core_.robot_model_);
    layer.waypoint = waypoint;
    layer.pose = pose;
    const int continuation_seeds = previous ? static_cast<int>(previous->size()) : 0;
    const int attempts = continuation_seeds + ik_seeds_per_waypoint_;
    layer.ik_attempts = attempts;
    for (int attempt = 0; attempt < attempts; ++attempt)
    {
      moveit::core::RobotState state = base;
      int seed_id = attempt;
      bool continuation_from_reachable = false;
      if (previous && attempt < continuation_seeds)
      {
        state = (*previous)[attempt].state;
        seed_id = -1 - attempt;
        continuation_from_reachable = (*previous)[attempt].reachable;
      }
      else
      {
        const int random_id = attempt - continuation_seeds;
        seed_id = random_id;
        if (random_id > 0)
        {
          std::mt19937_64 rng(202608140000ULL +
            static_cast<std::uint64_t>(std::llround(lift * 1e6)) * 1000003ULL +
            static_cast<std::uint64_t>(waypoint) * 1009ULL + static_cast<std::uint64_t>(random_id));
          for (const auto& variable : core_.left_arm_group_->getVariableNames())
          {
            const auto& bounds = core_.robot_model_->getVariableBounds(variable);
            std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
            state.setVariablePosition(variable, distribution(rng));
          }
        }
      }
      state.setVariablePosition("lift_joint", lift);
      state.setVariablePosition("waist_yaw_joint", 0.0);
      state.setVariablePosition("waist_pitch_joint", 0.0);
      state.update();
      if (!state.setFromIK(core_.left_arm_group_, pose, core_.left_tcp_link_, core_.scene_config_.ik_timeout))
      {
        writeCandidateAttempt(lift, layer, attempt, seed_id, nullptr, nullptr,
                              continuation_from_reachable, false, false);
        continue;
      }
      ++layer.raw_ik_count;
      Metrics metrics = evaluate(state);
      if (!layer.raw_representative.available ||
          metrics.active_joint_margin > layer.raw_representative.metrics.active_joint_margin)
      {
        layer.raw_representative.state = state;
        layer.raw_representative.metrics = metrics;
        layer.raw_representative.seed_id = seed_id;
        layer.raw_representative.available = true;
      }
      if (!metrics.collision_free)
      {
        writeCandidateAttempt(lift, layer, attempt, seed_id, &state, &metrics,
                              continuation_from_reachable, false, false);
        continue;
      }
      ++layer.collision_free_count;
      Node node(core_.robot_model_);
      node.state = state;
      node.metrics = metrics;
      node.seed_id = seed_id;
      node.raw_attempt_id = attempt;
      node.continuation_from_reachable = continuation_from_reachable;
      const bool is_duplicate = deduplicate_nodes && duplicate(node, layer.nodes);
      const bool retained = !is_duplicate;
      writeCandidateAttempt(lift, layer, attempt, seed_id, &state, &metrics,
                            continuation_from_reachable, is_duplicate, retained);
      if (!is_duplicate)
        layer.nodes.push_back(std::move(node));
    }
    std::stable_sort(layer.nodes.begin(), layer.nodes.end(), [](const Node& a, const Node& b) {
      if (a.continuation_from_reachable != b.continuation_from_reachable)
        return a.continuation_from_reachable;
      if (std::abs(a.metrics.active_joint_margin - b.metrics.active_joint_margin) > 1e-12)
        return a.metrics.active_joint_margin > b.metrics.active_joint_margin;
      if (std::abs(a.metrics.environment_clearance - b.metrics.environment_clearance) > 1e-12)
        return a.metrics.environment_clearance > b.metrics.environment_clearance;
      if (std::abs(a.metrics.self_clearance - b.metrics.self_clearance) > 1e-12)
        return a.metrics.self_clearance > b.metrics.self_clearance;
      return a.seed_id < b.seed_id;
    });
    if (apply_cap && layer.nodes.size() > static_cast<std::size_t>(max_candidates_per_waypoint_))
      layer.nodes.erase(layer.nodes.begin() + max_candidates_per_waypoint_, layer.nodes.end());
    return layer;
  }

  bool edgeValid(const moveit::core::RobotState& from, const moveit::core::RobotState& to,
                 double& displacement, double& edge_cost, double& min_joint_margin,
                 double& min_active_joint_margin, double& min_environment_clearance,
                 double& min_self_clearance,
                 std::string& failure_reason, std::string& collision_pairs) const
  {
    displacement = jointDistance(from, to);
    std::size_t interpolation_steps = 1;
    for (const auto& name : core_.whole_body_group_->getVariableNames())
    {
      const auto* joint = core_.robot_model_->getJointOfVariable(name);
      const double delta = std::abs(to.getVariablePosition(name) - from.getVariablePosition(name));
      const bool prismatic = joint->getType() == moveit::core::JointModel::PRISMATIC;
      const double max_edge_step = prismatic ? max_prismatic_edge_step_ : max_revolute_edge_step_;
      if (delta > max_edge_step + 1e-12)
      {
        failure_reason = prismatic ? "EDGE_PRISMATIC_STEP_EXCEEDED" : "EDGE_REVOLUTE_STEP_EXCEEDED";
        return false;
      }
      const double dense_resolution = prismatic ? 0.005 : 0.01;
      interpolation_steps = std::max(interpolation_steps,
        static_cast<std::size_t>(std::ceil(delta / dense_resolution)));
    }

    min_joint_margin = kInfinity;
    min_active_joint_margin = kInfinity;
    min_environment_clearance = kInfinity;
    min_self_clearance = kInfinity;
    for (std::size_t step = 1; step <= interpolation_steps; ++step)
    {
      moveit::core::RobotState state = from;
      const double ratio = static_cast<double>(step) / interpolation_steps;
      for (const auto& name : core_.whole_body_group_->getVariableNames())
        state.setVariablePosition(name, from.getVariablePosition(name) + ratio *
          (to.getVariablePosition(name) - from.getVariablePosition(name)));
      Metrics metrics = evaluate(state);
      min_joint_margin = std::min(min_joint_margin, metrics.joint_margin);
      min_active_joint_margin = std::min(min_active_joint_margin, metrics.active_joint_margin);
      min_environment_clearance = std::min(min_environment_clearance, metrics.environment_clearance);
      min_self_clearance = std::min(min_self_clearance, metrics.self_clearance);
      if (!metrics.collision_free)
      {
        failure_reason = metrics.failure_reason;
        collision_pairs = metrics.collision_pairs;
        return false;
      }
    }
    const double proximity = 1.0 / std::max(1e-6, min_active_joint_margin);
    edge_cost = displacement_weight_ * displacement + limit_proximity_weight_ * proximity;
    return true;
  }

  void connectLayers(double lift, Layer& previous, Layer& current) const
  {
    std::ofstream out(edges_csv_, std::ios::app);
    for (std::size_t target_index = 0; target_index < current.nodes.size(); ++target_index)
    {
      Node& target = current.nodes[target_index];
      for (std::size_t source_index = 0; source_index < previous.nodes.size(); ++source_index)
      {
        const Node& source = previous.nodes[source_index];
        if (!source.reachable)
          continue;
        ++current.evaluated_edges_from_reachable;
        double displacement{}, edge_cost{}, min_joint{}, min_active{}, min_environment{}, min_self{};
        std::string reason, pairs;
        const bool valid = edgeValid(source.state, target.state, displacement, edge_cost,
                                     min_joint, min_active, min_environment, min_self, reason, pairs);
        if (valid)
          ++current.connected_edges_from_reachable;
        out << std::setprecision(15) << lift << ',' << previous.waypoint << ',' << source_index << ','
            << current.waypoint << ',' << target_index << ',' << (valid ? 1 : 0) << ','
            << displacement << ',' << edge_cost << ',' << min_joint << ',' << min_active << ','
            << min_environment << ',' << min_self << ',' << csvEscape(reason) << ','
            << csvEscape(pairs) << '\n';
        if (!valid)
          continue;
        const double candidate_cost = source.cost + edge_cost;
        if (candidate_cost < target.cost)
        {
          target.cost = candidate_cost;
          target.predecessor = static_cast<int>(source_index);
          target.reachable = true;
        }
      }
    }
  }

  GraphResult buildGraph(double lift, const geometry_msgs::msg::Pose& from,
                         const geometry_msgs::msg::Pose& to, bool attached)
  {
    GraphResult result;
    result.lift = lift;
    Candidate candidate;
    candidate.id = "stage_v3_lift_" + std::to_string(lift);
    candidate.lift = lift;
    candidate.yaw = candidate.pitch = 0.0;
    moveit::core::RobotState base = core_.initialState(candidate);
    const Eigen::Vector3d start(from.position.x, from.position.y, from.position.z);
    const Eigen::Vector3d finish(to.position.x, to.position.y, to.position.z);
    const int intervals = std::max(1, static_cast<int>(std::ceil((finish - start).norm() / cartesian_spacing_)));

    for (int waypoint = 0; waypoint <= intervals; ++waypoint)
    {
      const double ratio = static_cast<double>(waypoint) / intervals;
      const auto pose = interpolatePose(from, to, ratio);
      const std::vector<Node>* previous_nodes = result.layers.empty() ? nullptr : &result.layers.back().nodes;
      Layer layer = generateLayer(lift, waypoint, pose, base, previous_nodes, attached);
      if (waypoint == 0)
      {
        for (auto& node : layer.nodes)
        {
          node.cost = limit_proximity_weight_ /
            std::max(1e-6, node.metrics.joint_margin);
          node.reachable = true;
        }
      }
      else
        connectLayers(lift, result.layers.back(), layer);

      const bool any_reachable = std::any_of(layer.nodes.begin(), layer.nodes.end(),
                                             [](const Node& node) { return node.reachable; });
      layer.reachable_nodes = static_cast<std::size_t>(std::count_if(
        layer.nodes.begin(), layer.nodes.end(), [](const Node& node) { return node.reachable; }));
      writeLayer(lift, layer);
      result.layers.push_back(std::move(layer));
      if (!any_reachable)
      {
        const Layer& failed = result.layers.back();
        result.failure_waypoint = waypoint;
        result.last_reachable_waypoint = waypoint - 1;
        if (failed.raw_ik_count == 0)
        {
          result.first_node_empty_waypoint = waypoint;
          result.failure_label = "NO_IK_SOLUTION_AT_WAYPOINT";
          result.failure_reason = "No IK solution was returned by deterministic multistart at the fixed pose";
        }
        else if (failed.nodes.empty())
        {
          result.first_node_empty_waypoint = waypoint;
          result.failure_label = "NO_COLLISION_FREE_IK_AT_WAYPOINT";
          result.failure_reason = "Raw IK exists, but no candidate satisfies limits and collision checks";
          result.collision_pairs = failed.raw_representative.metrics.collision_pairs;
        }
        else
        {
          result.first_edge_gap_waypoint = waypoint;
          result.failure_label = "IK_BRANCH_CONNECTIVITY_GAP";
          result.failure_reason = "Collision-free IK candidates exist, but no valid edge reaches them from the previous layer";
        }
        return result;
      }
      result.last_reachable_waypoint = waypoint;
    }

    const Layer& final_layer = result.layers.back();
    int best = -1;
    for (std::size_t index = 0; index < final_layer.nodes.size(); ++index)
      if (final_layer.nodes[index].reachable &&
          (best < 0 || final_layer.nodes[index].cost < final_layer.nodes[best].cost))
        best = static_cast<int>(index);
    if (best < 0)
    {
      result.failure_waypoint = intervals;
      result.failure_label = "IK_BRANCH_CONNECTIVITY_GAP";
      result.failure_reason = "No reachable terminal node";
      return result;
    }

    result.path.resize(result.layers.size(), base);
    int node_index = best;
    for (int layer_index = static_cast<int>(result.layers.size()) - 1; layer_index >= 0; --layer_index)
    {
      const Node& node = result.layers[layer_index].nodes[node_index];
      result.path[layer_index] = node.state;
      node_index = node.predecessor;
      if (layer_index > 0 && node_index < 0)
        throw std::runtime_error("Layered IK predecessor chain terminated early");
    }
    result.success = true;
    result.failure_label = "GRAPH_PATH_FOUND";
    result.failure_reason = "Full fixed-orientation reverse Cartesian IK graph path found";
    return result;
  }

  double pathMinimumMargin(const GraphResult& result) const
  {
    double margin = kInfinity;
    for (auto state : result.path)
      margin = std::min(margin, evaluate(state).active_joint_margin);
    return margin;
  }

  void initializeOutputs() const
  {
    {
      std::ofstream out(summary_csv_, std::ios::trunc);
      out << "timestamp,lift,success,last_reachable_waypoint,first_failed_waypoint,failure_label,"
             "failure_reason,collision_pairs,first_node_empty_waypoint,first_edge_gap_waypoint,"
             "total_layers,total_path_states\n";
    }
    {
      std::ofstream out(candidates_csv_, std::ios::trunc);
      out << "lift,waypoint,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw,attempt_id,seed_id,"
             "continuation_from_reachable,ik_success,collision_free,duplicate,deduplicated_pre_cap,"
             "failure_reason,collision_pairs,joint_margin,"
             "active_joint_margin,environment_clearance,self_clearance";
      for (const auto& name : core_.whole_body_group_->getVariableNames())
        out << ',' << name;
      out << '\n';
    }
    {
      std::ofstream out(layers_csv_, std::ios::trunc);
      out << "lift,waypoint,pose_x,pose_y,pose_z,ik_attempts,raw_ik_count,collision_free_ik_count,"
             "retained_graph_nodes,evaluated_edges_from_reachable,connected_edges_from_reachable,"
             "reachable_graph_nodes\n";
    }
    {
      std::ofstream out(edges_csv_, std::ios::trunc);
      out << "lift,from_waypoint,from_candidate,to_waypoint,to_candidate,valid,joint_displacement,"
             "edge_cost,min_joint_margin,min_active_joint_margin,min_environment_clearance,min_self_clearance,rejection_reason,"
             "collision_pairs\n";
    }
    {
      std::ofstream out(boundary_csv_, std::ios::trunc);
      out << "lift,boundary_role,waypoint,state_source,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw,"
             "joint_margin,active_joint_margin,environment_clearance,self_clearance,failure_label,"
             "collision_pairs";
      for (const auto& name : core_.whole_body_group_->getVariableNames())
        out << ',' << name;
      out << '\n';
    }
  }

  void writeSummary(const GraphResult& result) const
  {
    std::ofstream out(summary_csv_, std::ios::app);
    out << csvEscape(timestampNow()) << ',' << result.lift << ',' << (result.success ? 1 : 0) << ','
        << result.last_reachable_waypoint << ',' << result.failure_waypoint << ','
        << csvEscape(result.failure_label) << ',' << csvEscape(result.failure_reason) << ','
        << csvEscape(result.collision_pairs) << ',' << result.first_node_empty_waypoint << ','
        << result.first_edge_gap_waypoint << ',' << result.layers.size() << ',' << result.path.size() << '\n';
  }

  void writeLayer(double lift, const Layer& layer) const
  {
    std::ofstream out(layers_csv_, std::ios::app);
    out << std::setprecision(15) << lift << ',' << layer.waypoint << ',' << layer.pose.position.x << ','
        << layer.pose.position.y << ',' << layer.pose.position.z << ',' << layer.ik_attempts << ','
        << layer.raw_ik_count << ',' << layer.collision_free_count << ',' << layer.nodes.size() << ','
        << layer.evaluated_edges_from_reachable << ',' << layer.connected_edges_from_reachable << ','
        << layer.reachable_nodes << '\n';
  }

  void writeBoundaryState(std::ofstream& out, double lift, const std::string& role,
                          const Layer& layer, const std::string& source,
                          const moveit::core::RobotState* state, const Metrics* metrics,
                          const std::string& label, const std::string& pairs) const
  {
    out << std::setprecision(15) << lift << ',' << role << ',' << layer.waypoint << ',' << source << ','
        << layer.pose.position.x << ',' << layer.pose.position.y << ',' << layer.pose.position.z << ','
        << layer.pose.orientation.x << ',' << layer.pose.orientation.y << ',' << layer.pose.orientation.z << ','
        << layer.pose.orientation.w << ',';
    if (metrics)
      out << metrics->joint_margin << ',' << metrics->active_joint_margin << ','
          << metrics->environment_clearance << ',' << metrics->self_clearance;
    else
      out << "nan,nan,nan,nan";
    out << ',' << label << ',' << csvEscape(pairs);
    for (const auto& name : core_.whole_body_group_->getVariableNames())
      out << ',' << (state ? std::to_string(state->getVariablePosition(name)) : std::string());
    out << '\n';
  }

  void writeBoundary(const GraphResult& result) const
  {
    if (result.success || result.layers.empty())
      return;
    std::ofstream out(boundary_csv_, std::ios::app);
    if (result.last_reachable_waypoint >= 0)
    {
      const Layer& last = result.layers[result.last_reachable_waypoint];
      const Node* best = nullptr;
      for (const auto& node : last.nodes)
        if (node.reachable && (!best || node.cost < best->cost))
          best = &node;
      if (best)
        writeBoundaryState(out, result.lift, "LAST_REACHABLE", last, "REACHABLE_GRAPH_NODE",
                           &best->state, &best->metrics, result.failure_label, best->metrics.collision_pairs);
    }
    const Layer& failed = result.layers.back();
    const Node* collision_free = failed.nodes.empty() ? nullptr : &failed.nodes.front();
    if (collision_free)
      writeBoundaryState(out, result.lift, "FIRST_FAILED", failed, "UNREACHABLE_COLLISION_FREE_IK",
                         &collision_free->state, &collision_free->metrics,
                         result.failure_label, collision_free->metrics.collision_pairs);
    else if (failed.raw_representative.available)
      writeBoundaryState(out, result.lift, "FIRST_FAILED", failed, "RAW_IK_REJECTED_BY_VALIDITY",
                         &failed.raw_representative.state, &failed.raw_representative.metrics,
                         result.failure_label, failed.raw_representative.metrics.collision_pairs);
    else
      writeBoundaryState(out, result.lift, "FIRST_FAILED", failed, "NO_IK_STATE_EXISTS",
                         nullptr, nullptr, result.failure_label, "");
  }

  void writeGraphPath(const GraphResult& result) const
  {
    std::ofstream out(graph_path_csv_, std::ios::trunc);
    out << "stage,path_index,source_waypoint";
    for (const auto& name : core_.whole_body_group_->getVariableNames())
      out << ',' << name;
    out << ",tcp_x,tcp_y,tcp_z,joint_margin,active_joint_margin,environment_clearance,self_clearance\n";
    for (std::size_t index = 0; index < result.path.size(); ++index)
    {
      moveit::core::RobotState state = result.path[result.path.size() - 1 - index];
      const Metrics metrics = evaluate(state);
      const auto& tcp = state.getGlobalLinkTransform(core_.left_tcp_link_);
      out << "VERTICAL_DESCENT_GRAPH," << index << ',' << result.path.size() - 1 - index;
      for (const auto& name : core_.whole_body_group_->getVariableNames())
        out << ',' << std::setprecision(15) << state.getVariablePosition(name);
      out << ',' << tcp.translation().x() << ',' << tcp.translation().y() << ',' << tcp.translation().z()
          << ',' << metrics.joint_margin << ',' << metrics.active_joint_margin << ','
          << metrics.environment_clearance << ',' << metrics.self_clearance << '\n';
    }
  }

  void writeWaypoints(const GraphResult& result) const
  {
    std::ofstream out(waypoints_yaml_, std::ios::trunc);
    out << "status: GRAPH_RECOVERY_FOUND_LOCAL_FULL_STAGE_VALIDATION_PENDING\n"
        << "protocol: STAGE_CONSTRAINED_REFERENCE_V3\n"
        << "planning_only: true\ntrajectory_execution_performed: false\n"
        << "lift: " << result.lift << "\nyaw: 0.0\npitch: 0.0\n"
        << "cartesian_spacing_m: " << cartesian_spacing_ << "\n"
        << "reverse_graph_waypoints: " << result.path.size() << "\n"
        << "note: A graph path is diagnostic evidence, not a complete five-stage reference trajectory.\n";
  }

  void writeNotSelectedArtifacts() const
  {
    std::ofstream path(graph_path_csv_, std::ios::trunc);
    path << "status,reason\nNOT_SELECTED,NO_COMPLETE_FIXED_ORIENTATION_LAYERED_IK_PATH\n";
    std::ofstream waypoints(waypoints_yaml_, std::ios::trunc);
    waypoints << "status: NOT_SELECTED\nreason: NO_COMPLETE_FIXED_ORIENTATION_LAYERED_IK_PATH\n";
  }

  bool isContinuousRevolute(const std::string& variable) const
  {
    const auto* joint = core_.robot_model_->getJointOfVariable(variable);
    if (!joint || joint->getType() != moveit::core::JointModel::REVOLUTE)
      return false;
    const auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint);
    return revolute && revolute->isContinuous();
  }

  std::string jointType(const std::string& variable) const
  {
    const auto* joint = core_.robot_model_->getJointOfVariable(variable);
    if (!joint)
      return "UNKNOWN";
    if (joint->getType() == moveit::core::JointModel::REVOLUTE)
      return isContinuousRevolute(variable) ? "REVOLUTE_CONTINUOUS" : "REVOLUTE_BOUNDED";
    if (joint->getType() == moveit::core::JointModel::PRISMATIC)
      return "PRISMATIC";
    return "OTHER";
  }

  double correctedDelta(const std::string& variable, double from, double to) const
  {
    const double raw = to - from;
    if (!isContinuousRevolute(variable))
      return raw;
    return std::remainder(raw, 2.0 * kPi);
  }

  AuditEdgeResult auditEdge(const moveit::core::RobotState& from,
                            const moveit::core::RobotState& to) const
  {
    AuditEdgeResult result;
    double corrected_squared = 0.0;
    double raw_squared = 0.0;
    std::size_t interpolation_steps = 1;
    for (const auto& name : core_.whole_body_group_->getVariableNames())
    {
      const auto* joint = core_.robot_model_->getJointOfVariable(name);
      const double raw = to.getVariablePosition(name) - from.getVariablePosition(name);
      const double delta = correctedDelta(name, from.getVariablePosition(name), to.getVariablePosition(name));
      const auto& active_variables = core_.left_arm_group_->getVariableNames();
      if (std::find(active_variables.begin(), active_variables.end(), name) != active_variables.end())
      {
        raw_squared += raw * raw;
        corrected_squared += delta * delta;
      }
      if (std::abs(delta) > result.max_corrected_delta)
      {
        result.max_corrected_delta = std::abs(delta);
        result.max_delta_joint = name;
      }
      const bool prismatic = joint && joint->getType() == moveit::core::JointModel::PRISMATIC;
      const double threshold = prismatic ? max_prismatic_edge_step_ : max_revolute_edge_step_;
      result.raw_step_exceeded = result.raw_step_exceeded || std::abs(raw) > threshold + 1e-12;
      result.corrected_step_exceeded = result.corrected_step_exceeded || std::abs(delta) > threshold + 1e-12;
      const double resolution = prismatic ? 0.005 : 0.01;
      interpolation_steps = std::max(interpolation_steps,
        static_cast<std::size_t>(std::ceil(std::abs(delta) / resolution)));
    }
    result.raw_displacement = std::sqrt(raw_squared);
    result.displacement = std::sqrt(corrected_squared);
    if (result.corrected_step_exceeded)
    {
      result.failure_reason = "EDGE_STEP_EXCEEDED:" + result.max_delta_joint;
      return result;
    }
    for (std::size_t step = 1; step <= interpolation_steps; ++step)
    {
      moveit::core::RobotState state = from;
      const double ratio = static_cast<double>(step) / interpolation_steps;
      for (const auto& name : core_.whole_body_group_->getVariableNames())
      {
        const double delta = correctedDelta(name, from.getVariablePosition(name), to.getVariablePosition(name));
        state.setVariablePosition(name, from.getVariablePosition(name) + ratio * delta);
      }
      Metrics metrics = evaluate(state);
      if (!metrics.collision_free)
      {
        result.failure_reason = metrics.failure_reason;
        result.collision_pairs = metrics.collision_pairs;
        return result;
      }
    }
    result.valid = true;
    return result;
  }

  bool presentInCappedLayer(const Node& candidate, const Layer& capped) const
  {
    return std::any_of(capped.nodes.begin(), capped.nodes.end(), [&](const Node& retained) {
      return jointDistance(candidate.state, retained.state) <= duplicate_tolerance_;
    });
  }

  std::string zeroMarginJoints(const moveit::core::RobotState& state) const
  {
    std::ostringstream names;
    bool first = true;
    for (const auto& name : core_.left_arm_group_->getVariableNames())
    {
      const auto& bounds = core_.robot_model_->getVariableBounds(name);
      if (!bounds.position_bounded_)
        continue;
      const double value = state.getVariablePosition(name);
      const double margin = std::min(value - bounds.min_position_, bounds.max_position_ - value);
      if (margin > 1e-8)
        continue;
      if (!first)
        names << ';';
      names << name;
      first = false;
    }
    return names.str();
  }

  JacobianMetrics jacobianMetrics(moveit::core::RobotState state) const
  {
    state.update();
    const auto* link = core_.robot_model_->getLinkModel(core_.left_tcp_link_);
    Eigen::MatrixXd jacobian;
    if (!state.getJacobian(core_.left_arm_group_, link, Eigen::Vector3d::Zero(), jacobian, false))
      throw std::runtime_error("Failed to compute left-arm TCP Jacobian");
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = svd.singularValues();
    JacobianMetrics metrics;
    metrics.minimum_singular_value = singular.size() ? singular.minCoeff() : 0.0;
    metrics.maximum_singular_value = singular.size() ? singular.maxCoeff() : 0.0;
    metrics.condition_number = metrics.minimum_singular_value > 1e-15 ?
      metrics.maximum_singular_value / metrics.minimum_singular_value : kInfinity;
    metrics.manipulability = 1.0;
    std::ostringstream values;
    for (Eigen::Index index = 0; index < singular.size(); ++index)
    {
      metrics.manipulability *= singular[index];
      if (index)
        values << ';';
      values << std::setprecision(15) << singular[index];
    }
    metrics.singular_values = values.str();
    return metrics;
  }

  void initializeBoundaryAuditOutputs() const
  {
    std::ofstream summary(boundary_audit_summary_csv_, std::ios::trunc);
    summary << "lift,previous_waypoint,target_waypoint,previous_reachable_candidates,target_raw_ik,"
               "target_collision_free_ik,target_uncapped_candidates,target_cap48_candidates,total_edges,"
               "connected_edges,connected_edges_removed_by_cap,representation_only_edges,refinement_paths,"
               "minimum_joint_displacement,closest_previous,closest_target,previous_zero_margin_joints,"
               "target_zero_margin_joints,classification\n";
    std::ofstream edges(boundary_audit_edges_csv_, std::ios::trunc);
    edges << "lift,previous_waypoint,target_waypoint,previous_candidate,target_candidate,target_in_cap48,"
             "valid,raw_step_exceeded,corrected_step_exceeded,raw_displacement,corrected_displacement,"
             "max_corrected_delta,max_delta_joint,failure_reason,collision_pairs\n";
    std::ofstream joints(boundary_audit_joints_csv_, std::ios::trunc);
    joints << "lift,previous_candidate,target_candidate,edge_valid,joint_name,joint_type,continuous,"
              "previous_value,target_value,raw_delta,corrected_delta,lower_limit,upper_limit,"
              "previous_margin,target_margin\n";
    std::ofstream closest(boundary_audit_closest_csv_, std::ios::trunc);
    closest << "lift,previous_candidate,target_candidate,joint_name,joint_type,continuous,previous_value,"
               "target_value,raw_delta,corrected_delta,lower_limit,upper_limit,previous_margin,target_margin\n";
    std::ofstream jacobian(boundary_audit_jacobian_csv_, std::ios::trunc);
    jacobian << "lift,boundary_role,waypoint,minimum_singular_value,maximum_singular_value,condition_number,"
                "manipulability,singular_values,zero_margin_joints\n";
    std::ofstream continuation(boundary_audit_continuation_csv_, std::ios::trunc);
    continuation << "lift,previous_candidate,raw_ik,collision_free,present_in_uncapped,present_in_cap48,"
                    "joint_displacement,max_corrected_delta,max_delta_joint,edge_valid,failure_reason,"
                    "active_joint_margin,environment_clearance,self_clearance,collision_pairs\n";
    std::ofstream refinement(boundary_audit_refinement_csv_, std::ios::trunc);
    refinement << "lift,spacing_m,previous_candidate,substep,total_substeps,pose_z,raw_ik,collision_free,"
                  "edge_valid,joint_displacement,max_corrected_delta,max_delta_joint,failure_reason,"
                  "active_joint_margin,environment_clearance,self_clearance,collision_pairs\n";
  }

  std::size_t auditRefinement(double lift, const Layer& previous, const geometry_msgs::msg::Pose& target_pose) const
  {
    std::ofstream out(boundary_audit_refinement_csv_, std::ios::app);
    std::size_t completed_paths = 0;
    for (const double spacing : { 0.0005, 0.00025, 0.0001 })
    {
      const double distance = std::abs(target_pose.position.z - previous.pose.position.z);
      const int total_steps = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
      for (std::size_t source_index = 0; source_index < previous.nodes.size(); ++source_index)
      {
        if (!previous.nodes[source_index].reachable)
          continue;
        moveit::core::RobotState current = previous.nodes[source_index].state;
        bool complete = true;
        for (int step = 1; step <= total_steps; ++step)
        {
          const double ratio = static_cast<double>(step) / total_steps;
          const auto pose = interpolatePose(previous.pose, target_pose, ratio);
          moveit::core::RobotState next = current;
          const bool raw_ik = next.setFromIK(core_.left_arm_group_, pose, core_.left_tcp_link_,
                                             core_.scene_config_.ik_timeout);
          Metrics metrics;
          AuditEdgeResult edge;
          if (raw_ik)
          {
            metrics = evaluate(next);
            if (metrics.collision_free)
              edge = auditEdge(current, next);
          }
          const std::string reason = !raw_ik ? "NO_IK_SOLUTION" :
            (!metrics.collision_free ? metrics.failure_reason : edge.failure_reason);
          out << std::setprecision(15) << lift << ',' << spacing << ',' << source_index << ',' << step << ','
              << total_steps << ',' << pose.position.z << ',' << (raw_ik ? 1 : 0) << ','
              << (raw_ik && metrics.collision_free ? 1 : 0) << ',' << (edge.valid ? 1 : 0) << ','
              << edge.displacement << ',' << edge.max_corrected_delta << ',' << edge.max_delta_joint << ','
              << csvEscape(reason) << ',' << metrics.active_joint_margin << ','
              << metrics.environment_clearance << ',' << metrics.self_clearance << ','
              << csvEscape(metrics.collision_pairs) << '\n';
          if (!raw_ik || !metrics.collision_free || !edge.valid)
          {
            complete = false;
            break;
          }
          current = next;
        }
        if (complete)
          ++completed_paths;
      }
    }
    return completed_paths;
  }

  BoundaryAuditResult auditBoundary(double lift, int previous_waypoint, int target_waypoint)
  {
    core_.resetSceneForCandidate();
    Candidate candidate;
    candidate.id = "stage_v3_boundary_audit";
    candidate.lift = lift;
    candidate.yaw = candidate.pitch = 0.0;
    moveit::core::RobotState base = core_.initialState(candidate);
    const auto from = core_.graspPose();
    const auto to = core_.approachPose(core_.scene_config_.rim_clearance);
    const Eigen::Vector3d a(from.position.x, from.position.y, from.position.z);
    const Eigen::Vector3d b(to.position.x, to.position.y, to.position.z);
    const int intervals = std::max(1, static_cast<int>(std::ceil((b - a).norm() / cartesian_spacing_)));
    std::vector<Layer> layers;
    for (int waypoint = 0; waypoint <= previous_waypoint; ++waypoint)
    {
      const auto pose = interpolatePose(from, to, static_cast<double>(waypoint) / intervals);
      const std::vector<Node>* previous_nodes = layers.empty() ? nullptr : &layers.back().nodes;
      Layer layer = generateLayer(lift, waypoint, pose, base, previous_nodes, false);
      if (waypoint == 0)
      {
        for (auto& node : layer.nodes)
        {
          node.reachable = true;
          node.cost = limit_proximity_weight_ / std::max(1e-6, node.metrics.active_joint_margin);
        }
      }
      else
        connectLayers(lift, layers.back(), layer);
      layer.reachable_nodes = static_cast<std::size_t>(std::count_if(
        layer.nodes.begin(), layer.nodes.end(), [](const Node& node) { return node.reachable; }));
      writeLayer(lift, layer);
      if (layer.reachable_nodes == 0)
        throw std::runtime_error("Boundary audit could not reproduce reachable previous waypoint " +
                                 std::to_string(waypoint));
      layers.push_back(std::move(layer));
    }
    Layer& previous = layers.back();
    const auto target_pose = interpolatePose(from, to, static_cast<double>(target_waypoint) / intervals);
    Layer capped = generateLayer(lift, target_waypoint, target_pose, base, &previous.nodes, false);
    Layer uncapped = generateLayer(lift, target_waypoint, target_pose, base, &previous.nodes,
                                   false, false, false);

    BoundaryAuditResult result;
    result.lift = lift;
    result.previous_waypoint = previous_waypoint;
    result.target_waypoint = target_waypoint;
    result.previous_reachable_candidates = previous.reachable_nodes;
    result.target_raw_ik = uncapped.raw_ik_count;
    result.target_collision_free_ik = uncapped.collision_free_count;
    result.target_uncapped_candidates = uncapped.nodes.size();
    result.target_cap48_candidates = capped.nodes.size();

    std::ofstream edge_out(boundary_audit_edges_csv_, std::ios::app);
    std::ofstream joint_out(boundary_audit_joints_csv_, std::ios::app);
    for (std::size_t source_index = 0; source_index < previous.nodes.size(); ++source_index)
    {
      if (!previous.nodes[source_index].reachable)
        continue;
      for (std::size_t target_index = 0; target_index < uncapped.nodes.size(); ++target_index)
      {
        const Node& source = previous.nodes[source_index];
        const Node& target = uncapped.nodes[target_index];
        const bool in_cap = presentInCappedLayer(target, capped);
        const AuditEdgeResult edge = auditEdge(source.state, target.state);
        ++result.total_edges;
        if (edge.valid)
        {
          ++result.connected_edges;
          if (!in_cap)
            ++result.connected_edges_removed_by_cap;
          if (edge.raw_step_exceeded && !edge.corrected_step_exceeded)
            ++result.representation_only_edges;
        }
        if (edge.displacement < result.minimum_displacement)
        {
          result.minimum_displacement = edge.displacement;
          result.closest_previous = static_cast<int>(source_index);
          result.closest_target = static_cast<int>(target_index);
        }
        edge_out << std::setprecision(15) << lift << ',' << previous_waypoint << ',' << target_waypoint << ','
                 << source_index << ',' << target_index << ',' << (in_cap ? 1 : 0) << ','
                 << (edge.valid ? 1 : 0) << ',' << (edge.raw_step_exceeded ? 1 : 0) << ','
                 << (edge.corrected_step_exceeded ? 1 : 0) << ',' << edge.raw_displacement << ','
                 << edge.displacement << ',' << edge.max_corrected_delta << ',' << edge.max_delta_joint << ','
                 << csvEscape(edge.failure_reason) << ',' << csvEscape(edge.collision_pairs) << '\n';
        for (const auto& name : core_.whole_body_group_->getVariableNames())
        {
          const auto& bounds = core_.robot_model_->getVariableBounds(name);
          const double before = source.state.getVariablePosition(name);
          const double after = target.state.getVariablePosition(name);
          const double raw = after - before;
          const double corrected = correctedDelta(name, before, after);
          const double lower = bounds.position_bounded_ ? bounds.min_position_ :
            std::numeric_limits<double>::quiet_NaN();
          const double upper = bounds.position_bounded_ ? bounds.max_position_ :
            std::numeric_limits<double>::quiet_NaN();
          const double before_margin = bounds.position_bounded_ ?
            std::min(before - lower, upper - before) : kInfinity;
          const double after_margin = bounds.position_bounded_ ?
            std::min(after - lower, upper - after) : kInfinity;
          joint_out << std::setprecision(15) << lift << ',' << source_index << ',' << target_index << ','
                    << (edge.valid ? 1 : 0) << ',' << name << ',' << jointType(name) << ','
                    << (isContinuousRevolute(name) ? 1 : 0) << ',' << before << ',' << after << ','
                    << raw << ',' << corrected << ',' << lower << ',' << upper << ','
                    << before_margin << ',' << after_margin << '\n';
        }
      }
    }

    if (result.closest_previous < 0 || result.closest_target < 0)
      throw std::runtime_error("Boundary audit found no candidate pair");
    const Node& closest_previous = previous.nodes[result.closest_previous];
    const Node& closest_target = uncapped.nodes[result.closest_target];
    result.previous_zero_margin_joints = zeroMarginJoints(closest_previous.state);
    result.target_zero_margin_joints = zeroMarginJoints(closest_target.state);
    result.previous_jacobian = jacobianMetrics(closest_previous.state);
    result.target_jacobian = jacobianMetrics(closest_target.state);

    std::ofstream closest_out(boundary_audit_closest_csv_, std::ios::app);
    for (const auto& name : core_.whole_body_group_->getVariableNames())
    {
      const auto& bounds = core_.robot_model_->getVariableBounds(name);
      const double before = closest_previous.state.getVariablePosition(name);
      const double after = closest_target.state.getVariablePosition(name);
      const double lower = bounds.position_bounded_ ? bounds.min_position_ :
        std::numeric_limits<double>::quiet_NaN();
      const double upper = bounds.position_bounded_ ? bounds.max_position_ :
        std::numeric_limits<double>::quiet_NaN();
      closest_out << std::setprecision(15) << lift << ',' << result.closest_previous << ','
                  << result.closest_target << ',' << name << ',' << jointType(name) << ','
                  << (isContinuousRevolute(name) ? 1 : 0) << ',' << before << ',' << after << ','
                  << after - before << ',' << correctedDelta(name, before, after) << ',' << lower << ','
                  << upper << ',' << (bounds.position_bounded_ ? std::min(before - lower, upper - before) : kInfinity)
                  << ',' << (bounds.position_bounded_ ? std::min(after - lower, upper - after) : kInfinity) << '\n';
    }

    std::ofstream jacobian_out(boundary_audit_jacobian_csv_, std::ios::app);
    const auto write_jacobian = [&](const std::string& role, int waypoint, const JacobianMetrics& metrics,
                                    const std::string& zero_joints) {
      jacobian_out << std::setprecision(15) << lift << ',' << role << ',' << waypoint << ','
                   << metrics.minimum_singular_value << ',' << metrics.maximum_singular_value << ','
                   << metrics.condition_number << ',' << metrics.manipulability << ','
                   << csvEscape(metrics.singular_values) << ',' << csvEscape(zero_joints) << '\n';
    };
    write_jacobian("PREVIOUS_CLOSEST", previous_waypoint, result.previous_jacobian,
                   result.previous_zero_margin_joints);
    write_jacobian("TARGET_CLOSEST", target_waypoint, result.target_jacobian,
                   result.target_zero_margin_joints);

    std::ofstream continuation_out(boundary_audit_continuation_csv_, std::ios::app);
    for (std::size_t source_index = 0; source_index < previous.nodes.size(); ++source_index)
    {
      if (!previous.nodes[source_index].reachable)
        continue;
      moveit::core::RobotState direct = previous.nodes[source_index].state;
      const bool raw = direct.setFromIK(core_.left_arm_group_, target_pose, core_.left_tcp_link_,
                                        core_.scene_config_.ik_timeout);
      Metrics metrics;
      AuditEdgeResult edge;
      bool in_uncapped = false;
      bool in_cap = false;
      if (raw)
      {
        metrics = evaluate(direct);
        edge = auditEdge(previous.nodes[source_index].state, direct);
        Node probe(core_.robot_model_);
        probe.state = direct;
        in_uncapped = std::any_of(uncapped.nodes.begin(), uncapped.nodes.end(), [&](const Node& node) {
          return jointDistance(probe.state, node.state) <= duplicate_tolerance_;
        });
        in_cap = presentInCappedLayer(probe, capped);
      }
      continuation_out << std::setprecision(15) << lift << ',' << source_index << ',' << (raw ? 1 : 0) << ','
                       << (raw && metrics.collision_free ? 1 : 0) << ',' << (in_uncapped ? 1 : 0) << ','
                       << (in_cap ? 1 : 0) << ',' << edge.displacement << ',' << edge.max_corrected_delta << ','
                       << edge.max_delta_joint << ',' << (edge.valid ? 1 : 0) << ','
                       << csvEscape(raw ? edge.failure_reason : "NO_IK_SOLUTION") << ','
                       << metrics.active_joint_margin << ',' << metrics.environment_clearance << ','
                       << metrics.self_clearance << ',' << csvEscape(metrics.collision_pairs) << '\n';
    }

    result.refinement_paths = auditRefinement(lift, previous, target_pose);
    if (result.connected_edges_removed_by_cap > 0)
      result.classification = "CANDIDATE_CAP_PRUNING_ARTIFACT";
    else if (result.representation_only_edges > 0)
      result.classification = "REVOLUTE_ANGLE_REPRESENTATION_ARTIFACT";
    else if (result.connected_edges > 0 || result.refinement_paths > 0)
      result.classification = "GREEDY_OR_FRONTIER_PRUNING_ARTIFACT";
    else if (!result.previous_zero_margin_joints.empty() &&
             result.previous_jacobian.minimum_singular_value > 1e-3)
      result.classification = "JOINT_LIMIT_BRANCH_TERMINATION";
    else if (result.previous_jacobian.minimum_singular_value <= 1e-3)
      result.classification = "SINGULARITY_BRANCH_TERMINATION";
    else
      result.classification = "TRUE_IK_BRANCH_CONNECTIVITY_GAP";
    return result;
  }

  bool runBoundaryAudit()
  {
    initializeBoundaryAuditOutputs();
    std::vector<BoundaryAuditResult> results;
    results.push_back(auditBoundary(0.35, 78, 79));
    results.push_back(auditBoundary(0.40, 28, 29));
    std::ofstream summary(boundary_audit_summary_csv_, std::ios::app);
    std::ofstream audit(boundary_audit_md_, std::ios::trunc);
    audit << "# Stage v3 focused IK boundary audit\n\n"
             "Yaw and Pitch stayed at zero. No OMPL planning, RViz, controller, ros2_control, hardware, or trajectory execution was used.\n\n"
             "|Lift|Boundary|Reachable previous|Uncapped candidates|Cap-48 candidates|Edges|Connected|"
             "Removed by cap|Representation-only|Refinement paths|Minimum distance|Classification|\n"
             "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n";
    for (const auto& result : results)
    {
      summary << std::setprecision(15) << result.lift << ',' << result.previous_waypoint << ','
              << result.target_waypoint << ',' << result.previous_reachable_candidates << ','
              << result.target_raw_ik << ',' << result.target_collision_free_ik << ','
              << result.target_uncapped_candidates << ',' << result.target_cap48_candidates << ','
              << result.total_edges << ',' << result.connected_edges << ','
              << result.connected_edges_removed_by_cap << ',' << result.representation_only_edges << ','
              << result.refinement_paths << ',' << result.minimum_displacement << ','
              << result.closest_previous << ',' << result.closest_target << ','
              << csvEscape(result.previous_zero_margin_joints) << ','
              << csvEscape(result.target_zero_margin_joints) << ',' << result.classification << '\n';
      audit << '|' << result.lift << '|' << result.previous_waypoint << "->" << result.target_waypoint << '|'
            << result.previous_reachable_candidates << '|' << result.target_uncapped_candidates << '|'
            << result.target_cap48_candidates << '|' << result.total_edges << '|' << result.connected_edges << '|'
            << result.connected_edges_removed_by_cap << '|' << result.representation_only_edges << '|'
            << result.refinement_paths << '|' << result.minimum_displacement << '|'
            << result.classification << "|\n";
      RCLCPP_INFO(node_->get_logger(),
                  "STAGE_V3_BOUNDARY_AUDIT lift=%.2f boundary=%d->%d candidates=%zu edges=%zu connected=%zu refinement=%zu classification=%s",
                  result.lift, result.previous_waypoint, result.target_waypoint,
                  result.target_uncapped_candidates, result.total_edges, result.connected_edges,
                  result.refinement_paths, result.classification.c_str());
    }
    audit << "\n`UNRESOLVED` is intentionally retained when cap, angle representation, frontier expansion, and configured refinements do not restore an edge; Jacobian and joint-limit evidence must then determine the physical classification.\n";
    return true;
  }

  void writeAudit(const std::vector<GraphResult>& results, const GraphResult* selected) const
  {
    std::ofstream out(audit_md_, std::ios::trunc);
    out << "# Stage-constrained reference v3 layered IK audit\n\n"
           "This planning-only diagnostic preserves fixed TCP orientation, Yaw=0, Pitch=0, the audited scene, grasp, robot model, and MoveIt configuration. It does not execute trajectories.\n\n"
           "|Lift|Result|Last reachable|First failed|Failure label|Raw IK at boundary|Collision-free retained at boundary|\n"
           "|---:|---|---:|---:|---|---:|---:|\n";
    for (const auto& result : results)
    {
      const Layer* boundary = result.success || result.layers.empty() ? nullptr : &result.layers.back();
      out << '|' << result.lift << '|' << (result.success ? "GRAPH_PATH_FOUND" : "FAIL") << '|'
          << result.last_reachable_waypoint << '|' << result.failure_waypoint << '|'
          << result.failure_label << '|' << (boundary ? boundary->raw_ik_count : 0) << '|'
          << (boundary ? boundary->nodes.size() : 0) << "|\n";
    }
    out << "\n## Interpretation\n\n"
           "- `NO_IK_SOLUTION_AT_WAYPOINT`: deterministic multistart returned no state at the exact fixed pose.\n"
           "- `NO_COLLISION_FREE_IK_AT_WAYPOINT`: raw IK exists, but all solutions fail exact bounds or collision checks.\n"
           "- `IK_BRANCH_CONNECTIVITY_GAP`: collision-free states exist on both sides, but no edge satisfies step, dense bounds, self-collision, and environment-collision checks.\n"
           "- `GRAPH_PATH_FOUND`: dynamic programming found a complete reverse path; only then can v2 greedy pruning be tested as an explanation.\n\n";
    if (selected)
      out << "Selected reverse graph lift: " << selected->lift
          << ". Full five-stage validation remains required before promotion.\n";
    else
      out << "No graph path was selected. Fixed-orientation structural impossibility is not claimed beyond the configured samples, candidate cap, and edge thresholds.\n";
    out << "\nIf no IK state exists at the failed pose, boundary joint metrics are `nan` with `state_source=NO_IK_STATE_EXISTS`; values are never copied from another pose or represented as infinity.\n";
  }

  rclcpp::Node::SharedPtr node_;
  ReferenceTrajectoryGenerator core_;
  std::string summary_csv_;
  std::string candidates_csv_;
  std::string layers_csv_;
  std::string edges_csv_;
  std::string boundary_csv_;
  std::string graph_path_csv_;
  std::string waypoints_yaml_;
  std::string audit_md_;
  int ik_seeds_per_waypoint_{};
  int max_candidates_per_waypoint_{};
  double cartesian_spacing_{};
  double duplicate_tolerance_{};
  double max_revolute_edge_step_{};
  double max_prismatic_edge_step_{};
  double displacement_weight_{};
  double limit_proximity_weight_{};
  bool boundary_audit_only_{ false };
  std::string boundary_audit_summary_csv_;
  std::string boundary_audit_edges_csv_;
  std::string boundary_audit_joints_csv_;
  std::string boundary_audit_closest_csv_;
  std::string boundary_audit_jacobian_csv_;
  std::string boundary_audit_continuation_csv_;
  std::string boundary_audit_refinement_csv_;
  std::string boundary_audit_md_;
};
}  // namespace stage_constrained_v3

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("stage_v3_reference_generator", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  int exit_code = 1;
  try
  {
    stage_constrained_v3::Runner runner(node);
    exit_code = runner.run() ? 0 : 2;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Stage v3 generator failed: %s", error.what());
  }
  executor.cancel();
  if (spin_thread.joinable())
    spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
