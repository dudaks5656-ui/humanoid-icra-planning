#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// Reuse only the audited data types and YAML helpers from the reference source.
// ReferenceTrajectoryGenerator is never instantiated: this pilot creates a local
// RobotModel and PlanningScene and therefore does not start move_group or OMPL.
#define main preserved_reference_trajectory_generator_main_for_box_pilot
#include "reference_trajectory_generator.cpp"
#undef main

namespace box_target_yaw_pitch_pilot_v1
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct PositionSpec
{
  std::string id;
  double x{};
  double y{};
  double z{};
  double inner_wall_clearance{};
};

struct Metrics
{
  bool bounds_valid{ false };
  bool self_collision{ false };
  bool environment_collision{ false };
  double joint3_margin{ std::numeric_limits<double>::quiet_NaN() };
  double joint5_margin{ std::numeric_limits<double>::quiet_NaN() };
  double active_margin{ std::numeric_limits<double>::quiet_NaN() };
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
  double attached_min_z{ std::numeric_limits<double>::quiet_NaN() };
  double object_clearance{ std::numeric_limits<double>::quiet_NaN() };
  std::string collision_pairs;
};

struct GraspCandidate
{
  moveit::core::RobotState state;
  Metrics metrics;
  double yaw{};
  double pitch{};
  int seed_id{ -1 };

  explicit GraspCandidate(const moveit::core::RobotModelConstPtr& model) : state(model) {}
};

struct SearchResult
{
  int attempts{};
  int raw_ik{};
  int collision_free_ik{};
  int coarse_attempts{};
  int fine_attempts{};
  bool found{ false };
  bool collision_example_found{ false };
  Metrics collision_example;
  GraspCandidate selected;

  explicit SearchResult(const moveit::core::RobotModelConstPtr& model) : selected(model) {}
};

struct ModeResult
{
  PositionSpec position;
  std::string mode;
  double lift{};
  bool success{ false };
  int raw_ik{};
  int collision_free_ik{};
  int search_attempts{};
  int coarse_attempts{};
  int fine_attempts{};
  double selected_yaw{};
  double selected_pitch{};
  std::vector<double> selected_arm;
  double min_joint3_margin{ kInfinity };
  double min_joint5_margin{ kInfinity };
  double min_active_margin{ kInfinity };
  double min_environment_clearance{ kInfinity };
  double min_self_clearance{ kInfinity };
  double descent_distance{};
  double ascent_distance{};
  double final_object_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double max_arm_delta{};
  double max_torso_delta{};
  double max_expected_z_error{};
  double max_xy_error{};
  double max_orientation_error{};
  std::string failure_stage;
  int failure_waypoint{ -1 };
  std::string failure_label;
  std::string collision_pairs;
  double computation_time_ms{};
};

class Pilot
{
public:
  explicit Pilot(const rclcpp::Node::SharedPtr& node)
    : node_(node), scene_config_(loadSceneConfig(node_->get_parameter("scene_config").as_string()))
  {
    const YAML::Node config = YAML::LoadFile(node_->get_parameter("pilot_config").as_string());
    wall_clearance_ = config["positions"]["minimum_target_to_inner_wall_clearance_m"].as<double>();
    for (const auto& value : config["lift_candidates_m"])
      lift_candidates_.push_back(value.as<double>());
    locked_multistart_ = config["locked_baseline"]["arm_ik_multistart"].as<int>();
    yaw_min_deg_ = config["yaw_pitch_posture_selection"]["yaw_min_deg"].as<double>();
    yaw_max_deg_ = config["yaw_pitch_posture_selection"]["yaw_max_deg"].as<double>();
    pitch_min_deg_ = config["yaw_pitch_posture_selection"]["pitch_min_deg"].as<double>();
    pitch_max_deg_ = config["yaw_pitch_posture_selection"]["pitch_max_deg"].as<double>();
    coarse_step_deg_ = config["yaw_pitch_posture_selection"]["coarse_step_deg"].as<double>();
    fine_step_deg_ = config["yaw_pitch_posture_selection"]["refinement_step_deg"].as<double>();
    fine_half_width_deg_ =
      config["yaw_pitch_posture_selection"]["refinement_half_width_deg"].as<double>();
    posture_multistart_ =
      config["yaw_pitch_posture_selection"]["arm_ik_multistart_per_posture"].as<int>();
    sample_spacing_ = config["lift_motion"]["maximum_sample_spacing_m"].as<double>();
    safety_clearance_ =
      config["lift_motion"]["object_bottom_clearance_above_box_top_m"].as<double>();
    arm_tolerance_ = config["strict_posture_lock"]["arm_tolerance_rad"].as<double>();
    torso_tolerance_ = config["strict_posture_lock"]["torso_tolerance_rad"].as<double>();
    tcp_xy_tolerance_ = config["strict_posture_lock"]["tcp_xy_tolerance_m"].as<double>();
    tcp_z_tolerance_ =
      config["strict_posture_lock"]["tcp_z_translation_tolerance_m"].as<double>();
    tcp_orientation_tolerance_ =
      config["strict_posture_lock"]["tcp_orientation_tolerance_rad"].as<double>();

    position_catalog_csv_ = node_->get_parameter("position_catalog_csv").as_string();
    results_csv_ = node_->get_parameter("feasibility_results_csv").as_string();
    comparison_csv_ = node_->get_parameter("comparison_csv").as_string();
    samples_csv_ = node_->get_parameter("trajectory_samples_csv").as_string();
    result_yaml_ = node_->get_parameter("result_yaml").as_string();
    audit_md_ = node_->get_parameter("audit_md").as_string();

    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    model_ = loader_->getModel();
    if (!model_)
      throw std::runtime_error("Robot model or SRDF could not be loaded");
    whole_body_ = requiredGroup("whole_body");
    left_arm_ = requiredGroup("left_arm");
    active_group_ = requiredGroup("left_arm_with_torso");
    if (!left_arm_->getSolverInstance())
      throw std::runtime_error("left_arm IK solver is unavailable");
    const auto& yaw_bounds = model_->getVariableBounds("waist_yaw_joint");
    const auto& pitch_bounds = model_->getVariableBounds("waist_pitch_joint");
    urdf_yaw_min_ = yaw_bounds.min_position_;
    urdf_yaw_max_ = yaw_bounds.max_position_;
    urdf_pitch_min_ = pitch_bounds.min_position_;
    urdf_pitch_max_ = pitch_bounds.max_position_;
    validateImmutableInputs();
    positions_ = buildPositions();
    initializeOutputs();
  }

  bool run()
  {
    std::vector<ModeResult> all_results;
    std::vector<std::tuple<PositionSpec, double, ModeResult, ModeResult, std::string>> comparisons;
    int position_index = 0;
    for (const auto& position : positions_)
    {
      for (const double lift : lift_candidates_)
      {
        RCLCPP_INFO(node_->get_logger(), "PILOT position=%s lift=%.2f LOCKED_BASELINE",
                    position.id.c_str(), lift);
        ModeResult locked = runMode(position, lift, "LOCKED_BASELINE", position_index);
        appendResult(locked);
        all_results.push_back(locked);

        RCLCPP_INFO(node_->get_logger(), "PILOT position=%s lift=%.2f YAW_PITCH_POSTURE_SELECTION",
                    position.id.c_str(), lift);
        ModeResult posture = runMode(position, lift, "YAW_PITCH_POSTURE_SELECTION", position_index);
        appendResult(posture);
        all_results.push_back(posture);

        std::string label;
        if (!locked.success && posture.success)
          label = "YAW_PITCH_FEASIBILITY_RECOVERY";
        else if (locked.success && posture.success)
          label = "BOTH_FEASIBLE";
        else if (locked.success)
          label = "LOCKED_ONLY_FEASIBLE";
        else
          label = "BOTH_INFEASIBLE";
        appendComparison(position, lift, locked, posture, label);
        comparisons.emplace_back(position, lift, locked, posture, label);
        RCLCPP_INFO(node_->get_logger(),
                    "PILOT result position=%s lift=%.2f locked=%s yaw_pitch=%s comparison=%s",
                    position.id.c_str(), lift, locked.failure_label.c_str(),
                    posture.failure_label.c_str(), label.c_str());
      }
      ++position_index;
    }
    writeYaml(all_results, comparisons);
    writeAudit(all_results, comparisons);
    return all_results.size() == 36 && comparisons.size() == 18;
  }

private:
  const moveit::core::JointModelGroup* requiredGroup(const std::string& name) const
  {
    const auto* group = model_->getJointModelGroup(name);
    if (!group)
      throw std::runtime_error("Required SRDF group missing: " + name);
    return group;
  }

  void validateImmutableInputs() const
  {
    if (!scene_config_.top_open_reference || std::abs(scene_config_.box_width - 0.600) > 1e-12 ||
        std::abs(scene_config_.box_depth - 0.400) > 1e-12 ||
        std::abs(scene_config_.box_height - 0.150) > 1e-12 ||
        std::abs(scene_config_.target_size[0] - 0.050) > 1e-12 ||
        std::abs(scene_config_.target_size[1] - 0.050) > 1e-12 ||
        std::abs(scene_config_.target_size[2] - 0.050) > 1e-12)
      throw std::runtime_error("Immutable box or target geometry differs from the approved reference");
    const auto& lift_bounds = model_->getVariableBounds("lift_joint");
    if (!lift_bounds.position_bounded_ || std::abs(lift_bounds.min_position_) > 1e-12 ||
        std::abs(lift_bounds.max_position_ - 0.7) > 1e-12)
      throw std::runtime_error("Unexpected lift_joint bounds; expected [0.0, 0.7] m");
    if (std::max(yaw_min_deg_ * kPi / 180.0, urdf_yaw_min_) >
          std::min(yaw_max_deg_ * kPi / 180.0, urdf_yaw_max_) ||
        std::max(pitch_min_deg_ * kPi / 180.0, urdf_pitch_min_) >
          std::min(pitch_max_deg_ * kPi / 180.0, urdf_pitch_max_))
      throw std::runtime_error("Configured posture range has no overlap with URDF bounds");
  }

  std::vector<PositionSpec> buildPositions() const
  {
    const double center_x = scene_config_.box_center[0];
    const double center_y = scene_config_.box_center[1];
    const double z = scene_config_.target_position[2];
    const double x_min = center_x - scene_config_.box_depth / 2.0;
    const double x_max = center_x + scene_config_.box_depth / 2.0;
    const double y_min = center_y - scene_config_.box_width / 2.0;
    const double y_max = center_y + scene_config_.box_width / 2.0;
    const double half_x = scene_config_.target_size[0] / 2.0;
    const double half_y = scene_config_.target_size[1] / 2.0;
    const double near_x = x_min + half_x + wall_clearance_;
    const double far_x = x_max - half_x - wall_clearance_;
    const double left_y = y_max - half_y - wall_clearance_;
    const double right_y = y_min + half_y + wall_clearance_;
    std::vector<PositionSpec> positions{
      { "CENTER", center_x, center_y, z, 0.0 },
      { "NEAR_CENTER", near_x, center_y, z, 0.0 },
      { "FAR_CENTER", far_x, center_y, z, 0.0 },
      { "LEFT_CENTER", center_x, left_y, z, 0.0 },
      { "RIGHT_CENTER", center_x, right_y, z, 0.0 },
      { "NEAR_LEFT_CORNER", near_x, left_y, z, 0.0 },
      { "NEAR_RIGHT_CORNER", near_x, right_y, z, 0.0 },
      { "FAR_LEFT_CORNER", far_x, left_y, z, 0.0 },
      { "FAR_RIGHT_CORNER", far_x, right_y, z, 0.0 },
    };
    for (auto& position : positions)
    {
      const double x_clearance = std::min(position.x - half_x - x_min, x_max - position.x - half_x);
      const double y_clearance = std::min(position.y - half_y - y_min, y_max - position.y - half_y);
      position.inner_wall_clearance = std::min(x_clearance, y_clearance);
      if (position.inner_wall_clearance < -1e-12)
        throw std::runtime_error("Computed target overlaps an inner wall: " + position.id);
    }
    return positions;
  }

  moveit_msgs::msg::CollisionObject boxObject(const std::string& id,
                                               const std::vector<double>& dimensions,
                                               const std::vector<double>& position) const
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = scene_config_.frame_id;
    object.id = id;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.assign(dimensions.begin(), dimensions.end());
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
  }

  void resetScene(const PositionSpec& position)
  {
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    const auto& c = scene_config_.box_center;
    const double w = scene_config_.box_width;
    const double d = scene_config_.box_depth;
    const double h = scene_config_.box_height;
    const double t = scene_config_.wall_thickness;
    const double floor = scene_config_.floor_thickness;
    const std::vector<moveit_msgs::msg::CollisionObject> objects{
      boxObject("box_bottom", { d + 2.0 * t, w + 2.0 * t, floor },
                { c[0], c[1], c[2] - h / 2.0 - floor / 2.0 }),
      boxObject("box_left_wall", { d, t, h }, { c[0], c[1] + w / 2.0 + t / 2.0, c[2] }),
      boxObject("box_right_wall", { d, t, h }, { c[0], c[1] - w / 2.0 - t / 2.0, c[2] }),
      boxObject("box_back_wall", { t, w + 2.0 * t, h }, { c[0] + d / 2.0 + t / 2.0, c[1], c[2] }),
      boxObject("box_front_wall", { t, w + 2.0 * t, h }, { c[0] - d / 2.0 - t / 2.0, c[1], c[2] }),
      boxObject(target_id_, scene_config_.target_size, { position.x, position.y, position.z }),
    };
    for (const auto& object : objects)
      if (!scene_->processCollisionObjectMsg(object))
        throw std::runtime_error("PlanningScene rejected object " + object.id);
    attached_target_in_tcp_ = Eigen::Isometry3d::Identity();
  }

  geometry_msgs::msg::Pose graspPose(const PositionSpec& position) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = position.x;
    pose.position.y = position.y;
    const double object_bottom = position.z - scene_config_.target_size[2] / 2.0;
    pose.position.z = object_bottom + scene_config_.grasp_height + scene_config_.tcp_to_grasp_center;
    tf2::Quaternion orientation;
    orientation.setRPY(scene_config_.eef_rpy[0], scene_config_.eef_rpy[1], scene_config_.eef_rpy[2]);
    orientation.normalize();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
  }

  Eigen::Isometry3d poseTransform(const geometry_msgs::msg::Pose& pose) const
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    transform.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                             pose.orientation.y, pose.orientation.z).toRotationMatrix();
    return transform;
  }

  moveit::core::RobotState initialState(double lift, double yaw, double pitch) const
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setVariablePosition("lift_joint", lift);
    state.setVariablePosition("waist_yaw_joint", yaw);
    state.setVariablePosition("waist_pitch_joint", pitch);
    state.setVariablePosition("openarm_left_finger_joint1", scene_config_.left_finger);
    state.setVariablePosition("openarm_right_finger_joint1", scene_config_.right_finger);
    state.update();
    return state;
  }

  double variableMargin(const moveit::core::RobotState& state, const std::string& name) const
  {
    const auto& bounds = model_->getVariableBounds(name);
    if (!bounds.position_bounded_)
      return kInfinity;
    const double value = state.getVariablePosition(name);
    return std::min(value - bounds.min_position_, bounds.max_position_ - value);
  }

  CollisionStatus collisionStatus(moveit::core::RobotState& state) const
  {
    state.update();
    CollisionStatus status;
    status.joint_limit_valid = state.satisfiesBounds(whole_body_);
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 50;
    collision_detection::CollisionResult self_result;
    scene_->checkSelfCollision(request, self_result, state);
    status.self_collision = self_result.collision;
    collision_detection::CollisionResult full_result;
    scene_->checkCollision(request, full_result, state);
    for (const auto& entry : self_result.contacts)
    {
      status.pairs.insert(entry.first);
      status.self_pairs.insert(entry.first);
    }
    for (const auto& entry : full_result.contacts)
    {
      bool world = false;
      for (const auto& contact : entry.second)
        if (contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT ||
            contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
          world = true;
      if (world)
      {
        status.environment_collision = true;
        status.pairs.insert(entry.first);
        status.environment_pairs.insert(entry.first);
      }
    }
    return status;
  }

  double attachedObjectMinimumZ(const moveit::core::RobotState& state) const
  {
    const Eigen::Isometry3d object_world = state.getGlobalLinkTransform(tcp_link_) *
                                           attached_target_in_tcp_;
    const Eigen::Vector3d half(scene_config_.target_size[0] / 2.0,
                               scene_config_.target_size[1] / 2.0,
                               scene_config_.target_size[2] / 2.0);
    double minimum = kInfinity;
    for (const double x : { -half.x(), half.x() })
      for (const double y : { -half.y(), half.y() })
        for (const double z : { -half.z(), half.z() })
          minimum = std::min(minimum, (object_world * Eigen::Vector3d(x, y, z)).z());
    return minimum;
  }

  Metrics evaluate(moveit::core::RobotState& state, const Eigen::Isometry3d& expected_tcp,
                   bool attached) const
  {
    const CollisionStatus status = collisionStatus(state);
    const auto& acm = scene_->getAllowedCollisionMatrix();
    const Eigen::Isometry3d& tcp = state.getGlobalLinkTransform(tcp_link_);
    Metrics metrics;
    metrics.bounds_valid = status.joint_limit_valid;
    metrics.self_collision = status.self_collision;
    metrics.environment_collision = status.environment_collision;
    metrics.collision_pairs = pairString(status.pairs);
    metrics.environment_clearance = scene_->getCollisionEnv()->distanceRobot(state, acm);
    metrics.self_clearance = scene_->getCollisionEnv()->distanceSelf(state, acm);
    metrics.active_margin = kInfinity;
    for (const auto& name : active_group_->getVariableNames())
      metrics.active_margin = std::min(metrics.active_margin, variableMargin(state, name));
    metrics.joint3_margin = variableMargin(state, "openarm_left_joint3");
    metrics.joint5_margin = variableMargin(state, "openarm_left_joint5");
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
    if (attached)
    {
      metrics.attached_min_z = attachedObjectMinimumZ(state);
      const double box_top = scene_config_.box_center[2] + scene_config_.box_height / 2.0;
      metrics.object_clearance = metrics.attached_min_z - box_top;
    }
    return metrics;
  }

  bool stateValid(const Metrics& metrics) const
  {
    return metrics.bounds_valid && !metrics.self_collision && !metrics.environment_collision;
  }

  moveit::core::RobotState randomSeed(double lift, double yaw, double pitch,
                                      int position_index, int posture_index, int seed_id) const
  {
    moveit::core::RobotState state = initialState(lift, yaw, pitch);
    if (seed_id > 0)
    {
      const std::uint64_t key = 202608150000ULL + static_cast<std::uint64_t>(position_index) * 1000003ULL +
        static_cast<std::uint64_t>(posture_index + 10000) * 1009ULL +
        static_cast<std::uint64_t>(seed_id);
      std::mt19937_64 rng(key);
      for (const auto& name : left_arm_->getVariableNames())
      {
        const auto& bounds = model_->getVariableBounds(name);
        std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
        state.setVariablePosition(name, distribution(rng));
      }
      state.update();
    }
    return state;
  }

  bool better(const GraspCandidate& candidate, const GraspCandidate& incumbent) const
  {
    if (candidate.metrics.active_margin > incumbent.metrics.active_margin + 1e-12)
      return true;
    if (incumbent.metrics.active_margin > candidate.metrics.active_margin + 1e-12)
      return false;
    const double candidate_clearance =
      std::min(candidate.metrics.environment_clearance, candidate.metrics.self_clearance);
    const double incumbent_clearance =
      std::min(incumbent.metrics.environment_clearance, incumbent.metrics.self_clearance);
    if (candidate_clearance > incumbent_clearance + 1e-12)
      return true;
    if (incumbent_clearance > candidate_clearance + 1e-12)
      return false;
    const double candidate_posture = std::abs(candidate.yaw) + std::abs(candidate.pitch);
    const double incumbent_posture = std::abs(incumbent.yaw) + std::abs(incumbent.pitch);
    if (candidate_posture < incumbent_posture - 1e-12)
      return true;
    if (incumbent_posture < candidate_posture - 1e-12)
      return false;
    return candidate.seed_id < incumbent.seed_id;
  }

  void searchPosture(const PositionSpec& position, double lift, double yaw_deg, double pitch_deg,
                     int multistart, int position_index, int posture_index, bool fine,
                     SearchResult& result)
  {
    double yaw = yaw_deg * kPi / 180.0;
    double pitch = pitch_deg * kPi / 180.0;
    const auto& yaw_bounds = model_->getVariableBounds("waist_yaw_joint");
    const auto& pitch_bounds = model_->getVariableBounds("waist_pitch_joint");
    if (yaw_bounds.position_bounded_)
      yaw = std::clamp(yaw, yaw_bounds.min_position_, yaw_bounds.max_position_);
    if (pitch_bounds.position_bounded_)
      pitch = std::clamp(pitch, pitch_bounds.min_position_, pitch_bounds.max_position_);
    const geometry_msgs::msg::Pose pose = graspPose(position);
    const Eigen::Isometry3d expected = poseTransform(pose);
    for (int seed_id = 0; seed_id < multistart; ++seed_id)
    {
      ++result.attempts;
      if (fine)
        ++result.fine_attempts;
      else
        ++result.coarse_attempts;
      moveit::core::RobotState state =
        randomSeed(lift, yaw, pitch, position_index, posture_index, seed_id);
      if (!state.setFromIK(left_arm_, pose, tcp_link_, scene_config_.ik_timeout))
        continue;
      ++result.raw_ik;
      Metrics metrics = evaluate(state, expected, false);
      if (!stateValid(metrics))
      {
        if (!result.collision_example_found &&
            (metrics.self_collision || metrics.environment_collision))
        {
          result.collision_example_found = true;
          result.collision_example = metrics;
        }
        continue;
      }
      ++result.collision_free_ik;
      GraspCandidate candidate(model_);
      candidate.state = state;
      candidate.metrics = metrics;
      candidate.yaw = yaw;
      candidate.pitch = pitch;
      candidate.seed_id = seed_id;
      if (!result.found || better(candidate, result.selected))
      {
        result.found = true;
        result.selected = candidate;
      }
    }
  }

  SearchResult search(const PositionSpec& position, double lift, const std::string& mode,
                      int position_index)
  {
    resetScene(position);
    SearchResult result(model_);
    if (mode == "LOCKED_BASELINE")
    {
      searchPosture(position, lift, 0.0, 0.0, locked_multistart_, position_index, 0, false, result);
      return result;
    }
    int posture_index = 0;
    for (double yaw = yaw_min_deg_; yaw <= yaw_max_deg_ + 1e-9; yaw += coarse_step_deg_)
      for (double pitch = pitch_min_deg_; pitch <= pitch_max_deg_ + 1e-9; pitch += coarse_step_deg_)
        searchPosture(position, lift, yaw, pitch, posture_multistart_, position_index,
                      posture_index++, false, result);
    if (!result.found)
      return result;
    const double best_yaw_deg = result.selected.yaw * 180.0 / kPi;
    const double best_pitch_deg = result.selected.pitch * 180.0 / kPi;
    const double fine_yaw_min = std::max(yaw_min_deg_, best_yaw_deg - fine_half_width_deg_);
    const double fine_yaw_max = std::min(yaw_max_deg_, best_yaw_deg + fine_half_width_deg_);
    const double fine_pitch_min = std::max(pitch_min_deg_, best_pitch_deg - fine_half_width_deg_);
    const double fine_pitch_max = std::min(pitch_max_deg_, best_pitch_deg + fine_half_width_deg_);
    for (double yaw = fine_yaw_min; yaw <= fine_yaw_max + 1e-9; yaw += fine_step_deg_)
      for (double pitch = fine_pitch_min; pitch <= fine_pitch_max + 1e-9; pitch += fine_step_deg_)
        searchPosture(position, lift, yaw, pitch, posture_multistart_, position_index,
                      posture_index++, true, result);
    return result;
  }

  double armDistance(const moveit::core::RobotState& first,
                     const moveit::core::RobotState& second) const
  {
    double maximum = 0.0;
    for (const auto& name : left_arm_->getVariableNames())
      maximum = std::max(maximum, std::abs(second.getVariablePosition(name) -
                                           first.getVariablePosition(name)));
    return maximum;
  }

  double torsoDistance(const moveit::core::RobotState& first,
                       const moveit::core::RobotState& second) const
  {
    return std::max(std::abs(second.getVariablePosition("waist_yaw_joint") -
                             first.getVariablePosition("waist_yaw_joint")),
                    std::abs(second.getVariablePosition("waist_pitch_joint") -
                             first.getVariablePosition("waist_pitch_joint")));
  }

  void updateResult(ModeResult& result, const Metrics& metrics,
                    const moveit::core::RobotState& locked,
                    const moveit::core::RobotState& state)
  {
    result.min_joint3_margin = std::min(result.min_joint3_margin, metrics.joint3_margin);
    result.min_joint5_margin = std::min(result.min_joint5_margin, metrics.joint5_margin);
    result.min_active_margin = std::min(result.min_active_margin, metrics.active_margin);
    result.min_environment_clearance =
      std::min(result.min_environment_clearance, metrics.environment_clearance);
    result.min_self_clearance = std::min(result.min_self_clearance, metrics.self_clearance);
    result.max_arm_delta = std::max(result.max_arm_delta, armDistance(locked, state));
    result.max_torso_delta = std::max(result.max_torso_delta, torsoDistance(locked, state));
    result.max_expected_z_error =
      std::max(result.max_expected_z_error, std::abs(metrics.expected_z_error));
    result.max_xy_error = std::max(result.max_xy_error, metrics.xy_error);
    result.max_orientation_error =
      std::max(result.max_orientation_error, metrics.orientation_error);
    if (std::isfinite(metrics.object_clearance))
      result.final_object_clearance = metrics.object_clearance;
  }

  void writeSample(const ModeResult& result, const std::string& stage, int waypoint,
                   double stage_start_lift, const moveit::core::RobotState& state,
                   const moveit::core::RobotState& previous, const Metrics& metrics,
                   const std::string& label) const
  {
    std::ofstream out(samples_csv_, std::ios::app);
    out << std::setprecision(15) << result.position.id << ',' << result.position.x << ','
        << result.position.y << ',' << result.position.z << ',' << result.lift << ',' << result.mode << ','
        << stage << ',' << waypoint << ',' << state.getVariablePosition("lift_joint") << ','
        << state.getVariablePosition("lift_joint") - stage_start_lift << ','
        << state.getVariablePosition("waist_yaw_joint") << ','
        << state.getVariablePosition("waist_pitch_joint");
    for (const auto& name : left_arm_->getVariableNames())
      out << ',' << state.getVariablePosition(name);
    for (const auto& name : left_arm_->getVariableNames())
      out << ',' << state.getVariablePosition(name) - previous.getVariablePosition(name);
    out << ',' << metrics.tcp_x << ',' << metrics.tcp_y << ',' << metrics.tcp_z << ','
        << metrics.tcp_qx << ',' << metrics.tcp_qy << ',' << metrics.tcp_qz << ',' << metrics.tcp_qw << ','
        << metrics.expected_z_error << ',' << metrics.xy_error << ',' << metrics.orientation_error << ','
        << metrics.joint3_margin << ',' << metrics.joint5_margin << ',' << metrics.active_margin << ','
        << metrics.environment_clearance << ',' << metrics.self_clearance << ',' << metrics.attached_min_z << ','
        << metrics.object_clearance << ','
        << ((!metrics.bounds_valid || metrics.self_collision || metrics.environment_collision) ? 1 : 0) << ','
        << csvEscape(metrics.collision_pairs) << ',' << csvEscape(label) << '\n';
  }

  bool strictValid(const moveit::core::RobotState& locked, const moveit::core::RobotState& state,
                   const Metrics& metrics) const
  {
    return armDistance(locked, state) <= arm_tolerance_ &&
           torsoDistance(locked, state) <= torso_tolerance_ &&
           metrics.xy_error <= tcp_xy_tolerance_ &&
           std::abs(metrics.expected_z_error) <= tcp_z_tolerance_ &&
           metrics.orientation_error <= tcp_orientation_tolerance_;
  }

  bool liftStage(const std::string& stage, const std::string& limit_label,
                 const std::string& collision_label, const moveit::core::RobotState& locked,
                 const moveit::core::RobotState& initial, double target_lift, bool attached,
                 ModeResult& result, moveit::core::RobotState& final)
  {
    const auto& bounds = model_->getVariableBounds("lift_joint");
    if (target_lift < bounds.min_position_ - 1e-12 || target_lift > bounds.max_position_ + 1e-12)
    {
      result.failure_stage = stage;
      result.failure_waypoint = 0;
      result.failure_label = limit_label;
      return false;
    }
    const double start_lift = initial.getVariablePosition("lift_joint");
    const int intervals = std::max(1, static_cast<int>(
      std::ceil(std::abs(target_lift - start_lift) / sample_spacing_)));
    const Eigen::Isometry3d reference_tcp = initial.getGlobalLinkTransform(tcp_link_);
    moveit::core::RobotState previous = initial;
    for (int waypoint = 0; waypoint <= intervals; ++waypoint)
    {
      const double ratio = static_cast<double>(waypoint) / intervals;
      moveit::core::RobotState state = initial;
      const double lift = start_lift + ratio * (target_lift - start_lift);
      state.setVariablePosition("lift_joint", lift);
      state.update();
      Eigen::Isometry3d expected = reference_tcp;
      expected.translation().z() = reference_tcp.translation().z() - (lift - start_lift);
      Metrics metrics = evaluate(state, expected, attached);
      updateResult(result, metrics, locked, state);
      std::string label;
      if (!metrics.bounds_valid)
        label = limit_label;
      else if (metrics.self_collision || metrics.environment_collision)
        label = collision_label;
      else if (!strictValid(locked, state, metrics))
        label = "POSTURE_LOCK_CONNECTIVITY_FAILURE";
      writeSample(result, stage, waypoint, start_lift, state,
                  waypoint == 0 ? state : previous, metrics, label);
      if (!label.empty())
      {
        result.failure_stage = stage;
        result.failure_waypoint = waypoint;
        result.failure_label = label;
        result.collision_pairs = metrics.collision_pairs;
        return false;
      }
      previous = state;
      final = state;
    }
    return true;
  }

  void allowFingerTargetContact(bool allowed)
  {
    auto& acm = scene_->getAllowedCollisionMatrixNonConst();
    for (const auto& finger : finger_links_)
      acm.setEntry(finger, target_id_, allowed);
  }

  void attachTarget(const PositionSpec& position, const moveit::core::RobotState& state)
  {
    scene_->setCurrentState(state);
    allowFingerTargetContact(false);
    Eigen::Isometry3d target_world = Eigen::Isometry3d::Identity();
    target_world.translation() = Eigen::Vector3d(position.x, position.y, position.z);
    attached_target_in_tcp_ = state.getGlobalLinkTransform(tcp_link_).inverse() * target_world;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp_link_;
    attached.touch_links = { finger_links_[0], finger_links_[1] };
    attached.object.header.frame_id = tcp_link_;
    attached.object.id = target_id_;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.assign(scene_config_.target_size.begin(), scene_config_.target_size.end());
    attached.object.primitives.push_back(primitive);
    geometry_msgs::msg::Pose pose;
    pose.position.x = attached_target_in_tcp_.translation().x();
    pose.position.y = attached_target_in_tcp_.translation().y();
    pose.position.z = attached_target_in_tcp_.translation().z();
    const Eigen::Quaterniond quaternion(attached_target_in_tcp_.rotation());
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    attached.object.primitive_poses.push_back(pose);
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!scene_->processAttachedCollisionObjectMsg(attached))
      throw std::runtime_error("PlanningScene rejected target attachment");
  }

  bool graspStage(const moveit::core::RobotState& locked,
                  const moveit::core::RobotState& initial, ModeResult& result,
                  moveit::core::RobotState& grasped)
  {
    allowFingerTargetContact(true);
    const double finger_start = initial.getVariablePosition("openarm_left_finger_joint1");
    const Eigen::Isometry3d expected = initial.getGlobalLinkTransform(tcp_link_);
    moveit::core::RobotState previous = initial;
    for (int waypoint = 0; waypoint <= 10; ++waypoint)
    {
      moveit::core::RobotState state = initial;
      state.setVariablePosition("openarm_left_finger_joint1", finger_start +
        static_cast<double>(waypoint) / 10.0 * (scene_config_.q_contact - finger_start));
      state.update();
      Metrics metrics = evaluate(state, expected, false);
      updateResult(result, metrics, locked, state);
      std::string label;
      if (!stateValid(metrics))
        label = "GRASP_GEOMETRY_FAILURE";
      else if (!strictValid(locked, state, metrics))
        label = "POSTURE_LOCK_CONNECTIVITY_FAILURE";
      writeSample(result, "GRASP", waypoint, initial.getVariablePosition("lift_joint"), state,
                  waypoint == 0 ? state : previous, metrics, label);
      if (!label.empty())
      {
        result.failure_stage = "GRASP";
        result.failure_waypoint = waypoint;
        result.failure_label = label;
        result.collision_pairs = metrics.collision_pairs;
        return false;
      }
      previous = state;
      grasped = state;
    }
    attachTarget(result.position, grasped);
    return true;
  }

  void validateSelectedPath(const SearchResult& search_result, ModeResult& result)
  {
    resetScene(result.position);
    moveit::core::RobotState locked = search_result.selected.state;
    locked.setVariablePosition("lift_joint", result.lift);
    locked.update();
    locked.copyJointGroupPositions(left_arm_, result.selected_arm);
    result.selected_yaw = locked.getVariablePosition("waist_yaw_joint");
    result.selected_pitch = locked.getVariablePosition("waist_pitch_joint");

    const double box_top = scene_config_.box_center[2] + scene_config_.box_height / 2.0;
    const double object_bottom = result.position.z - scene_config_.target_size[2] / 2.0;
    const double required_rise = box_top + safety_clearance_ - object_bottom;
    const double upper_lift = result.lift - required_rise;
    result.descent_distance = required_rise;
    result.ascent_distance = required_rise;
    const auto& lift_bounds = model_->getVariableBounds("lift_joint");
    if (upper_lift < lift_bounds.min_position_ - 1e-12 ||
        upper_lift > lift_bounds.max_position_ + 1e-12)
    {
      result.failure_stage = "LIFT_VERTICAL_DESCENT";
      result.failure_label = "LIFT_DESCENT_LIMIT_FAILURE";
      result.failure_waypoint = 0;
      return;
    }
    moveit::core::RobotState start = locked;
    start.setVariablePosition("lift_joint", upper_lift);
    start.update();
    moveit::core::RobotState at_grasp = start;
    if (!liftStage("LIFT_VERTICAL_DESCENT", "LIFT_DESCENT_LIMIT_FAILURE",
                   "LIFT_DESCENT_COLLISION_FAILURE", locked, start, result.lift, false,
                   result, at_grasp))
      return;
    moveit::core::RobotState grasped = at_grasp;
    if (!graspStage(locked, at_grasp, result, grasped))
      return;
    moveit::core::RobotState cleared = grasped;
    if (!liftStage("LIFT_ACTUATED_CLEARANCE", "LIFT_ASCENT_LIMIT_FAILURE",
                   "LIFT_ASCENT_COLLISION_FAILURE", locked, grasped, upper_lift, true,
                   result, cleared))
      return;
    result.final_object_clearance = attachedObjectMinimumZ(cleared) - box_top;
    if (result.final_object_clearance + 1e-9 < safety_clearance_)
    {
      result.failure_stage = "LIFT_ACTUATED_CLEARANCE";
      result.failure_label = "ATTACHED_OBJECT_CLEARANCE_FAILURE";
      return;
    }
    result.success = true;
    result.failure_label = "LIFT_ACTUATED_EXTRACTION_SUCCESS";
    result.failure_stage.clear();
    result.failure_waypoint = -1;
  }

  ModeResult runMode(const PositionSpec& position, double lift, const std::string& mode,
                     int position_index)
  {
    const auto start = std::chrono::steady_clock::now();
    ModeResult result;
    result.position = position;
    result.mode = mode;
    result.lift = lift;
    const SearchResult search_result = search(position, lift, mode, position_index);
    result.raw_ik = search_result.raw_ik;
    result.collision_free_ik = search_result.collision_free_ik;
    result.search_attempts = search_result.attempts;
    result.coarse_attempts = search_result.coarse_attempts;
    result.fine_attempts = search_result.fine_attempts;
    if (!search_result.found)
    {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      result.selected_yaw = nan;
      result.selected_pitch = nan;
      result.min_joint3_margin = nan;
      result.min_joint5_margin = nan;
      result.min_active_margin = nan;
      result.min_environment_clearance = nan;
      result.min_self_clearance = nan;
      if (search_result.collision_example_found)
        result.collision_pairs = search_result.collision_example.collision_pairs;
      result.failure_stage = "GRASP_CONFIGURATION_SEARCH";
      result.failure_label = search_result.raw_ik == 0 ?
        "GRASP_CONFIGURATION_IK_FAILURE" : "GRASP_CONFIGURATION_COLLISION_FAILURE";
    }
    else
    {
      validateSelectedPath(search_result, result);
    }
    result.computation_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    return result;
  }

  void initializeOutputs() const
  {
    {
      std::ofstream out(position_catalog_csv_, std::ios::trunc);
      out << "position_id,world_x,world_y,world_z,target_half_x,target_half_y,"
             "minimum_inner_wall_clearance,overlaps_inner_wall\n";
      for (const auto& position : positions_)
        out << std::setprecision(15) << position.id << ',' << position.x << ',' << position.y << ','
            << position.z << ',' << scene_config_.target_size[0] / 2.0 << ','
            << scene_config_.target_size[1] / 2.0 << ',' << position.inner_wall_clearance << ",0\n";
    }
    {
      std::ofstream out(results_csv_, std::ios::trunc);
      out << "position_id,world_x,world_y,world_z,lift,mode,success,search_attempts,raw_ik_count,"
             "collision_free_ik_count,coarse_attempts,fine_attempts,selected_yaw_rad,selected_pitch_rad";
      for (const auto& name : left_arm_->getVariableNames())
        out << ',' << name;
      out << ",joint3_min_margin,joint5_min_margin,minimum_active_joint_margin,"
             "minimum_environment_clearance,minimum_self_clearance,lift_descent_distance,"
             "lift_ascent_distance,final_object_box_top_clearance,max_arm_delta,max_torso_delta,"
             "max_expected_tcp_z_error,max_tcp_xy_error,max_tcp_orientation_error,failure_stage,"
             "failure_waypoint,failure_label,collision_pairs,computation_time_ms\n";
    }
    {
      std::ofstream out(comparison_csv_, std::ios::trunc);
      out << "position_id,world_x,world_y,world_z,lift,locked_success,yaw_pitch_success,"
             "locked_label,yaw_pitch_label,selected_yaw_rad,selected_pitch_rad,"
             "locked_min_active_margin,yaw_pitch_min_active_margin,locked_environment_clearance,"
             "yaw_pitch_environment_clearance,comparison_label\n";
    }
    {
      std::ofstream out(samples_csv_, std::ios::trunc);
      out << "position_id,world_x,world_y,world_z,lift_candidate,mode,stage,waypoint_index,"
             "lift_joint,lift_displacement,yaw,pitch";
      for (const auto& name : left_arm_->getVariableNames())
        out << ',' << name;
      for (const auto& name : left_arm_->getVariableNames())
        out << ",delta_" << name;
      out << ",tcp_x,tcp_y,tcp_z,tcp_qx,tcp_qy,tcp_qz,tcp_qw,expected_tcp_z_error,"
             "tcp_xy_error,tcp_orientation_error,joint3_margin,joint5_margin,active_joint_min_margin,"
             "environment_clearance,self_clearance,attached_object_min_z,object_box_top_clearance,"
             "collision,collision_pairs,failure_label\n";
    }
  }

  void appendResult(const ModeResult& result) const
  {
    std::ofstream out(results_csv_, std::ios::app);
    out << std::setprecision(15) << result.position.id << ',' << result.position.x << ','
        << result.position.y << ',' << result.position.z << ',' << result.lift << ',' << result.mode << ','
        << (result.success ? 1 : 0) << ',' << result.search_attempts << ',' << result.raw_ik << ','
        << result.collision_free_ik << ',' << result.coarse_attempts << ',' << result.fine_attempts << ','
        << result.selected_yaw << ',' << result.selected_pitch;
    for (const double value : result.selected_arm)
      out << ',' << value;
    for (std::size_t index = result.selected_arm.size(); index < left_arm_->getVariableNames().size(); ++index)
      out << ",nan";
    out << ',' << result.min_joint3_margin << ',' << result.min_joint5_margin << ','
        << result.min_active_margin << ',' << result.min_environment_clearance << ','
        << result.min_self_clearance << ',' << result.descent_distance << ',' << result.ascent_distance << ','
        << result.final_object_clearance << ',' << result.max_arm_delta << ',' << result.max_torso_delta << ','
        << result.max_expected_z_error << ',' << result.max_xy_error << ',' << result.max_orientation_error << ','
        << csvEscape(result.failure_stage) << ',' << result.failure_waypoint << ','
        << csvEscape(result.failure_label) << ',' << csvEscape(result.collision_pairs) << ','
        << result.computation_time_ms << '\n';
  }

  void appendComparison(const PositionSpec& position, double lift, const ModeResult& locked,
                        const ModeResult& posture, const std::string& label) const
  {
    std::ofstream out(comparison_csv_, std::ios::app);
    out << std::setprecision(15) << position.id << ',' << position.x << ',' << position.y << ','
        << position.z << ',' << lift << ',' << (locked.success ? 1 : 0) << ','
        << (posture.success ? 1 : 0) << ',' << locked.failure_label << ',' << posture.failure_label << ','
        << posture.selected_yaw << ',' << posture.selected_pitch << ',' << locked.min_active_margin << ','
        << posture.min_active_margin << ',' << locked.min_environment_clearance << ','
        << posture.min_environment_clearance << ',' << label << '\n';
  }

  void writeYaml(
    const std::vector<ModeResult>& results,
    const std::vector<std::tuple<PositionSpec, double, ModeResult, ModeResult, std::string>>& comparisons) const
  {
    std::ofstream out(result_yaml_, std::ios::trunc);
    out << "protocol: BOX_TARGET_YAW_PITCH_FEASIBILITY_PILOT_V1\nplanning_only: true\n"
           "move_group_started: false\nompl_started: false\ntrajectory_execution_performed: false\n"
           "rviz_started: false\npositions:\n";
    for (const auto& position : positions_)
      out << "  - id: " << position.id << "\n    world_xyz: [" << position.x << ", " << position.y
          << ", " << position.z << "]\n    minimum_inner_wall_clearance_m: "
          << position.inner_wall_clearance << '\n';
    out << "results:\n";
    for (const auto& result : results)
      out << "  - position: " << result.position.id << "\n    lift: " << result.lift
          << "\n    mode: " << result.mode << "\n    success: " << (result.success ? "true" : "false")
          << "\n    label: " << result.failure_label << "\n    yaw_rad: " << result.selected_yaw
          << "\n    pitch_rad: " << result.selected_pitch << "\n";
    out << "comparisons:\n";
    for (const auto& comparison : comparisons)
      out << "  - position: " << std::get<0>(comparison).id << "\n    lift: " << std::get<1>(comparison)
          << "\n    label: " << std::get<4>(comparison) << "\n";
  }

  void writeAudit(
    const std::vector<ModeResult>& results,
    const std::vector<std::tuple<PositionSpec, double, ModeResult, ModeResult, std::string>>& comparisons) const
  {
    int locked_success = 0;
    int posture_success = 0;
    int recovery = 0;
    for (const auto& result : results)
    {
      if (result.mode == "LOCKED_BASELINE" && result.success)
        ++locked_success;
      if (result.mode == "YAW_PITCH_POSTURE_SELECTION" && result.success)
        ++posture_success;
    }
    for (const auto& comparison : comparisons)
      if (std::get<4>(comparison) == "YAW_PITCH_FEASIBILITY_RECOVERY")
        ++recovery;
    std::ofstream out(audit_md_, std::ios::trunc);
    out << "# Box target Yaw/Pitch feasibility pilot v1\n\nGenerated: " << timestampNow()
        << "\n\n- Scope: 9 target positions x 2 Lift values x 2 modes = 36 results.\n"
           "- LOCKED success: " << locked_success << "/18.\n- Yaw/Pitch posture-selection success: "
        << posture_success << "/18.\n- Qualified YAW_PITCH_FEASIBILITY_RECOVERY cases: " << recovery
        << ".\n- Target positions were calculated from the 0.400 x 0.600 m inner bounds, the 0.025 m "
           "target half-size, and a 0.025 m target-to-wall clearance.\n"
           "- Arm and selected Yaw/Pitch were locked after grasp selection; only lift_joint generated vertical "
           "descent/ascent.\n- No move_group, OMPL, controller, ros2_control, hardware, trajectory execution, or RViz "
           "was started. Force closure is not claimed.\n- The run stops after the attached object reaches 20 mm "
           "clearance above the box top.\n- URDF Yaw bounds [rad]: [" << urdf_yaw_min_ << ", " << urdf_yaw_max_
        << "]; Pitch bounds [rad]: [" << urdf_pitch_min_ << ", " << urdf_pitch_max_
        << "]. Requested endpoints are clamped to these exact URDF bounds.\n";
  }

  rclcpp::Node::SharedPtr node_;
  SceneConfig scene_config_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* whole_body_{ nullptr };
  const moveit::core::JointModelGroup* left_arm_{ nullptr };
  const moveit::core::JointModelGroup* active_group_{ nullptr };
  planning_scene::PlanningScenePtr scene_;
  std::vector<PositionSpec> positions_;
  std::vector<double> lift_candidates_;
  double wall_clearance_{};
  int locked_multistart_{};
  int posture_multistart_{};
  double yaw_min_deg_{};
  double yaw_max_deg_{};
  double pitch_min_deg_{};
  double pitch_max_deg_{};
  double coarse_step_deg_{};
  double fine_step_deg_{};
  double fine_half_width_deg_{};
  double sample_spacing_{};
  double safety_clearance_{};
  double arm_tolerance_{};
  double torso_tolerance_{};
  double tcp_xy_tolerance_{};
  double tcp_z_tolerance_{};
  double tcp_orientation_tolerance_{};
  double urdf_yaw_min_{};
  double urdf_yaw_max_{};
  double urdf_pitch_min_{};
  double urdf_pitch_max_{};
  Eigen::Isometry3d attached_target_in_tcp_{ Eigen::Isometry3d::Identity() };
  const std::string tcp_link_{ "openarm_left_hand_tcp" };
  const std::string target_id_{ "target_object" };
  const std::array<std::string, 2> finger_links_{ "openarm_left_left_finger",
                                                 "openarm_left_right_finger" };
  std::string position_catalog_csv_;
  std::string results_csv_;
  std::string comparison_csv_;
  std::string samples_csv_;
  std::string result_yaml_;
  std::string audit_md_;
};
}  // namespace box_target_yaw_pitch_pilot_v1

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("box_target_yaw_pitch_feasibility_pilot_v1", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  int exit_code = 1;
  std::unique_ptr<box_target_yaw_pitch_pilot_v1::Pilot> pilot;
  try
  {
    pilot = std::make_unique<box_target_yaw_pitch_pilot_v1::Pilot>(node);
    exit_code = pilot->run() ? 0 : 2;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Box target pilot failed: %s", error.what());
  }
  executor.cancel();
  if (spin_thread.joinable())
    spin_thread.join();
  pilot.reset();
  node.reset();
  rclcpp::shutdown();
  return exit_code;
}
