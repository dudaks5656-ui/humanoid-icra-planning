#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRequestedPitchUpper = 45.0 * kPi / 180.0;
constexpr double kObjectWidth = 0.050;
constexpr double kMinimumVerticalOverlap = 0.015;
constexpr char kTcpLink[] = "openarm_left_hand_tcp";
constexpr char kFingerJoint[] = "openarm_left_finger_joint1";
constexpr char kMimicJoint[] = "openarm_left_finger_joint2";
constexpr char kLeftFinger[] = "openarm_left_left_finger";
constexpr char kRightFinger[] = "openarm_left_right_finger";

std::string csvEscape(const std::string& value)
{
  std::string result = "\"";
  for (const char character : value)
    result += character == '"' ? "\"\"" : std::string(1, character);
  return result + '"';
}

std::string boolString(bool value)
{
  return value ? "true" : "false";
}

std::string pairString(const std::set<std::pair<std::string, std::string>>& pairs)
{
  std::ostringstream output;
  bool first = true;
  for (const auto& pair : pairs)
  {
    if (!first)
      output << ';';
    output << pair.first << "<->" << pair.second;
    first = false;
  }
  return output.str();
}

struct Bounds
{
  bool valid{ false };
  Eigen::Vector3d minimum{ Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()) };
  Eigen::Vector3d maximum{ Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity()) };

  void include(const bodies::AABB& aabb)
  {
    minimum = minimum.cwiseMin(aabb.min());
    maximum = maximum.cwiseMax(aabb.max());
    valid = true;
  }

  double center(int axis) const { return (minimum[axis] + maximum[axis]) / 2.0; }
  double extent(int axis) const { return maximum[axis] - minimum[axis]; }
};

struct Config
{
  std::string frame_id;
  double width{};
  double depth{};
  double height{};
  double center_y{};
  double floor_z{};
  double front_x{};
  double wall_thickness{};
  double floor_thickness{};
  std::array<double, 3> target_size{};
  std::array<double, 3> target_center{};
  double q_open{};
  double approach_clearance{};
  double pre_grasp_clearance{};
  int seed_count{};
  std::uint32_t seed_base{};
  double ik_timeout{};

  double backX() const { return front_x + depth; }
  double minY() const { return center_y - width / 2.0; }
  double maxY() const { return center_y + width / 2.0; }
  double topZ() const { return floor_z + height; }
  double centerX() const { return front_x + depth / 2.0; }
  double centerZ() const { return floor_z + height / 2.0; }
};

Config loadConfig(const std::string& path)
{
  const YAML::Node root = YAML::LoadFile(path);
  Config config;
  config.frame_id = root["frame_id"].as<std::string>();
  const auto box = root["box"];
  config.width = box["internal_width_y"].as<double>();
  config.depth = box["internal_depth_x"].as<double>();
  config.height = box["internal_height_z"].as<double>();
  config.center_y = box["box_center_y"].as<double>();
  config.floor_z = box["box_floor_z"].as<double>();
  config.front_x = box["front_inner_plane_x"].as<double>();
  config.wall_thickness = box["wall_thickness"].as<double>();
  config.floor_thickness = box["floor_thickness"].as<double>();
  for (std::size_t index = 0; index < 3; ++index)
  {
    config.target_size[index] = root["target"]["size_xyz"][index].as<double>();
    config.target_center[index] = root["target"]["center_xyz"][index].as<double>();
  }
  config.q_open = root["display"]["initial_left_finger"].as<double>();
  const auto audit = root["reachability_audit"];
  config.approach_clearance = audit["approach_clearance_above_rim"].as<double>();
  config.pre_grasp_clearance = audit["pre_grasp_clearance_above_object"].as<double>();
  config.seed_count = audit["ik_seed_count"].as<int>();
  config.seed_base = audit["ik_seed_base"].as<std::uint32_t>();
  config.ik_timeout = audit["ik_timeout_s"].as<double>();
  return config;
}

moveit_msgs::msg::CollisionObject boxObject(const Config& config, const std::string& id,
                                            const std::array<double, 3>& size,
                                            const std::array<double, 3>& center)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = config.frame_id;
  object.id = id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.assign(size.begin(), size.end());
  geometry_msgs::msg::Pose pose;
  pose.position.x = center[0];
  pose.position.y = center[1];
  pose.position.z = center[2];
  pose.orientation.w = 1.0;
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

std::vector<moveit_msgs::msg::CollisionObject> sceneObjects(const Config& config)
{
  const double outside_width = config.width + 2.0 * config.wall_thickness;
  const double outside_depth = config.depth + 2.0 * config.wall_thickness;
  return {
    boxObject(config, "box_floor", { outside_depth, outside_width, config.floor_thickness },
              { config.centerX(), config.center_y, config.floor_z - config.floor_thickness / 2.0 }),
    boxObject(config, "box_front_wall", { config.wall_thickness, outside_width, config.height },
              { config.front_x - config.wall_thickness / 2.0, config.center_y, config.centerZ() }),
    boxObject(config, "box_back_wall", { config.wall_thickness, outside_width, config.height },
              { config.backX() + config.wall_thickness / 2.0, config.center_y, config.centerZ() }),
    boxObject(config, "box_left_wall", { config.depth, config.wall_thickness, config.height },
              { config.centerX(), config.maxY() + config.wall_thickness / 2.0, config.centerZ() }),
    boxObject(config, "box_right_wall", { config.depth, config.wall_thickness, config.height },
              { config.centerX(), config.minY() - config.wall_thickness / 2.0, config.centerZ() }),
    boxObject(config, "target_object", config.target_size, config.target_center),
  };
}

struct FingerGeometry
{
  Bounds left_tcp;
  Bounds right_tcp;
  const Bounds* low_y{};
  const Bounds* high_y{};
  double gap{};
  double vertical_min{};
  double vertical_max{};
  double vertical_center{};
  double vertical_extent{};
};

struct CollisionAudit
{
  bool bounds_valid{ false };
  bool self_collision{ false };
  bool robot_box_collision{ false };
  bool gripper_box_collision{ false };
  bool finger_target_contact{ false };
  bool nonfinger_target_collision{ false };
  double joint_margin{ std::numeric_limits<double>::quiet_NaN() };
  double self_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double environment_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double minimum_clearance{ std::numeric_limits<double>::quiet_NaN() };
  std::set<std::pair<std::string, std::string>> pairs;

  bool collisionFree() const
  {
    return bounds_valid && !self_collision && !robot_box_collision && !nonfinger_target_collision;
  }
};

struct Candidate
{
  std::string mode;
  std::string id;
  double lift{};
  double yaw{};
  double pitch{};
  bool within_limits{ false };
};

struct StageAggregate
{
  int seeds{};
  int ik{};
  int free{};
  double best_joint_margin{ -std::numeric_limits<double>::infinity() };
  double best_clearance{ -std::numeric_limits<double>::infinity() };
  std::set<std::pair<std::string, std::string>> pairs;
};

struct HeightResult
{
  double height{};
  double tcp_z{};
  double floor_clearance{};
  double vertical_overlap{};
  bool geometry_valid{};
  int q_open_ik{};
  int q_open_free{};
  int q_contact_ik{};
  int q_contact_free{};
  double best_joint_margin{ -std::numeric_limits<double>::infinity() };
  double best_clearance{ -std::numeric_limits<double>::infinity() };
  std::set<std::pair<std::string, std::string>> failure_pairs;
};
}  // namespace

class GraspHeightGeometryAudit
{
public:
  explicit GraspHeightGeometryAudit(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
  }

  void run()
  {
    config_ = loadConfig(node_->get_parameter("geometry_config").as_string());
    geometry_report_ = node_->get_parameter("geometry_report").as_string();
    aperture_csv_ = node_->get_parameter("aperture_csv").as_string();
    heights_csv_ = node_->get_parameter("heights_csv").as_string();
    corrected_csv_ = node_->get_parameter("corrected_csv").as_string();
    corrected_report_ = node_->get_parameter("corrected_report").as_string();
    fine_mode_ = node_->has_parameter("fine_boundary_mode") &&
                 node_->get_parameter("fine_boundary_mode").as_bool();
    reference_yaml_ = node_->has_parameter("reference_yaml") ?
      node_->get_parameter("reference_yaml").as_string() : std::string();

    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    model_ = loader_->getModel();
    if (!model_)
      throw std::runtime_error("Robot model unavailable");
    left_arm_ = model_->getJointModelGroup("left_arm");
    whole_body_ = model_->getJointModelGroup("whole_body");
    left_with_torso_ = model_->getJointModelGroup("left_arm_with_torso");
    if (!left_arm_ || !whole_body_ || !left_with_torso_ || !left_arm_->getSolverInstance())
      throw std::runtime_error("Required SRDF group or left-arm IK solver unavailable");

    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    for (const auto& object : sceneObjects(config_))
      if (!scene_->processCollisionObjectMsg(object))
        throw std::runtime_error("PlanningScene rejected " + object.id);

    validateUnchangedGeometry();
    if (fine_mode_)
    {
      // Reuse the previously audited aperture root; do not repeat the aperture sweep.
      q_contact_ = 0.0327611885070801;
      open_geometry_ = fingerGeometry(config_.q_open);
      contact_geometry_ = fingerGeometry(q_contact_);
      if (std::abs(contact_geometry_.gap - kObjectWidth) > 1e-9)
        throw std::runtime_error("Reused q_contact no longer produces a 50 mm collision-geometry aperture");
      computeActualInnerFaces();
    }
    else
    {
      sweepAperture();
      grasp_face_z_min_ = contact_geometry_.vertical_min;
      grasp_face_z_max_ = contact_geometry_.vertical_max;
      grasp_face_center_ = contact_geometry_.vertical_center;
      left_inner_face_ = { grasp_face_z_min_, grasp_face_z_max_ };
      right_inner_face_ = { grasp_face_z_min_, grasp_face_z_max_ };
      inner_face_triangle_counts_[kLeftFinger] = 0;
      inner_face_triangle_counts_[kRightFinger] = 0;
    }
    auditHeightsAndSelect();
    runCorrectedReachability();
    writeGeometryReport();
    if (fine_mode_)
      writeReferenceYaml();
  }

private:
  void validateUnchangedGeometry() const
  {
    const double bottom = config_.target_center[2] - config_.target_size[2] / 2.0;
    const double top = config_.target_center[2] + config_.target_size[2] / 2.0;
    if (std::abs(config_.target_size[0] - 0.050) > 1e-12 ||
        std::abs(config_.target_size[1] - 0.050) > 1e-12 ||
        std::abs(config_.target_size[2] - 0.050) > 1e-12 ||
        std::abs(bottom - 0.940) > 1e-12 || std::abs(top - 0.990) > 1e-12)
      throw std::runtime_error("The approved 50 mm target geometry is not present");
  }

  Bounds linkBoundsInFrame(moveit::core::RobotState& state, const std::string& link_name,
                           const Eigen::Isometry3d& frame_world) const
  {
    const moveit::core::LinkModel* link = model_->getLinkModel(link_name);
    if (!link)
      throw std::runtime_error("Unknown link " + link_name);
    const auto& shapes = link->getShapes();
    Bounds bounds;
    for (std::size_t index = 0; index < shapes.size(); ++index)
    {
      std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shapes[index].get()));
      if (!body)
        throw std::runtime_error("Cannot create collision body for " + link_name);
      body->setPose(frame_world.inverse() * state.getCollisionBodyTransform(link, index));
      bodies::AABB aabb;
      body->computeBoundingBox(aabb);
      bounds.include(aabb);
    }
    if (!bounds.valid)
      throw std::runtime_error("No collision geometry for " + link_name);
    return bounds;
  }

  FingerGeometry fingerGeometry(double q) const
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setVariablePosition(kFingerJoint, q);
    state.update();
    const Eigen::Isometry3d tcp_world = state.getGlobalLinkTransform(kTcpLink);
    FingerGeometry geometry;
    geometry.left_tcp = linkBoundsInFrame(state, kLeftFinger, tcp_world);
    geometry.right_tcp = linkBoundsInFrame(state, kRightFinger, tcp_world);
    if (geometry.left_tcp.center(1) < geometry.right_tcp.center(1))
    {
      geometry.low_y = &geometry.left_tcp;
      geometry.high_y = &geometry.right_tcp;
    }
    else
    {
      geometry.low_y = &geometry.right_tcp;
      geometry.high_y = &geometry.left_tcp;
    }
    geometry.gap = geometry.high_y->minimum.y() - geometry.low_y->maximum.y();
    geometry.vertical_min = std::max(geometry.left_tcp.minimum.z(), geometry.right_tcp.minimum.z());
    geometry.vertical_max = std::min(geometry.left_tcp.maximum.z(), geometry.right_tcp.maximum.z());
    geometry.vertical_center = (geometry.vertical_min + geometry.vertical_max) / 2.0;
    geometry.vertical_extent = geometry.vertical_max - geometry.vertical_min;
    return geometry;
  }

  std::pair<double, double> actualInnerFaceRange(const std::string& link_name,
                                                 bool low_y_finger,
                                                 const Bounds& collision_bounds) const
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setVariablePosition(kFingerJoint, q_contact_);
    state.update();
    const Eigen::Isometry3d tcp_world = state.getGlobalLinkTransform(kTcpLink);
    const moveit::core::LinkModel* link = model_->getLinkModel(link_name);
    const double inner_y = low_y_finger ? collision_bounds.maximum.y() : collision_bounds.minimum.y();
    const std::array<double, 3> tolerances = { 1e-7, 1e-5, 5e-4 };
    for (const double tolerance : tolerances)
    {
      double z_min = std::numeric_limits<double>::infinity();
      double z_max = -std::numeric_limits<double>::infinity();
      std::size_t selected_triangles = 0;
      const auto& shapes = link->getShapes();
      for (std::size_t shape_index = 0; shape_index < shapes.size(); ++shape_index)
      {
        if (shapes[shape_index]->type != shapes::MESH)
          continue;
        const auto* mesh = static_cast<const shapes::Mesh*>(shapes[shape_index].get());
        const Eigen::Isometry3d transform =
          tcp_world.inverse() * state.getCollisionBodyTransform(link, shape_index);
        for (unsigned int triangle_index = 0; triangle_index < mesh->triangle_count; ++triangle_index)
        {
          std::array<Eigen::Vector3d, 3> points;
          for (int vertex = 0; vertex < 3; ++vertex)
          {
            const unsigned int mesh_index = mesh->triangles[triangle_index * 3 + vertex];
            points[vertex] = transform * Eigen::Vector3d(
              mesh->vertices[mesh_index * 3], mesh->vertices[mesh_index * 3 + 1],
              mesh->vertices[mesh_index * 3 + 2]);
          }
          const Eigen::Vector3d normal = (points[1] - points[0]).cross(points[2] - points[0]);
          if (normal.norm() < 1e-12 || std::abs(normal.normalized().y()) < 0.90)
            continue;
          bool on_inner_plane = true;
          for (const auto& point : points)
            on_inner_plane = on_inner_plane && std::abs(point.y() - inner_y) <= tolerance;
          if (!on_inner_plane)
            continue;
          ++selected_triangles;
          for (const auto& point : points)
          {
            z_min = std::min(z_min, point.z());
            z_max = std::max(z_max, point.z());
          }
        }
      }
      if (selected_triangles > 0 && z_max > z_min)
      {
        inner_face_triangle_counts_[link_name] = selected_triangles;
        inner_face_tolerances_[link_name] = tolerance;
        return { z_min, z_max };
      }
    }
    throw std::runtime_error("Could not isolate actual inward-facing mesh triangles for " + link_name);
  }

  void computeActualInnerFaces()
  {
    const bool left_is_low = contact_geometry_.left_tcp.center(1) < contact_geometry_.right_tcp.center(1);
    left_inner_face_ = actualInnerFaceRange(kLeftFinger, left_is_low, contact_geometry_.left_tcp);
    right_inner_face_ = actualInnerFaceRange(kRightFinger, !left_is_low, contact_geometry_.right_tcp);
    grasp_face_z_min_ = std::max(left_inner_face_.first, right_inner_face_.first);
    grasp_face_z_max_ = std::min(left_inner_face_.second, right_inner_face_.second);
    if (grasp_face_z_max_ <= grasp_face_z_min_)
      throw std::runtime_error("Left/right actual inner grasp-face Z ranges do not overlap");
    grasp_face_center_ = (grasp_face_z_min_ + grasp_face_z_max_) / 2.0;
  }

  void writeApertureRow(std::ofstream& output, double q, const FingerGeometry& geometry,
                        bool selected, const std::string& classification) const
  {
    const double penetration = std::max(0.0, kObjectWidth - geometry.gap);
    output << std::setprecision(15) << q << ',' << q << ',' << geometry.gap << ','
           << (geometry.gap - kObjectWidth) << ',' << penetration << ','
           << geometry.left_tcp.minimum.x() << ',' << geometry.left_tcp.maximum.x() << ','
           << geometry.left_tcp.minimum.y() << ',' << geometry.left_tcp.maximum.y() << ','
           << geometry.left_tcp.minimum.z() << ',' << geometry.left_tcp.maximum.z() << ','
           << geometry.right_tcp.minimum.x() << ',' << geometry.right_tcp.maximum.x() << ','
           << geometry.right_tcp.minimum.y() << ',' << geometry.right_tcp.maximum.y() << ','
           << geometry.right_tcp.minimum.z() << ',' << geometry.right_tcp.maximum.z() << ','
           << boolString(selected) << ',' << classification << '\n';
  }

  void sweepAperture()
  {
    const auto& joint_bounds = model_->getVariableBounds(kFingerJoint);
    q_lower_ = joint_bounds.min_position_;
    q_upper_ = joint_bounds.max_position_;
    constexpr double step = 0.00025;
    std::vector<std::pair<double, FingerGeometry>> sweep;
    for (double q = q_lower_; q < q_upper_ - 1e-12; q += step)
      sweep.emplace_back(q, fingerGeometry(q));
    sweep.emplace_back(q_upper_, fingerGeometry(q_upper_));

    bool bracket_found = false;
    double lower_q = 0.0;
    double upper_q = 0.0;
    for (std::size_t index = 1; index < sweep.size(); ++index)
    {
      const double prior = sweep[index - 1].second.gap - kObjectWidth;
      const double current = sweep[index].second.gap - kObjectWidth;
      if ((prior <= 0.0 && current >= 0.0) || (prior >= 0.0 && current <= 0.0))
      {
        lower_q = sweep[index - 1].first;
        upper_q = sweep[index].first;
        bracket_found = true;
        break;
      }
    }
    if (!bracket_found)
      throw std::runtime_error("50 mm aperture is not bracketed by the finger joint limits");

    for (int iteration = 0; iteration < 60; ++iteration)
    {
      const double midpoint = (lower_q + upper_q) / 2.0;
      if (fingerGeometry(midpoint).gap < kObjectWidth)
        lower_q = midpoint;
      else
        upper_q = midpoint;
    }
    q_contact_ = (lower_q + upper_q) / 2.0;
    contact_geometry_ = fingerGeometry(q_contact_);
    open_geometry_ = fingerGeometry(config_.q_open);
    if (std::abs(contact_geometry_.gap - kObjectWidth) > 1e-9)
      throw std::runtime_error("Aperture root solve did not reach 50 mm");

    std::filesystem::create_directories(std::filesystem::path(aperture_csv_).parent_path());
    std::ofstream output(aperture_csv_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write aperture sweep CSV");
    output << "q_joint1_m,q_mimic_joint2_m,inside_gap_m,gap_minus_50mm_m,object_penetration_m,"
              "left_tcp_x_min,left_tcp_x_max,left_tcp_y_min,left_tcp_y_max,left_tcp_z_min,left_tcp_z_max,"
              "right_tcp_x_min,right_tcp_x_max,right_tcp_y_min,right_tcp_y_max,right_tcp_z_min,right_tcp_z_max,"
              "selected,classification\n";
    for (const auto& row : sweep)
      writeApertureRow(output, row.first, row.second, false, "SWEEP_0.25MM");
    writeApertureRow(output, q_contact_, contact_geometry_, true,
                     "KINEMATIC_CONTACT_CONFIGURATION_FOR_50MM_OBJECT");
    output.flush();
    if (!output)
      throw std::runtime_error("Aperture CSV flush failed");
    output.close();
  }

  geometry_msgs::msg::Pose topEntryPose(double grasp_center_z) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = config_.target_center[0];
    pose.position.y = config_.target_center[1];
    // Preserve the previously audited grasp-height coordinate convention: the requested
    // height locates the midpoint of the complete finger collision AABB. The actual
    // inward-facing mesh surface is audited separately and must not redefine this pose.
    pose.position.z = grasp_center_z + contact_geometry_.vertical_center;
    pose.orientation.x = 0.0;
    pose.orientation.y = 1.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
  }

  bool withinBound(const std::string& variable, double value) const
  {
    const auto& bounds = model_->getVariableBounds(variable);
    return !bounds.position_bounded_ || (value >= bounds.min_position_ && value <= bounds.max_position_);
  }

  moveit::core::RobotState seededState(double lift, double yaw, double pitch, double finger_q,
                                       std::uint32_t seed) const
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setVariablePosition("lift_joint", lift);
    state.setVariablePosition("waist_yaw_joint", yaw);
    state.setVariablePosition("waist_pitch_joint", pitch);
    state.setVariablePosition(kFingerJoint, finger_q);
    state.setVariablePosition("openarm_right_finger_joint1", config_.q_open);
    std::mt19937 generator(seed);
    for (const std::string& variable : left_arm_->getVariableNames())
    {
      const auto& bounds = model_->getVariableBounds(variable);
      if (bounds.position_bounded_)
      {
        std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
        state.setVariablePosition(variable, distribution(generator));
      }
    }
    state.update();
    return state;
  }

  double jointMargin(const moveit::core::RobotState& state) const
  {
    double margin = std::numeric_limits<double>::infinity();
    for (const std::string& variable : left_with_torso_->getVariableNames())
    {
      const auto& bounds = model_->getVariableBounds(variable);
      if (!bounds.position_bounded_)
        continue;
      const double q = state.getVariablePosition(variable);
      margin = std::min(margin, std::min(q - bounds.min_position_, bounds.max_position_ - q));
    }
    return margin;
  }

  static bool isBox(const std::string& name)
  {
    return name == "box_floor" || name == "box_front_wall" || name == "box_back_wall" ||
           name == "box_left_wall" || name == "box_right_wall";
  }

  static bool isFinger(const std::string& name)
  {
    return name == kLeftFinger || name == kRightFinger;
  }

  static bool isGripper(const std::string& name)
  {
    return isFinger(name) || name == "openarm_left_link7" || name == kTcpLink;
  }

  CollisionAudit checkState(moveit::core::RobotState& state, bool allow_finger_target) const
  {
    state.update();
    CollisionAudit audit;
    audit.bounds_valid = state.satisfiesBounds(whole_body_);
    audit.joint_margin = jointMargin(state);

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 2000;
    request.max_contacts_per_pair = 200;
    collision_detection::CollisionResult self_result;
    scene_->checkSelfCollision(request, self_result, state);
    audit.self_collision = self_result.collision;
    for (const auto& entry : self_result.contacts)
      audit.pairs.insert(entry.first);

    collision_detection::CollisionResult raw_result;
    scene_->checkCollision(request, raw_result, state);
    for (const auto& entry : raw_result.contacts)
    {
      const bool target = entry.first.first == "target_object" || entry.first.second == "target_object";
      const std::string robot_link = entry.first.first == "target_object" ? entry.first.second : entry.first.first;
      if (target)
      {
        if (isFinger(robot_link))
          audit.finger_target_contact = true;
        else
          audit.nonfinger_target_collision = true;
      }
    }

    collision_detection::AllowedCollisionMatrix task_acm = scene_->getAllowedCollisionMatrix();
    if (allow_finger_target)
    {
      task_acm.setEntry(kLeftFinger, "target_object", true);
      task_acm.setEntry(kRightFinger, "target_object", true);
    }
    collision_detection::CollisionResult task_result;
    scene_->checkCollision(request, task_result, state, task_acm);
    for (const auto& entry : task_result.contacts)
    {
      audit.pairs.insert(entry.first);
      const bool first_box = isBox(entry.first.first);
      const bool second_box = isBox(entry.first.second);
      if (first_box || second_box)
      {
        audit.robot_box_collision = true;
        const std::string robot_link = first_box ? entry.first.second : entry.first.first;
        if (isGripper(robot_link))
          audit.gripper_box_collision = true;
      }
      if (entry.first.first == "target_object" || entry.first.second == "target_object")
        audit.nonfinger_target_collision = true;
    }
    audit.environment_clearance = scene_->getCollisionEnv()->distanceRobot(state, task_acm);
    audit.self_clearance = scene_->getCollisionEnv()->distanceSelf(state, task_acm);
    audit.minimum_clearance = std::min(audit.environment_clearance, audit.self_clearance);
    return audit;
  }

  double verticalOverlap(double grasp_height) const
  {
    const double tcp_z = config_.floor_z + grasp_height + contact_geometry_.vertical_center;
    const double finger_bottom = tcp_z - grasp_face_z_max_;
    const double finger_top = tcp_z - grasp_face_z_min_;
    const double object_bottom = config_.floor_z;
    const double object_top = config_.floor_z + config_.target_size[2];
    return std::max(0.0, std::min(finger_top, object_top) - std::max(finger_bottom, object_bottom));
  }

  HeightResult evaluateHeight(double height, std::size_t height_index) const
  {
    HeightResult result;
    result.height = height;
    const geometry_msgs::msg::Pose pose = topEntryPose(config_.floor_z + height);
    result.tcp_z = pose.position.z;
    const double finger_bottom = pose.position.z - contact_geometry_.vertical_max;
    result.floor_clearance = finger_bottom - config_.floor_z;
    result.vertical_overlap = verticalOverlap(height);
    result.geometry_valid = result.floor_clearance >= -1e-9 &&
                            result.vertical_overlap + 1e-9 >= kMinimumVerticalOverlap;

    for (int seed_id = 0; seed_id < config_.seed_count; ++seed_id)
    {
      const std::uint32_t seed = config_.seed_base + 700000U +
        static_cast<std::uint32_t>(height_index * 100 + seed_id);
      moveit::core::RobotState open_state = seededState(0.35, 0.0, 0.0, config_.q_open, seed);
      const bool open_ik = open_state.setFromIK(left_arm_, pose, kTcpLink, config_.ik_timeout);
      if (open_ik)
      {
        ++result.q_open_ik;
        const CollisionAudit audit = checkState(open_state, false);
        if (audit.collisionFree())
          ++result.q_open_free;
        else
          result.failure_pairs.insert(audit.pairs.begin(), audit.pairs.end());
      }

      moveit::core::RobotState contact_state = seededState(0.35, 0.0, 0.0, q_contact_, seed);
      const bool contact_ik = contact_state.setFromIK(left_arm_, pose, kTcpLink, config_.ik_timeout);
      if (contact_ik)
      {
        ++result.q_contact_ik;
        const CollisionAudit audit = checkState(contact_state, true);
        if (audit.collisionFree())
        {
          ++result.q_contact_free;
          result.best_joint_margin = std::max(result.best_joint_margin, audit.joint_margin);
          result.best_clearance = std::max(result.best_clearance, audit.minimum_clearance);
        }
        else
          result.failure_pairs.insert(audit.pairs.begin(), audit.pairs.end());
      }
    }
    return result;
  }

  void auditHeightsAndSelect()
  {
    std::vector<double> heights;
    if (fine_mode_)
      heights = { 0.0475, 0.0480, 0.0485, 0.0490, 0.0495, 0.0500 };
    else
      heights = { 0.025, 0.030, 0.035, 0.040, 0.045 };
    for (const double clearance : { 0.0, 0.001, 0.002, 0.003, 0.005 })
    {
      const double minimum_center_height =
        contact_geometry_.vertical_max - contact_geometry_.vertical_center + clearance;
      floor_clearance_minimum_heights_.push_back({ clearance, minimum_center_height });
      if (!fine_mode_ && minimum_center_height >= 0.0 && minimum_center_height <= 0.050)
        heights.push_back(minimum_center_height);
    }
    std::sort(heights.begin(), heights.end());
    heights.erase(std::unique(heights.begin(), heights.end(), [](double a, double b) {
      return std::abs(a - b) < 1e-9;
    }), heights.end());

    for (std::size_t index = 0; index < heights.size(); ++index)
      height_results_.push_back(evaluateHeight(heights[index], index));

    std::vector<const HeightResult*> valid;
    for (const auto& result : height_results_)
      if (result.geometry_valid && result.q_open_free > 0 && result.q_contact_free > 0)
        valid.push_back(&result);
    if (!valid.empty())
    {
      std::sort(valid.begin(), valid.end(), [](const HeightResult* a, const HeightResult* b) {
        const bool a_robust_clearance = a->floor_clearance >= 0.002 - 1e-12;
        const bool b_robust_clearance = b->floor_clearance >= 0.002 - 1e-12;
        if (a_robust_clearance != b_robust_clearance)
          return a_robust_clearance;
        if (std::abs(a->vertical_overlap - b->vertical_overlap) > 1e-12)
          return a->vertical_overlap > b->vertical_overlap;
        if (std::abs(a->best_clearance - b->best_clearance) > 1e-12)
          return a->best_clearance > b->best_clearance;
        return a->height < b->height;
      });
      selected_height_ = valid.front()->height;
      selected_height_result_ = *valid.front();
      grasp_geometry_possible_ = true;
    }

    std::ofstream output(heights_csv_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write grasp-height candidate CSV");
    output << "grasp_center_height_above_object_bottom_m,tcp_world_z_m,inner_face_world_z_min_m,"
              "inner_face_world_z_max_m,collision_geometry_world_z_min_m,collision_geometry_world_z_max_m,"
              "finger_floor_clearance_m,vertical_grasp_face_overlap_m,minimum_overlap_requirement_m,"
              "aperture_m,object_penetration_m,symmetric_y_contact,geometry_valid,q_open_ik_count,"
              "q_open_collision_free_count,q_contact_ik_count,q_contact_collision_free_count,"
              "best_joint_margin,best_collision_clearance_m,failure_link_pairs,selected\n";
    for (const auto& result : height_results_)
    {
      const double inner_min = result.tcp_z - grasp_face_z_max_;
      const double inner_max = result.tcp_z - grasp_face_z_min_;
      const double collision_min = result.tcp_z - contact_geometry_.vertical_max;
      const double collision_max = result.tcp_z - contact_geometry_.vertical_min;
      const bool symmetric = std::abs(contact_geometry_.gap - kObjectWidth) <= 1e-9;
      output << std::setprecision(15) << result.height << ',' << result.tcp_z << ',' << inner_min << ','
             << inner_max << ',' << collision_min << ',' << collision_max << ',' << result.floor_clearance << ','
             << result.vertical_overlap << ',' << kMinimumVerticalOverlap << ',' << contact_geometry_.gap << ','
             << std::max(0.0, kObjectWidth - contact_geometry_.gap) << ',' << boolString(symmetric) << ','
             << boolString(result.geometry_valid)
             << ',' << result.q_open_ik << ',' << result.q_open_free << ',' << result.q_contact_ik << ','
             << result.q_contact_free << ',' << result.best_joint_margin << ',' << result.best_clearance << ','
             << csvEscape(pairString(result.failure_pairs)) << ','
             << boolString(grasp_geometry_possible_ && std::abs(result.height - selected_height_) < 1e-12) << '\n';
    }
    output.flush();
    if (!output)
      throw std::runtime_error("Height candidate CSV flush failed");
    output.close();
  }

  std::vector<Candidate> candidates()
  {
    std::vector<Candidate> output;
    int id = 0;
    for (int mm = 0; mm <= 700; mm += 50)
    {
      Candidate candidate;
      candidate.mode = "LIFT_ONLY";
      candidate.id = "LIFT_ONLY_" + std::to_string(id++);
      candidate.lift = mm / 1000.0;
      candidate.within_limits = withinBound("lift_joint", candidate.lift);
      output.push_back(candidate);
    }

    const std::array<double, 5> yaw_degrees = { -10.0, -5.0, 0.0, 5.0, 10.0 };
    const std::array<double, 6> pitch_degrees = { -10.0, 0.0, 10.0, 20.0, 30.0, 45.0 };
    const double pitch_upper = model_->getVariableBounds("waist_pitch_joint").max_position_;
    used_pitch_upper_ = kRequestedPitchUpper;
    if (std::abs(kRequestedPitchUpper - pitch_upper) <= 1e-6)
      used_pitch_upper_ = pitch_upper;
    id = 0;
    for (int mm = 0; mm <= 700; mm += 50)
      for (const double yaw_degree : yaw_degrees)
        for (const double pitch_degree : pitch_degrees)
        {
          Candidate candidate;
          candidate.mode = "LIFT_YAW_PITCH";
          candidate.id = "LIFT_YAW_PITCH_" + std::to_string(id++);
          candidate.lift = mm / 1000.0;
          candidate.yaw = yaw_degree * kPi / 180.0;
          candidate.pitch = pitch_degree == 45.0 ? used_pitch_upper_ : pitch_degree * kPi / 180.0;
          candidate.within_limits = withinBound("lift_joint", candidate.lift) &&
                                    withinBound("waist_yaw_joint", candidate.yaw) &&
                                    withinBound("waist_pitch_joint", candidate.pitch);
          output.push_back(candidate);
        }
    return output;
  }

  std::map<std::string, geometry_msgs::msg::Pose> correctedStages() const
  {
    return {
      { "APPROACH", topEntryPose(config_.topZ() + config_.approach_clearance) },
      { "PRE_GRASP", topEntryPose(config_.floor_z + config_.target_size[2] + config_.pre_grasp_clearance) },
      { "GRASP", topEntryPose(config_.floor_z + selected_height_) },
    };
  }

  void runCorrectedReachability()
  {
    if (!grasp_geometry_possible_)
    {
      writeNoGraspReport();
      return;
    }
    const auto stage_poses = correctedStages();
    const auto all_candidates = candidates();
    std::ofstream output(corrected_csv_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write corrected reachability CSV");
    output << "mode,candidate_id,lift_m,yaw_rad,pitch_rad,stage,finger_q_m,seed_id,explicit_seed,"
              "candidate_within_limits,ik_success,joint_limits_valid,self_collision,robot_box_collision,"
              "gripper_box_collision,finger_target_contact,nonfinger_target_collision,collision_free,"
              "minimum_joint_margin,min_self_clearance_m,min_environment_clearance_m,min_collision_clearance_m,"
              "collision_link_pairs\n";

    std::map<std::string, std::map<std::string, StageAggregate>> aggregate;
    for (std::size_t candidate_index = 0; candidate_index < all_candidates.size(); ++candidate_index)
    {
      const Candidate& candidate = all_candidates[candidate_index];
      for (std::size_t stage_index = 0; stage_index < 3; ++stage_index)
      {
        const std::string stage_name = std::array<std::string, 3>{ "APPROACH", "PRE_GRASP", "GRASP" }[stage_index];
        const bool grasp = stage_name == "GRASP";
        const double q = grasp ? q_contact_ : config_.q_open;
        StageAggregate& stage_aggregate = aggregate[candidate.id][stage_name];
        if (!candidate.within_limits)
        {
          output << candidate.mode << ',' << candidate.id << ',' << candidate.lift << ',' << candidate.yaw << ','
                 << candidate.pitch << ',' << stage_name << ',' << q
                 << ",-1,0,false,false,false,false,false,false,false,false,false,nan,nan,nan,nan,"
                 << csvEscape("CANDIDATE_OUTSIDE_URDF_LIMIT") << '\n';
          continue;
        }
        for (int seed_id = 0; seed_id < config_.seed_count; ++seed_id)
        {
          const std::uint32_t seed = config_.seed_base + 900000U +
            static_cast<std::uint32_t>(candidate_index * 1000 + stage_index * 100 + seed_id);
          moveit::core::RobotState state = seededState(candidate.lift, candidate.yaw, candidate.pitch, q, seed);
          const bool ik = state.setFromIK(left_arm_, stage_poses.at(stage_name), kTcpLink, config_.ik_timeout);
          ++stage_aggregate.seeds;
          CollisionAudit audit;
          if (ik)
          {
            ++stage_aggregate.ik;
            audit = checkState(state, grasp);
            if (audit.collisionFree())
            {
              ++stage_aggregate.free;
              stage_aggregate.best_joint_margin = std::max(stage_aggregate.best_joint_margin, audit.joint_margin);
              stage_aggregate.best_clearance = std::max(stage_aggregate.best_clearance, audit.minimum_clearance);
            }
            else
              stage_aggregate.pairs.insert(audit.pairs.begin(), audit.pairs.end());
          }
          output << std::setprecision(15) << candidate.mode << ',' << candidate.id << ',' << candidate.lift << ','
                 << candidate.yaw << ',' << candidate.pitch << ',' << stage_name << ',' << q << ',' << seed_id << ','
                 << seed << ",true," << boolString(ik) << ',' << boolString(ik && audit.bounds_valid) << ','
                 << boolString(ik && audit.self_collision) << ',' << boolString(ik && audit.robot_box_collision) << ','
                 << boolString(ik && audit.gripper_box_collision) << ',' << boolString(ik && audit.finger_target_contact)
                 << ',' << boolString(ik && audit.nonfinger_target_collision) << ','
                 << boolString(ik && audit.collisionFree()) << ','
                 << (ik ? audit.joint_margin : std::numeric_limits<double>::quiet_NaN()) << ','
                 << (ik ? audit.self_clearance : std::numeric_limits<double>::quiet_NaN()) << ','
                 << (ik ? audit.environment_clearance : std::numeric_limits<double>::quiet_NaN()) << ','
                 << (ik ? audit.minimum_clearance : std::numeric_limits<double>::quiet_NaN()) << ','
                 << csvEscape(ik ? pairString(audit.pairs) : "IK_FAILURE") << '\n';
        }
      }
      if ((candidate_index + 1) % 50 == 0)
        RCLCPP_INFO(node_->get_logger(), "Corrected candidates: %zu / %zu", candidate_index + 1,
                    all_candidates.size());
    }
    output.flush();
    if (!output)
      throw std::runtime_error("Corrected reachability CSV flush failed");
    output.close();
    writeCorrectedReport(all_candidates, aggregate);
  }

  void writeNoGraspReport() const
  {
    std::ofstream csv(corrected_csv_, std::ios::trunc);
    csv << "status,reason\nNOT_RUN,NO_GEOMETRICALLY_VALID_GRASP_HEIGHT\n";
    csv.close();
    std::ofstream report(corrected_report_, std::ios::trunc);
    report << "# Corrected center reachability\n\nNo grasp height simultaneously met the geometry and IK/collision gates. "
              "OMPL was not run.\n";
    report.close();
  }

  void writeCorrectedReport(const std::vector<Candidate>& candidates,
                            const std::map<std::string, std::map<std::string, StageAggregate>>& aggregate) const
  {
    struct Complete
    {
      const Candidate* candidate{};
      double clearance{};
    };
    std::vector<Complete> baseline;
    std::vector<Complete> proposed;
    std::map<std::string, bool> baseline_stage;
    std::map<std::string, bool> proposed_stage;
    std::set<std::pair<std::string, std::string>> failure_pairs;
    for (const Candidate& candidate : candidates)
    {
      const auto found = aggregate.find(candidate.id);
      if (found == aggregate.end())
        continue;
      bool complete = true;
      double clearance = std::numeric_limits<double>::infinity();
      for (const char* stage : { "APPROACH", "PRE_GRASP", "GRASP" })
      {
        const auto stage_found = found->second.find(stage);
        const bool possible = stage_found != found->second.end() && stage_found->second.free > 0;
        if (candidate.mode == "LIFT_ONLY")
          baseline_stage[stage] = baseline_stage[stage] || possible;
        else
          proposed_stage[stage] = proposed_stage[stage] || possible;
        complete = complete && possible;
        if (stage_found != found->second.end())
        {
          clearance = std::min(clearance, stage_found->second.best_clearance);
          failure_pairs.insert(stage_found->second.pairs.begin(), stage_found->second.pairs.end());
        }
      }
      if (complete)
        (candidate.mode == "LIFT_ONLY" ? baseline : proposed).push_back({ &candidate, clearance });
    }
    auto clearance_sort = [](const Complete& a, const Complete& b) { return a.clearance > b.clearance; };
    std::sort(baseline.begin(), baseline.end(), clearance_sort);
    std::sort(proposed.begin(), proposed.end(), clearance_sort);

    std::ofstream output(corrected_report_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write corrected reachability report");
    output << "# Top-open center reachability — corrected grasp geometry\n\n";
    output << "- Scope: explicit-seed IK and static RobotState/FCL checks; no OMPL or trajectory\n";
    output << "- q_open: " << config_.q_open << " m\n";
    output << "- q_contact_50mm: " << std::setprecision(15) << q_contact_
           << " m (`KINEMATIC_CONTACT_CONFIGURATION_FOR_50MM_OBJECT`)\n";
    output << "- Selected grasp-center height above object bottom: " << selected_height_ << " m\n";
    output << "- Finger-floor clearance: " << selected_height_result_.floor_clearance << " m\n";
    output << "- Vertical grasp-face overlap: " << selected_height_result_.vertical_overlap << " m\n";
    output << "- GRASP task-local ACM: only both left finger links vs `target_object`; global SRDF unchanged\n";
    output << "- Requested Pitch endpoint: " << kRequestedPitchUpper << " rad (45 deg)\n";
    output << "- Used Pitch endpoint: " << used_pitch_upper_ << " rad (URDF upper bound substitution within 1e-6 rad)\n\n";
    output << "|Stage|LIFT_ONLY|LIFT_YAW_PITCH|Proposed only|\n|---|---|---|---|\n";
    for (const char* stage : { "APPROACH", "PRE_GRASP", "GRASP" })
      output << '|' << stage << '|' << (baseline_stage[stage] ? "possible" : "not found") << '|'
             << (proposed_stage[stage] ? "possible" : "not found") << '|'
             << (!baseline_stage[stage] && proposed_stage[stage] ? "yes" : "no") << "|\n";
    output << "\n## Complete three-stage candidates\n\n";
    if (baseline.empty())
      output << "- LIFT_ONLY: none\n";
    else
      output << "- Best LIFT_ONLY: Lift=" << baseline.front().candidate->lift
             << " m; minimum clearance=" << baseline.front().clearance << " m\n";
    if (proposed.empty())
      output << "- LIFT_YAW_PITCH: none\n";
    else
      output << "- Best LIFT_YAW_PITCH: Lift=" << proposed.front().candidate->lift
             << " m, Yaw=" << proposed.front().candidate->yaw << " rad, Pitch="
             << proposed.front().candidate->pitch << " rad; minimum clearance="
             << proposed.front().clearance << " m\n";
    output << "\nObserved rejected-state pairs: `" << pairString(failure_pairs) << "`\n\n";
    output << "## Reference-generation gate\n\n";
    output << (baseline.empty() ?
      "LIFT_ONLY does not pass all three stages. Do not begin OMPL reference generation.\n" :
      "LIFT_ONLY has collision-free IK for all three stages. The static IK gate is satisfied, but OMPL was not run in this task.\n");
    output.flush();
    if (!output)
      throw std::runtime_error("Corrected report flush failed");
  }

  void writeGeometryReport() const
  {
    const auto report_bounds = [](std::ofstream& out, const std::string& name, const Bounds& bounds) {
      out << "|" << name << "|[" << bounds.minimum.x() << ", " << bounds.maximum.x() << "]|["
          << bounds.minimum.y() << ", " << bounds.maximum.y() << "]|[" << bounds.minimum.z() << ", "
          << bounds.maximum.z() << "]|\n";
    };
    std::ofstream output(geometry_report_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write geometry report");
    output << "# 50 mm grasp-height geometry audit\n\n";
    output << "## Fixed inputs and frame mapping\n\n";
    output << "- Target: 0.050 m cube, center [0.675, 0.200, 0.965] m, Z [0.940, 0.990] m\n";
    output << "- Top-entry orientation: RPY [0, pi, 0], quaternion xyzw [0, 1, 0, 0]\n";
    output << "- TCP local +Z -> world -Z; local +/-Y -> world +/-Y\n";
    output << "- Global SRDF/ACM and robot collision geometry were not changed\n\n";
    output << "## Finger collision AABBs in TCP frame\n\n";
    output << "|State/link|X [m]|Y [m]|Z [m]|\n|---|---|---|---|\n";
    report_bounds(output, "q_open left", open_geometry_.left_tcp);
    report_bounds(output, "q_open right", open_geometry_.right_tcp);
    report_bounds(output, "q_contact left", contact_geometry_.left_tcp);
    report_bounds(output, "q_contact right", contact_geometry_.right_tcp);
    output << "\n- q_open: " << config_.q_open << " m; inside gap=" << open_geometry_.gap << " m\n";
    output << "- q_contact_50mm: " << std::setprecision(15) << q_contact_
           << " m; inside gap=" << contact_geometry_.gap << " m\n";
    output << "- Classification: `KINEMATIC_CONTACT_CONFIGURATION_FOR_50MM_OBJECT`; this is not force-grasp validation\n";
    output << "- Actual inward-facing grasp triangles, left local-Z range: [" << left_inner_face_.first << ", "
           << left_inner_face_.second << "] m\n";
    output << "- Actual inward-facing grasp triangles, right local-Z range: [" << right_inner_face_.first << ", "
           << right_inner_face_.second << "] m\n";
    output << "- Common actual inner-face local-Z range: [" << grasp_face_z_min_ << ", "
           << grasp_face_z_max_ << "] m\n";
    output << "- Actual inner-face extraction: triangles parallel to the inward Y plane at the collision-mesh inner boundary (|normal.Y| >= 0.90).\n";
    output << "- Selected mesh triangles: left=" << inner_face_triangle_counts_.at(kLeftFinger)
           << ", right=" << inner_face_triangle_counts_.at(kRightFinger) << "\n";
    output << "- Full finger collision-AABB vertical height: " << contact_geometry_.vertical_extent << " m\n";
    output << "- TCP-to-finger-bottom downward offset under top entry: " << contact_geometry_.vertical_max << " m\n";
    output << "- TCP-to-full-collision-AABB center downward offset used by the existing grasp-height convention: "
           << contact_geometry_.vertical_center << " m\n";
    output << "- TCP-to-actual-inner-grasp-face center downward offset: " << grasp_face_center_ << " m\n";
    output << "- The fine search preserves the existing AABB-center grasp-height convention; the actual inner face is used only for overlap measurement.\n\n";
    output << "## Floor-clearance-derived minimum grasp-center heights\n\n";
    output << "|Required floor clearance [m]|Minimum grasp-center height above object bottom [m]|\n|---|---|\n";
    for (const auto& value : floor_clearance_minimum_heights_)
      output << '|' << value.first << '|' << value.second << "|\n";
    output << "\nThe 15 mm vertical overlap threshold is a development-only minimum contact-height criterion, not a final physical grasp-stability criterion.\n\n";
    output << "## Selection\n\n";
    if (grasp_geometry_possible_)
    {
      output << "- Selected grasp-center height above object bottom: " << selected_height_ << " m\n";
      output << "- Finger-floor clearance: " << selected_height_result_.floor_clearance << " m\n";
      output << "- Finger/object vertical overlap: " << selected_height_result_.vertical_overlap << " m\n";
      output << "- q_open static-pose collision-free IK count: " << selected_height_result_.q_open_free << " / "
             << config_.seed_count << "\n";
      output << "- q_contact task-ACM collision-free IK count: " << selected_height_result_.q_contact_free << " / "
             << config_.seed_count << "\n";
      output << "- A grasp center above the object center is an intentional side-grasp offset for an object resting on a floor.\n";
    }
    else
    {
      output << "No candidate simultaneously met floor clearance, >=15 mm overlap, symmetric aperture, and collision-free IK. Robot/object geometry was not changed.\n";
    }
    output << "\n## Finger state lifecycle (definition only)\n\n"
              "- APPROACH/PRE_GRASP/DESCENT entry: q_open\n"
              "- GRASP closure: q_open -> q_contact_50mm\n"
              "- LIFT (future work): q_contact_50mm with attached object\n"
              "- No force control, attachment, trajectory, OMPL, controller, or hardware was used here.\n";
    output.flush();
    if (!output)
      throw std::runtime_error("Geometry report flush failed");
    output.close();
  }

  void writeReferenceYaml() const
  {
    if (reference_yaml_.empty())
      return;
    std::ofstream output(reference_yaml_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write reference-grasp YAML");
    output << std::setprecision(15);
    output << "reference_grasp:\n";
    if (!grasp_geometry_possible_)
    {
      output << "  status: NOT_SELECTED_FINE_BOUNDARY_SEARCH_FAILED\n"
                "  selection_scope: CENTRAL_OBJECT_ONLY\n"
                "  reuse_policy: FIXED_FOR_ALL_XYZ_AND_ALL_ABLATIONS\n"
                "  grasp_axis: OBJECT_Y\n"
                "  approach_axis: WORLD_MINUS_Z\n"
                "  not_a_force_grasp_validation: true\n"
                "  q_contact_50mm_m: " << q_contact_ << "\n"
                "  selected_grasp_height_m: null\n";
      output.close();
      return;
    }

    const geometry_msgs::msg::Pose tcp = topEntryPose(config_.floor_z + selected_height_);
    const double tcp_target_z = selected_height_ + contact_geometry_.vertical_center -
                                config_.target_size[2] / 2.0;
    const bool at_least_two_mm = selected_height_result_.floor_clearance >= 0.002 - 1e-12;
    output << "  status: REFERENCE_GRASP_SELECTED_FINE_BOUNDARY_SEARCH\n"
              "  selection_scope: CENTRAL_OBJECT_ONLY\n"
              "  reuse_policy: FIXED_FOR_ALL_XYZ_AND_ALL_ABLATIONS\n"
              "  grasp_axis: OBJECT_Y\n"
              "  approach_axis: WORLD_MINUS_Z\n"
              "  not_a_force_grasp_validation: true\n"
              "  feasibility_classification: "
           << (at_least_two_mm ? "SIMULATION_FEASIBLE_GEOMETRIC_AUDIT_ONLY" :
                                 "SIMULATION_FEASIBLE_LOW_CLEARANCE_GRASP") << "\n\n";
    output << "  object_pose:\n"
              "    frame_id: world\n"
              "    xyz_m: [" << config_.target_center[0] << ", " << config_.target_center[1] << ", "
           << config_.target_center[2] << "]\n"
              "    quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]\n"
              "    size_xyz_m: [0.050, 0.050, 0.050]\n\n";
    output << "  grasp_tcp_pose:\n"
              "    frame_id: world\n"
              "    xyz_m: [" << tcp.position.x << ", " << tcp.position.y << ", " << tcp.position.z << "]\n"
              "    quaternion_xyzw: [0.0, 1.0, 0.0, 0.0]\n"
              "    rpy_rad: [0.0, 3.141592653589793, 0.0]\n\n";
    output << "  q_open_m: " << config_.q_open << "\n"
              "  q_contact_50mm_m: " << q_contact_ << "\n"
              "  q_contact_classification: KINEMATIC_CONTACT_CONFIGURATION_FOR_50MM_OBJECT\n"
              "  selected_grasp_height_above_object_bottom_m: " << selected_height_ << "\n"
              "  finger_floor_clearance_m: " << selected_height_result_.floor_clearance << "\n"
              "  finger_object_vertical_overlap_m: " << selected_height_result_.vertical_overlap << "\n"
              "  development_minimum_overlap_m: " << kMinimumVerticalOverlap << "\n\n";
    output << "  tcp_to_target_object_transform:\n"
              "    parent_frame: openarm_left_hand_tcp\n"
              "    child_object: target_object\n"
              "    xyz_m: [0.0, 0.0, " << tcp_target_z << "]\n"
              "    quaternion_xyzw: [0.0, 1.0, 0.0, 0.0]\n\n";
    output << "  finger_state_policy:\n"
              "    APPROACH: q_open\n"
              "    PRE_GRASP: q_open\n"
              "    DESCENT: q_open\n"
              "    GRASP: q_contact_50mm\n"
              "    future_LIFT: q_contact_50mm_with_attached_object\n\n";
    output << "  pitch_endpoint_policy:\n"
              "    requested_45deg_rad: " << kRequestedPitchUpper << "\n"
              "    actual_used_urdf_upper_bound_rad: " << used_pitch_upper_ << "\n"
              "    joint_limit_modified: false\n\n";
    output << "  restrictions:\n"
              "    force_grasp_validated: false\n"
              "    hardware_robustness_validated: false\n"
              "    global_acm_modified: false\n"
              "    ompl_run_in_this_stage: false\n"
              "    trajectory_generated: false\n";
    output.flush();
    if (!output)
      throw std::runtime_error("Reference-grasp YAML flush failed");
    output.close();
  }

  rclcpp::Node::SharedPtr node_;
  Config config_;
  std::string geometry_report_;
  std::string aperture_csv_;
  std::string heights_csv_;
  std::string corrected_csv_;
  std::string corrected_report_;
  std::string reference_yaml_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup* left_arm_{};
  const moveit::core::JointModelGroup* whole_body_{};
  const moveit::core::JointModelGroup* left_with_torso_{};
  planning_scene::PlanningScenePtr scene_;
  double q_lower_{};
  double q_upper_{};
  double q_contact_{};
  double selected_height_{};
  double used_pitch_upper_{};
  bool grasp_geometry_possible_{ false };
  bool fine_mode_{ false };
  FingerGeometry open_geometry_;
  FingerGeometry contact_geometry_;
  HeightResult selected_height_result_;
  std::vector<HeightResult> height_results_;
  std::vector<std::pair<double, double>> floor_clearance_minimum_heights_;
  std::pair<double, double> left_inner_face_{};
  std::pair<double, double> right_inner_face_{};
  double grasp_face_z_min_{};
  double grasp_face_z_max_{};
  double grasp_face_center_{};
  mutable std::map<std::string, std::size_t> inner_face_triangle_counts_;
  mutable std::map<std::string, double> inner_face_tolerances_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<rclcpp::Node>(
      "grasp_height_geometry_audit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    GraspHeightGeometryAudit audit(node);
    audit.run();
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("grasp_height_geometry_audit"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
