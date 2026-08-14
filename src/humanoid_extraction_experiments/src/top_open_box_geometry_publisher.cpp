#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr char kExpectedSceneId[] = "TOP_OPEN_BOX_600X400X150_GEOMETRY_VALIDATION";

std::string timestampNow()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream out;
  out << std::put_time(&local, "%Y-%m-%dT%H:%M:%S%z");
  return out.str();
}

std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha)
{
  std_msgs::msg::ColorRGBA value;
  value.r = red;
  value.g = green;
  value.b = blue;
  value.a = alpha;
  return value;
}

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

struct BoxGeometry
{
  std::string id;
  std::array<double, 3> size;
  std::array<double, 3> center;
};

struct Bounds
{
  Eigen::Vector3d minimum{ Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()) };
  Eigen::Vector3d maximum{ Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity()) };
  bool valid{ false };

  void include(const bodies::AABB& aabb)
  {
    minimum = minimum.cwiseMin(aabb.min());
    maximum = maximum.cwiseMax(aabb.max());
    valid = true;
  }

  void include(const Bounds& other)
  {
    if (!other.valid)
      return;
    minimum = minimum.cwiseMin(other.minimum);
    maximum = maximum.cwiseMax(other.maximum);
    valid = true;
  }
};

struct CandidatePlacement
{
  std::string id;
  std::string label;
  double nominal_clearance{};
  double front_x{};
  double back_x{};
  double target_x{};
  double outer_surface_clearance{};
  double fixed_body_distance{};
  double left_target_distance{};
  double right_target_distance{};
  bool left_reachable{};
  bool right_reachable{};
  bool amr_overlap{};
  bool waist_overlap{};
};

struct GeometryConfig
{
  std::string scene_id;
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
  double initial_lift{};
  double initial_yaw{};
  double initial_pitch{};
  double initial_left_finger{};
  double initial_right_finger{};

  double backX() const { return front_x + depth; }
  double minY() const { return center_y - width / 2.0; }
  double maxY() const { return center_y + width / 2.0; }
  double topZ() const { return floor_z + height; }
  double centerX() const { return front_x + depth / 2.0; }
  double centerZ() const { return floor_z + height / 2.0; }
};

GeometryConfig loadConfig(const std::string& path)
{
  const YAML::Node root = YAML::LoadFile(path);
  GeometryConfig config;
  config.scene_id = root["scene_id"].as<std::string>();
  config.frame_id = root["frame_id"].as<std::string>();
  if (config.scene_id != kExpectedSceneId)
    throw std::runtime_error("Unexpected scene_id: " + config.scene_id);

  const YAML::Node box = root["box"];
  config.width = box["internal_width_y"].as<double>();
  config.depth = box["internal_depth_x"].as<double>();
  config.height = box["internal_height_z"].as<double>();
  config.center_y = box["box_center_y"].as<double>();
  config.floor_z = box["box_floor_z"].as<double>();
  config.front_x = box["front_inner_plane_x"].as<double>();
  config.wall_thickness = box["wall_thickness"].as<double>();
  config.floor_thickness = box["floor_thickness"].as<double>();
  if (!box["front_wall_enabled"].as<bool>() || !box["top_open"].as<bool>())
    throw std::runtime_error("Geometry must have a front wall and an open top");

  const YAML::Node target_size = root["target"]["size_xyz"];
  const YAML::Node target_center = root["target"]["center_xyz"];
  for (std::size_t index = 0; index < 3; ++index)
  {
    config.target_size[index] = target_size[index].as<double>();
    config.target_center[index] = target_center[index].as<double>();
  }
  const std::array<double, 3> expected_target_center = {
    config.centerX(), config.center_y, config.floor_z + config.target_size[2] / 2.0
  };
  for (std::size_t index = 0; index < 3; ++index)
  {
    if (std::abs(config.target_center[index] - expected_target_center[index]) > 1e-12)
      throw std::runtime_error("Target center is inconsistent with centered-on-floor placement");
  }

  const YAML::Node display = root["display"];
  config.initial_lift = display["initial_lift"].as<double>();
  config.initial_yaw = display["initial_yaw"].as<double>();
  config.initial_pitch = display["initial_pitch"].as<double>();
  config.initial_left_finger = display["initial_left_finger"].as<double>();
  config.initial_right_finger = display["initial_right_finger"].as<double>();
  return config;
}

std::vector<BoxGeometry> makeBoxGeometry(const GeometryConfig& config)
{
  const double wall_z = config.centerZ();
  const double outside_width = config.width + 2.0 * config.wall_thickness;
  const double outside_depth = config.depth + 2.0 * config.wall_thickness;
  return {
    { "box_floor",
      { outside_depth, outside_width, config.floor_thickness },
      { config.centerX(), config.center_y, config.floor_z - config.floor_thickness / 2.0 } },
    { "box_front_wall",
      { config.wall_thickness, outside_width, config.height },
      { config.front_x - config.wall_thickness / 2.0, config.center_y, wall_z } },
    { "box_back_wall",
      { config.wall_thickness, outside_width, config.height },
      { config.backX() + config.wall_thickness / 2.0, config.center_y, wall_z } },
    { "box_left_wall",
      { config.depth, config.wall_thickness, config.height },
      { config.centerX(), config.maxY() + config.wall_thickness / 2.0, wall_z } },
    { "box_right_wall",
      { config.depth, config.wall_thickness, config.height },
      { config.centerX(), config.minY() - config.wall_thickness / 2.0, wall_z } },
  };
}

Eigen::Isometry3d urdfPoseToEigen(const urdf::Pose& pose)
{
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  pose.rotation.getQuaternion(qx, qy, qz, qw);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  return transform;
}

shapes::ShapePtr shapeFromUrdfGeometry(const urdf::GeometrySharedPtr& geometry)
{
  if (!geometry)
    return {};
  if (geometry->type == urdf::Geometry::BOX)
  {
    const auto box = std::dynamic_pointer_cast<urdf::Box>(geometry);
    return shapes::ShapePtr(new shapes::Box(box->dim.x, box->dim.y, box->dim.z));
  }
  if (geometry->type == urdf::Geometry::CYLINDER)
  {
    const auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
    return shapes::ShapePtr(new shapes::Cylinder(cylinder->radius, cylinder->length));
  }
  if (geometry->type == urdf::Geometry::SPHERE)
  {
    const auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
    return shapes::ShapePtr(new shapes::Sphere(sphere->radius));
  }
  if (geometry->type == urdf::Geometry::MESH)
  {
    const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
    Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    if (scale.isZero())
      scale = Eigen::Vector3d::Ones();
    return shapes::ShapePtr(shapes::createMeshFromResource(mesh->filename, scale));
  }
  return {};
}

void includeShape(Bounds& bounds, const shapes::ShapeConstPtr& shape, const Eigen::Isometry3d& pose)
{
  if (!shape)
    return;
  std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shape.get()));
  if (!body)
    return;
  body->setPose(pose);
  bodies::AABB aabb;
  body->computeBoundingBox(aabb);
  bounds.include(aabb);
}

Bounds boundsFromBoxGeometry(const std::vector<BoxGeometry>& geometry)
{
  Bounds bounds;
  for (const auto& box : geometry)
  {
    bodies::AABB aabb;
    aabb.extend(Eigen::Vector3d(box.center[0] - box.size[0] / 2.0,
                                box.center[1] - box.size[1] / 2.0,
                                box.center[2] - box.size[2] / 2.0));
    aabb.extend(Eigen::Vector3d(box.center[0] + box.size[0] / 2.0,
                                box.center[1] + box.size[1] / 2.0,
                                box.center[2] + box.size[2] / 2.0));
    bounds.include(aabb);
  }
  return bounds;
}

Bounds innerBounds(const GeometryConfig& config, double front_x)
{
  Bounds bounds;
  bodies::AABB aabb;
  aabb.extend(Eigen::Vector3d(front_x, config.minY(), config.floor_z));
  aabb.extend(Eigen::Vector3d(front_x + config.depth, config.maxY(), config.topZ()));
  bounds.include(aabb);
  return bounds;
}

double signedAxisGap(double first_min, double first_max, double second_min, double second_max)
{
  if (first_max < second_min)
    return second_min - first_max;
  if (second_max < first_min)
    return first_min - second_max;
  return -(std::min(first_max, second_max) - std::max(first_min, second_min));
}

double minimumDistance(const Bounds& first, const Bounds& second)
{
  if (!first.valid || !second.valid)
    return std::numeric_limits<double>::quiet_NaN();
  double squared_distance = 0.0;
  for (int axis = 0; axis < 3; ++axis)
  {
    const double gap = signedAxisGap(first.minimum[axis], first.maximum[axis],
                                     second.minimum[axis], second.maximum[axis]);
    if (gap > 0.0)
      squared_distance += gap * gap;
  }
  return std::sqrt(squared_distance);
}

bool overlaps(const Bounds& first, const Bounds& second)
{
  if (!first.valid || !second.valid)
    return false;
  for (int axis = 0; axis < 3; ++axis)
  {
    if (signedAxisGap(first.minimum[axis], first.maximum[axis], second.minimum[axis], second.maximum[axis]) > 0.0)
      return false;
  }
  return true;
}

moveit_msgs::msg::CollisionObject collisionObject(const GeometryConfig& config, const BoxGeometry& geometry)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = config.frame_id;
  object.id = geometry.id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(geometry.size.begin(), geometry.size.end());
  geometry_msgs::msg::Pose pose;
  pose.position.x = geometry.center[0];
  pose.position.y = geometry.center[1];
  pose.position.z = geometry.center[2];
  pose.orientation.w = 1.0;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

visualization_msgs::msg::Marker cubeMarker(const GeometryConfig& config, const BoxGeometry& geometry,
                                           int id, const std_msgs::msg::ColorRGBA& marker_color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.ns = "top_open_collision_objects";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = geometry.center[0];
  marker.pose.position.y = geometry.center[1];
  marker.pose.position.z = geometry.center[2];
  marker.pose.orientation.w = 1.0;
  marker.scale.x = geometry.size[0];
  marker.scale.y = geometry.size[1];
  marker.scale.z = geometry.size[2];
  marker.color = marker_color;
  return marker;
}

visualization_msgs::msg::Marker wireframeMarker(const GeometryConfig& config, double front_x, int id,
                                                const std::string& marker_namespace,
                                                const std_msgs::msg::ColorRGBA& marker_color,
                                                double line_width)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;
  marker.color = marker_color;

  const double x0 = front_x;
  const double x1 = front_x + config.depth;
  const double y0 = config.minY();
  const double y1 = config.maxY();
  const double z0 = config.floor_z;
  const double z1 = config.topZ();
  const std::array<geometry_msgs::msg::Point, 8> vertices = {
    point(x0, y0, z0), point(x1, y0, z0), point(x1, y1, z0), point(x0, y1, z0),
    point(x0, y0, z1), point(x1, y0, z1), point(x1, y1, z1), point(x0, y1, z1)
  };
  const std::array<std::pair<int, int>, 12> edges = {
    std::pair<int, int>{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
    { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
    { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
  };
  for (const auto& edge : edges)
  {
    marker.points.push_back(vertices[edge.first]);
    marker.points.push_back(vertices[edge.second]);
  }
  return marker;
}

visualization_msgs::msg::Marker arrowMarker(const GeometryConfig& config, int id,
                                            const geometry_msgs::msg::Point& start,
                                            const geometry_msgs::msg::Point& end,
                                            const std_msgs::msg::ColorRGBA& marker_color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.ns = "top_open_annotations";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.points = { start, end };
  marker.scale.x = 0.018;
  marker.scale.y = 0.045;
  marker.scale.z = 0.065;
  marker.color = marker_color;
  return marker;
}

visualization_msgs::msg::Marker textMarker(const GeometryConfig& config)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.ns = "top_open_annotations";
  marker.id = 103;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = config.centerX();
  marker.pose.position.y = config.center_y;
  marker.pose.position.z = config.topZ() + 0.33;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.065;
  marker.color = color(1.0F, 1.0F, 1.0F, 1.0F);
  marker.text = "TOP-OPEN BOX — GEOMETRY VALIDATION ONLY\n"
                "Internal size: 600 × 400 × 150 mm\n"
                "Front wall: ENABLED\nTop: OPEN\nNO TRAJECTORY PLANNING";
  return marker;
}

visualization_msgs::msg::Marker labelMarker(const GeometryConfig& config, int id,
                                            const std::string& marker_namespace,
                                            const std::string& text,
                                            const std::array<double, 3>& position,
                                            const std_msgs::msg::ColorRGBA& marker_color,
                                            double height)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = position[0];
  marker.pose.position.y = position[1];
  marker.pose.position.z = position[2];
  marker.pose.orientation.w = 1.0;
  marker.scale.z = height;
  marker.color = marker_color;
  marker.text = text;
  return marker;
}
}  // namespace

class TopOpenBoxGeometryPublisher
{
public:
  explicit TopOpenBoxGeometryPublisher(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
  }

  void initialize()
  {
    const std::string geometry_config = node_->get_parameter("geometry_config").as_string();
    const std::string collision_csv = node_->get_parameter("static_collision_csv").as_string();
    const std::string aabb_csv = node_->get_parameter("aabb_csv").as_string();
    const std::string candidates_csv = node_->get_parameter("placement_candidates_csv").as_string();
    if (geometry_config.empty() || collision_csv.empty() || aabb_csv.empty() || candidates_csv.empty())
      throw std::runtime_error("geometry_config and all validation output parameters are required");

    config_ = loadConfig(geometry_config);
    geometry_ = makeBoxGeometry(config_);
    for (const auto& geometry : geometry_)
      box_ids_.insert(geometry.id);

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_)
      throw std::runtime_error("Failed to load robot model for static geometry audit");

    state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    state_->setToDefaultValues();
    setIfPresent("lift_joint", config_.initial_lift);
    setIfPresent("waist_yaw_joint", config_.initial_yaw);
    setIfPresent("waist_pitch_joint", config_.initial_pitch);
    setIfPresent("openarm_left_finger_joint1", config_.initial_left_finger);
    setIfPresent("openarm_right_finger_joint1", config_.initial_right_finger);
    state_->update();

    computeRobotBounds();
    computePlacementCandidates();
    writeAabbComparison(aabb_csv);
    writePlacementCandidates(candidates_csv);

    local_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    collision_objects_.reserve(geometry_.size());
    for (const auto& geometry : geometry_)
    {
      collision_objects_.push_back(collisionObject(config_, geometry));
      if (!local_scene_->processCollisionObjectMsg(collision_objects_.back()))
        throw std::runtime_error("PlanningScene rejected collision object: " + geometry.id);
    }

    const auto durable_qos = rclcpp::QoS(1).reliable().transient_local();
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/top_open_box_geometry_markers", durable_qos);
    scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
      "/top_open_box_geometry_scene", durable_qos);
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    writeStaticCollisionAudit(collision_csv);
    publishAll();
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), [this]() { publishAll(); });

    RCLCPP_INFO(node_->get_logger(),
                "Geometry-only scene active: %s; five boxes, no MoveGroup/IK/OMPL/trajectory/controller",
                config_.scene_id.c_str());
  }

private:
  void setIfPresent(const std::string& variable, double value)
  {
    const auto& variable_names = robot_model_->getVariableNames();
    if (std::find(variable_names.begin(), variable_names.end(), variable) == variable_names.end())
      throw std::runtime_error("Required independent variable missing: " + variable);
    state_->setVariablePosition(variable, value);
  }

  std::vector<std::string> categoriesForLink(const std::string& name) const
  {
    std::vector<std::string> categories{ "whole_robot" };
    const bool fixed_body = name == "base_link" || name == "lift_fixed_link" || name == "lift_moving_link" ||
                            name == "waist_yaw_link" || name == "waist_pitch_link" ||
                            name == "left_arm_mount_link" || name == "right_arm_mount_link";
    if (fixed_body)
      categories.push_back("fixed_body");
    if (name == "base_link")
      categories.push_back("amr");
    if (name == "waist_yaw_link" || name == "waist_pitch_link")
      categories.push_back("waist");
    if (name.rfind("openarm_left_", 0) == 0)
      categories.push_back("left_arm");
    if (name.rfind("openarm_right_", 0) == 0)
      categories.push_back("right_arm");
    return categories;
  }

  void includeInCategories(std::map<std::string, Bounds>& destination, const std::string& link_name,
                           const Bounds& shape_bounds)
  {
    for (const auto& category : categoriesForLink(link_name))
      destination[category].include(shape_bounds);
  }

  void computeRobotBounds()
  {
    const auto& urdf_model = robot_model_loader_->getURDF();
    if (!urdf_model)
      throw std::runtime_error("URDF model unavailable for visual AABB audit");

    for (const moveit::core::LinkModel* link_model : robot_model_->getLinkModels())
    {
      const std::string& link_name = link_model->getName();
      const Eigen::Isometry3d& link_world = state_->getGlobalLinkTransform(link_model);

      const auto& collision_shapes = link_model->getShapes();
      const auto& collision_origins = link_model->getCollisionOriginTransforms();
      for (std::size_t index = 0; index < collision_shapes.size(); ++index)
      {
        Bounds shape_bounds;
        includeShape(shape_bounds, collision_shapes[index], link_world * collision_origins[index]);
        includeInCategories(collision_bounds_, link_name, shape_bounds);
      }

      const urdf::LinkConstSharedPtr urdf_link = urdf_model->getLink(link_name);
      if (!urdf_link)
        continue;
      const auto process_visual = [&](const urdf::VisualSharedPtr& visual) {
        if (!visual)
          return;
        const shapes::ShapePtr shape = shapeFromUrdfGeometry(visual->geometry);
        if (!shape)
          throw std::runtime_error("Failed to construct visual shape for link: " + link_name);
        Bounds shape_bounds;
        includeShape(shape_bounds, shape, link_world * urdfPoseToEigen(visual->origin));
        includeInCategories(visual_bounds_, link_name, shape_bounds);
      };
      if (!urdf_link->visual_array.empty())
      {
        for (const auto& visual : urdf_link->visual_array)
          process_visual(visual);
      }
      else
      {
        process_visual(urdf_link->visual);
      }
    }

    current_box_outer_ = boundsFromBoxGeometry(geometry_);
    current_box_inner_ = innerBounds(config_, config_.front_x);
    fixed_body_combined_ = visual_bounds_["fixed_body"];
    fixed_body_combined_.include(collision_bounds_["fixed_body"]);
    amr_combined_ = visual_bounds_["amr"];
    amr_combined_.include(collision_bounds_["amr"]);
    waist_combined_ = visual_bounds_["waist"];
    waist_combined_.include(collision_bounds_["waist"]);
    if (!fixed_body_combined_.valid)
      throw std::runtime_error("Fixed-body visual/collision AABB is empty");
    robot_fixed_front_x_ = fixed_body_combined_.maximum.x();
    left_reach_ = maximumKinematicReach("openarm_left_link0", "openarm_left_hand_tcp");
    right_reach_ = maximumKinematicReach("openarm_right_link0", "openarm_right_hand_tcp");
  }

  double maximumKinematicReach(const std::string& base_link, const std::string& tcp_link) const
  {
    const moveit::core::LinkModel* current = robot_model_->getLinkModel(tcp_link);
    if (!current)
      throw std::runtime_error("Missing TCP link for reach audit: " + tcp_link);
    double reach = 0.0;
    while (current && current->getName() != base_link)
    {
      reach += current->getJointOriginTransform().translation().norm();
      current = current->getParentLinkModel();
    }
    if (!current)
      throw std::runtime_error("TCP is not descended from expected arm base: " + base_link);
    return reach;
  }

  Bounds combinedCategory(const std::string& category) const
  {
    Bounds result;
    const auto visual = visual_bounds_.find(category);
    if (visual != visual_bounds_.end())
      result.include(visual->second);
    const auto collision = collision_bounds_.find(category);
    if (collision != collision_bounds_.end())
      result.include(collision->second);
    return result;
  }

  void computePlacementCandidates()
  {
    const Eigen::Vector3d left_base = state_->getGlobalLinkTransform("openarm_left_link0").translation();
    const Eigen::Vector3d right_base = state_->getGlobalLinkTransform("openarm_right_link0").translation();
    const std::array<std::pair<char, double>, 3> definitions = {
      std::pair<char, double>{ 'A', 0.10 }, { 'B', 0.20 }, { 'C', 0.30 }
    };
    for (const auto& definition : definitions)
    {
      GeometryConfig candidate_config = config_;
      candidate_config.front_x = robot_fixed_front_x_ + definition.second;
      const std::vector<BoxGeometry> candidate_geometry = makeBoxGeometry(candidate_config);
      const Bounds candidate_outer = boundsFromBoxGeometry(candidate_geometry);
      const Eigen::Vector3d target(candidate_config.centerX(), candidate_config.center_y,
                                   candidate_config.floor_z + candidate_config.target_size[2] / 2.0);
      CandidatePlacement candidate;
      candidate.id = std::string(1, definition.first);
      candidate.label = candidate.id + " — " + std::to_string(static_cast<int>(definition.second * 1000.0)) +
                        " mm body clearance";
      candidate.nominal_clearance = definition.second;
      candidate.front_x = candidate_config.front_x;
      candidate.back_x = candidate_config.backX();
      candidate.target_x = target.x();
      candidate.outer_surface_clearance = candidate_outer.minimum.x() - robot_fixed_front_x_;
      candidate.fixed_body_distance = minimumDistance(fixed_body_combined_, candidate_outer);
      candidate.left_target_distance = (target - left_base).norm();
      candidate.right_target_distance = (target - right_base).norm();
      candidate.left_reachable = candidate.left_target_distance <= left_reach_;
      candidate.right_reachable = candidate.right_target_distance <= right_reach_;
      candidate.amr_overlap = overlaps(amr_combined_, candidate_outer);
      candidate.waist_overlap = overlaps(waist_combined_, candidate_outer);
      candidates_.push_back(candidate);
    }
  }

  void writeAabbComparison(const std::string& path) const
  {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write AABB comparison CSV: " + path);
    output << "category,geometry_type,x_min_m,x_max_m,y_min_m,y_max_m,z_min_m,z_max_m,"
              "signed_gap_x_to_box_m,signed_gap_y_to_box_m,signed_gap_z_to_box_m,"
              "minimum_aabb_distance_m,aabb_overlap\n";
    const std::array<std::string, 6> categories = {
      "fixed_body", "left_arm", "right_arm", "amr", "waist", "whole_robot"
    };
    const auto write_bounds = [&](const std::string& category, const std::string& geometry_type,
                                  const Bounds& bounds) {
      if (!bounds.valid)
        return;
      output << category << ',' << geometry_type << ',' << std::setprecision(12)
             << bounds.minimum.x() << ',' << bounds.maximum.x() << ','
             << bounds.minimum.y() << ',' << bounds.maximum.y() << ','
             << bounds.minimum.z() << ',' << bounds.maximum.z() << ','
             << signedAxisGap(bounds.minimum.x(), bounds.maximum.x(), current_box_outer_.minimum.x(), current_box_outer_.maximum.x()) << ','
             << signedAxisGap(bounds.minimum.y(), bounds.maximum.y(), current_box_outer_.minimum.y(), current_box_outer_.maximum.y()) << ','
             << signedAxisGap(bounds.minimum.z(), bounds.maximum.z(), current_box_outer_.minimum.z(), current_box_outer_.maximum.z()) << ','
             << minimumDistance(bounds, current_box_outer_) << ',' << (overlaps(bounds, current_box_outer_) ? "true" : "false") << '\n';
    };
    for (const auto& category : categories)
    {
      const auto visual = visual_bounds_.find(category);
      if (visual != visual_bounds_.end())
        write_bounds(category, "visual", visual->second);
      const auto collision = collision_bounds_.find(category);
      if (collision != collision_bounds_.end())
        write_bounds(category, "collision", collision->second);
    }
    write_bounds("current_box_outer", "geometry", current_box_outer_);
    write_bounds("current_box_inner", "geometry", current_box_inner_);
    output.flush();
    if (!output)
      throw std::runtime_error("Failed while writing AABB comparison CSV: " + path);
  }

  void writePlacementCandidates(const std::string& path) const
  {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write placement candidate CSV: " + path);
    output << "candidate,robot_fixed_front_x_m,front_inner_plane_x_m,back_inner_plane_x_m,"
              "nominal_inner_plane_clearance_m,actual_outer_surface_clearance_m,fixed_body_min_aabb_distance_m,"
              "target_x_m,target_y_m,target_z_m,left_link0_to_target_distance_m,left_max_kinematic_reach_m,"
              "left_approx_reachable,right_link0_to_target_distance_m,right_max_kinematic_reach_m,"
              "right_approx_reachable,amr_aabb_overlap,waist_aabb_overlap\n";
    for (const auto& candidate : candidates_)
    {
      output << candidate.id << ',' << std::setprecision(12) << robot_fixed_front_x_ << ','
             << candidate.front_x << ',' << candidate.back_x << ',' << candidate.nominal_clearance << ','
             << candidate.outer_surface_clearance << ',' << candidate.fixed_body_distance << ','
             << candidate.target_x << ',' << config_.center_y << ','
             << config_.floor_z + config_.target_size[2] / 2.0 << ','
             << candidate.left_target_distance << ',' << left_reach_ << ','
             << (candidate.left_reachable ? "true" : "false") << ','
             << candidate.right_target_distance << ',' << right_reach_ << ','
             << (candidate.right_reachable ? "true" : "false") << ','
             << (candidate.amr_overlap ? "true" : "false") << ','
             << (candidate.waist_overlap ? "true" : "false") << '\n';
    }
    output.flush();
    if (!output)
      throw std::runtime_error("Failed while writing placement candidate CSV: " + path);
  }

  visualization_msgs::msg::MarkerArray makeMarkers() const
  {
    visualization_msgs::msg::MarkerArray result;
    for (std::size_t index = 0; index < geometry_.size(); ++index)
    {
      std_msgs::msg::ColorRGBA marker_color;
      if (geometry_[index].id == "box_front_wall")
        marker_color = color(0.05F, 0.25F, 1.0F, 1.0F);
      else if (geometry_[index].id == "box_back_wall")
        marker_color = color(1.0F, 0.08F, 0.08F, 0.80F);
      else if (geometry_[index].id == "box_left_wall")
        marker_color = color(0.05F, 0.95F, 0.15F, 0.80F);
      else if (geometry_[index].id == "box_right_wall")
        marker_color = color(1.0F, 0.85F, 0.05F, 0.80F);
      else
        marker_color = color(0.58F, 0.60F, 0.64F, 0.72F);
      result.markers.push_back(cubeMarker(config_, geometry_[index], static_cast<int>(index), marker_color));
    }
    result.markers.push_back(wireframeMarker(config_, config_.front_x, 100, "current_box_wireframe",
                                             color(1.0F, 1.0F, 1.0F, 1.0F), 0.014));

    BoxGeometry target{ "target_object_display_only", config_.target_size, config_.target_center };
    result.markers.push_back(cubeMarker(config_, target, 101, color(1.0F, 0.82F, 0.05F, 0.9F)));
    result.markers.back().ns = "top_open_target_display_only";

    result.markers.push_back(arrowMarker(
      config_, 104, point(config_.centerX(), config_.center_y, config_.topZ() + 0.02),
      point(config_.centerX(), config_.center_y, config_.topZ() + 0.24),
      color(0.1F, 1.0F, 0.2F, 1.0F)));
    result.markers.push_back(arrowMarker(
      config_, 105, point(config_.front_x - config_.wall_thickness - 0.02, config_.center_y, config_.centerZ()),
      point(config_.front_x - config_.wall_thickness - 0.30, config_.center_y, config_.centerZ()),
      color(1.0F, 0.2F, 0.15F, 1.0F)));
    result.markers.push_back(textMarker(config_));

    const geometry_msgs::msg::Point box_center = point(config_.centerX(), config_.center_y, config_.centerZ());
    result.markers.push_back(arrowMarker(config_, 106, box_center,
      point(config_.centerX() + 0.20, config_.center_y, config_.centerZ()), color(1.0F, 0.0F, 0.0F, 1.0F)));
    result.markers.push_back(arrowMarker(config_, 107, box_center,
      point(config_.centerX(), config_.center_y + 0.20, config_.centerZ()), color(0.0F, 1.0F, 0.0F, 1.0F)));
    result.markers.push_back(arrowMarker(config_, 108, box_center,
      point(config_.centerX(), config_.center_y, config_.centerZ() + 0.20), color(0.0F, 0.35F, 1.0F, 1.0F)));
    result.markers.push_back(labelMarker(config_, 109, "box_center_label", "BOX CENTER",
      { config_.centerX(), config_.center_y, config_.centerZ() + 0.24 }, color(1.0F, 1.0F, 1.0F, 1.0F), 0.085));

    const auto stamp = node_->now();
    for (auto& marker : result.markers)
      marker.header.stamp = stamp;
    return result;
  }

  void publishAll()
  {
    marker_publisher_->publish(makeMarkers());

    moveit_msgs::msg::PlanningScene scene_message;
    scene_message.is_diff = true;
    scene_message.world.collision_objects = collision_objects_;
    scene_publisher_->publish(scene_message);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = node_->now();
    joint_state.name = robot_model_->getVariableNames();
    const double* positions = state_->getVariablePositions();
    joint_state.position.assign(positions, positions + joint_state.name.size());
    joint_state_publisher_->publish(joint_state);
  }

  void writeStaticCollisionAudit(const std::string& output_path) const
  {
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;
    collision_detection::CollisionResult result;
    local_scene_->checkCollision(request, result, *state_);

    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
    std::ofstream output(output_path, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write static collision CSV: " + output_path);
    output << "timestamp,scene_id,state_description,robot_box_collision,robot_link,box_object,contact_count,"
              "maximum_penetration_depth_m\n";

    bool found_environment_collision = false;
    for (const auto& entry : result.contacts)
    {
      const bool first_is_box = box_ids_.count(entry.first.first) != 0;
      const bool second_is_box = box_ids_.count(entry.first.second) != 0;
      if (first_is_box == second_is_box)
        continue;
      found_environment_collision = true;
      const std::string& box = first_is_box ? entry.first.first : entry.first.second;
      const std::string& robot_link = first_is_box ? entry.first.second : entry.first.first;
      double max_depth = 0.0;
      for (const auto& contact : entry.second)
        max_depth = std::max(max_depth, contact.depth);
      output << timestampNow() << ',' << config_.scene_id
             << ",lift=0;yaw=0;pitch=0;fingers=0.044,true," << robot_link << ',' << box << ','
             << entry.second.size() << ',' << std::setprecision(12) << max_depth << '\n';
    }
    if (!found_environment_collision)
    {
      output << timestampNow() << ',' << config_.scene_id
             << ",lift=0;yaw=0;pitch=0;fingers=0.044,false,NONE,NONE,0,0\n";
    }
    output.flush();
    if (!output)
      throw std::runtime_error("Failed while writing static collision CSV: " + output_path);

    RCLCPP_INFO(node_->get_logger(), "Static robot-box collision: %s; CSV: %s",
                found_environment_collision ? "DETECTED" : "NONE", output_path.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  GeometryConfig config_;
  std::vector<BoxGeometry> geometry_;
  std::map<std::string, Bounds> visual_bounds_;
  std::map<std::string, Bounds> collision_bounds_;
  Bounds current_box_outer_;
  Bounds current_box_inner_;
  Bounds fixed_body_combined_;
  Bounds amr_combined_;
  Bounds waist_combined_;
  double robot_fixed_front_x_{};
  double left_reach_{};
  double right_reach_{};
  std::vector<CandidatePlacement> candidates_;
  std::set<std::string> box_ids_;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr state_;
  planning_scene::PlanningScenePtr local_scene_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<rclcpp::Node>(
      "top_open_box_geometry_publisher",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto publisher = std::make_shared<TopOpenBoxGeometryPublisher>(node);
    publisher->initialize();
    rclcpp::spin(node);
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("top_open_box_geometry_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
