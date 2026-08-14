#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <urdf_model/link.h>

namespace
{
using Pair = std::pair<std::string, std::string>;

Pair orderedPair(const std::string& first, const std::string& second)
{
  return first < second ? Pair(first, second) : Pair(second, first);
}

std::string csvEscape(const std::string& value)
{
  std::string result = "\"";
  for (const char character : value)
    result += character == '"' ? "\"\"" : std::string(1, character);
  result += '"';
  return result;
}

std::string vectorString(const Eigen::Vector3d& value)
{
  std::ostringstream output;
  output << std::setprecision(15) << value.x() << ' ' << value.y() << ' ' << value.z();
  return output.str();
}

std::string poseString(const Eigen::Isometry3d& pose)
{
  const Eigen::Vector3d rpy = pose.rotation().eulerAngles(0, 1, 2);
  return "xyz=" + vectorString(pose.translation()) + " rpy=" + vectorString(rpy);
}

struct Aabb
{
  Eigen::Vector3d minimum{ Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()) };
  Eigen::Vector3d maximum{ Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity()) };
  bool valid{ false };
};

void extendAabb(Aabb& aabb, const Eigen::Vector3d& point)
{
  aabb.minimum = aabb.minimum.cwiseMin(point);
  aabb.maximum = aabb.maximum.cwiseMax(point);
  aabb.valid = true;
}

Aabb linkAabb(const moveit::core::RobotState& state, const std::string& link_name,
              const Eigen::Isometry3d& reference_from_world = Eigen::Isometry3d::Identity())
{
  Aabb result;
  const auto* link = state.getRobotModel()->getLinkModel(link_name);
  if (!link)
    return result;
  const auto& shapes = link->getShapes();
  const auto& origins = link->getCollisionOriginTransforms();
  for (std::size_t shape_index = 0; shape_index < shapes.size(); ++shape_index)
  {
    const Eigen::Isometry3d transform =
        reference_from_world * state.getGlobalLinkTransform(link) * origins.at(shape_index);
    const auto& shape = shapes.at(shape_index);
    if (shape->type == shapes::MESH)
    {
      const auto* mesh = static_cast<const shapes::Mesh*>(shape.get());
      for (unsigned int vertex = 0; vertex < mesh->vertex_count; ++vertex)
      {
        const Eigen::Vector3d point(mesh->vertices[3 * vertex], mesh->vertices[3 * vertex + 1],
                                    mesh->vertices[3 * vertex + 2]);
        extendAabb(result, transform * point);
      }
    }
    else
    {
      const Eigen::Vector3d half = shapes::computeShapeExtents(shape.get()) * 0.5;
      for (int x : { -1, 1 })
        for (int y : { -1, 1 })
          for (int z : { -1, 1 })
            extendAabb(result, transform * Eigen::Vector3d(x * half.x(), y * half.y(), z * half.z()));
    }
  }
  return result;
}

std::string aabbString(const Aabb& aabb)
{
  if (!aabb.valid)
    return "NONE";
  return "min=" + vectorString(aabb.minimum) + " max=" + vectorString(aabb.maximum);
}

std::string stateString(const moveit::core::RobotState& state, const std::vector<std::string>& names)
{
  std::ostringstream output;
  output << std::setprecision(15);
  for (std::size_t index = 0; index < names.size(); ++index)
  {
    if (index != 0)
      output << ';';
    output << names[index] << '=' << state.getVariablePosition(names[index]);
  }
  return output.str();
}

struct DiagnosticState
{
  std::string name;
  std::map<std::string, double> values;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("permanent_contact_analyzer", options);

  try
  {
    if (!node->has_parameter("comparison_output"))
      node->declare_parameter<std::string>(
          "comparison_output", "/home/openarm/humanoid_sim_ws/validation/arm_mount_collision_comparison.txt");
    if (!node->has_parameter("contact_output"))
      node->declare_parameter<std::string>(
          "contact_output", "/home/openarm/humanoid_sim_ws/validation/permanent_contact_analysis.csv");

    auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description", false);
    const auto model = loader->getModel();
    if (!model)
      throw std::runtime_error("Robot model could not be loaded");
    const auto* whole_body = model->getJointModelGroup("whole_body");
    if (!whole_body || whole_body->getVariableNames().size() != 19)
      throw std::runtime_error("Expected whole_body with 19 independent variables");

    planning_scene::PlanningScene scene(model);
    moveit::core::RobotState zero(model);
    zero.setToDefaultValues();
    zero.update();
    const Eigen::Isometry3d pitch_from_world = zero.getGlobalLinkTransform("waist_pitch_link").inverse();

    std::ofstream comparison(node->get_parameter("comparison_output").as_string());
    if (!comparison)
      throw std::runtime_error("Cannot open mount comparison output");
    comparison << "ARM_MOUNT_COLLISION_COMPARISON\n";
    comparison << "reference_state=all_19_independent_variables_zero\n";
    comparison << "aabb_units=m\n\n";

    const auto urdf_model = loader->getURDF();
    for (const std::string side : { "left", "right" })
    {
      const std::string mount_joint = "waist_pitch_to_" + side + "_arm_mount_joint";
      const std::string adapter_joint = side + "_arm_mount_to_openarm_joint";
      const std::string link0 = "openarm_" + side + "_link0";
      const auto* link_model = model->getLinkModel(link0);
      const auto* mount_link_model = model->getLinkModel(side + "_arm_mount_link");
      comparison << '[' << side << "]\n";
      comparison << "mount_joint=" << mount_joint << ' ' << poseString(mount_link_model->getJointOriginTransform())
                 << "\n";
      comparison << "adapter_joint=" << adapter_joint << ' '
                 << poseString(link_model->getJointOriginTransform()) << "\n";
      comparison << "pitch_frame_link0_pose="
                 << poseString(pitch_from_world * zero.getGlobalLinkTransform(link0)) << "\n";
      if (!link_model->getCollisionOriginTransforms().empty())
        comparison << "link0_collision_origin=" << poseString(link_model->getCollisionOriginTransforms().front())
                   << "\n";
      const auto urdf_link = urdf_model->getLink(link0);
      if (urdf_link && urdf_link->collision && urdf_link->collision->geometry &&
          urdf_link->collision->geometry->type == urdf::Geometry::MESH)
      {
        const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(urdf_link->collision->geometry);
        comparison << "collision_mesh=" << mesh->filename << "\n";
        comparison << "collision_mesh_scale=" << mesh->scale.x << ' ' << mesh->scale.y << ' ' << mesh->scale.z
                   << "\n";
      }
      comparison << "collision_aabb_world=" << aabbString(linkAabb(zero, link0)) << "\n";
      comparison << "collision_aabb_pitch=" << aabbString(linkAabb(zero, link0, pitch_from_world)) << "\n\n";
    }
    comparison << "waist_pitch_aabb_world=" << aabbString(linkAabb(zero, "waist_pitch_link")) << "\n";
    comparison << "waist_pitch_aabb_pitch="
               << aabbString(linkAabb(zero, "waist_pitch_link", pitch_from_world)) << "\n";
    comparison << "STRUCTURAL_CHAIN_LEFT=fixed_via_left_arm_mount_link\n";
    comparison << "STRUCTURAL_CHAIN_RIGHT=fixed_via_right_arm_mount_link\n";
    comparison << "MOVABLE_JOINT_BETWEEN_PITCH_AND_LINK0=false_for_both_sides\n";
    comparison.close();

    constexpr double degrees_to_radians = M_PI / 180.0;
    const std::vector<DiagnosticState> diagnostic_states = {
      { "all_zero", {} },
      { "yaw_minus_10_deg", { { "waist_yaw_joint", -10.0 * degrees_to_radians } } },
      { "yaw_zero_deg", { { "waist_yaw_joint", 0.0 } } },
      { "yaw_plus_10_deg", { { "waist_yaw_joint", 10.0 * degrees_to_radians } } },
      { "pitch_minus_10_deg", { { "waist_pitch_joint", -10.0 * degrees_to_radians } } },
      { "pitch_zero_deg", { { "waist_pitch_joint", 0.0 } } },
      { "pitch_plus_45_deg", { { "waist_pitch_joint", 45.0 * degrees_to_radians } } },
      { "left_joint1_plus_5_deg_diagnostic", { { "openarm_left_joint1", 5.0 * degrees_to_radians } } },
      { "right_joint1_minus_5_deg_diagnostic", { { "openarm_right_joint1", -5.0 * degrees_to_radians } } },
      { "symmetric_joint1_diagnostic",
        { { "openarm_left_joint1", 5.0 * degrees_to_radians },
          { "openarm_right_joint1", -5.0 * degrees_to_radians } } },
    };
    const std::vector<Pair> target_pairs = {
      orderedPair("waist_pitch_link", "openarm_left_link0"),
      orderedPair("openarm_left_left_finger", "openarm_left_right_finger"),
      orderedPair("openarm_right_left_finger", "openarm_right_right_finger"),
    };

    std::ofstream contacts(node->get_parameter("contact_output").as_string());
    if (!contacts)
      throw std::runtime_error("Cannot open contact analysis CSV");
    contacts << "state_id,pair,contact_count,maximum_penetration_depth_m,deepest_contact_position_world_m,"
                "deepest_contact_normal_world,link1_aabb_world_m,link2_aabb_world_m,all_joint_values\n";

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 30000;
    request.max_contacts_per_pair = 10000;

    for (const auto& diagnostic : diagnostic_states)
    {
      moveit::core::RobotState state = zero;
      for (const auto& value : diagnostic.values)
        state.setVariablePosition(value.first, value.second);
      state.update();
      collision_detection::CollisionResult result;
      scene.checkSelfCollision(request, result, state);

      for (const auto& target : target_pairs)
      {
        std::size_t count = 0;
        double maximum_depth = 0.0;
        Eigen::Vector3d deepest_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d deepest_normal = Eigen::Vector3d::Zero();
        for (const auto& entry : result.contacts)
        {
          if (orderedPair(entry.first.first, entry.first.second) != target)
            continue;
          for (const auto& contact : entry.second)
          {
            if (contact.depth >= maximum_depth)
            {
              maximum_depth = contact.depth;
              deepest_position = contact.pos;
              deepest_normal = contact.normal;
            }
            ++count;
          }
        }
        contacts << csvEscape(diagnostic.name) << ',' << csvEscape(target.first + "|" + target.second) << ',' << count
                 << ',' << std::setprecision(15) << maximum_depth << ','
                 << csvEscape(count == 0 ? "" : vectorString(deepest_position)) << ','
                 << csvEscape(count == 0 ? "" : vectorString(deepest_normal)) << ','
                 << csvEscape(aabbString(linkAabb(state, target.first))) << ','
                 << csvEscape(aabbString(linkAabb(state, target.second))) << ','
                 << csvEscape(stateString(state, whole_body->getVariableNames())) << '\n';
      }
    }
    contacts.close();

    RCLCPP_INFO(node->get_logger(), "Wrote mount comparison and %zu contact-state rows",
                diagnostic_states.size() * target_pairs.size());
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Permanent contact analysis failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
