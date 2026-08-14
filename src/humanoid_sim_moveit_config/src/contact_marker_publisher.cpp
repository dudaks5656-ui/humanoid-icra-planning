#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using Pair = std::pair<std::string, std::string>;

Pair orderedPair(const std::string& first, const std::string& second)
{
  return first < second ? Pair(first, second) : Pair(second, first);
}

std_msgs::msg::ColorRGBA pairColor(const Pair& pair)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0F;
  if (pair.first == "waist_pitch_link" || pair.second == "waist_pitch_link")
  {
    color.r = 1.0F;
    color.g = 0.15F;
    color.b = 0.05F;
  }
  else if (pair.first.find("left") != std::string::npos)
  {
    color.r = 1.0F;
    color.g = 0.65F;
    color.b = 0.0F;
  }
  else
  {
    color.r = 0.2F;
    color.g = 0.65F;
    color.b = 1.0F;
  }
  return color;
}
}  // namespace

class ContactMarkerPublisher
{
public:
  explicit ContactMarkerPublisher(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", false);
    model_ = loader->getModel();
    if (!model_)
      throw std::runtime_error("Robot model could not be loaded");
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    state_ = std::make_shared<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();
    state_->update();

    target_pairs_ = {
      orderedPair("waist_pitch_link", "openarm_left_link0"),
      orderedPair("openarm_left_left_finger", "openarm_left_right_finger"),
      orderedPair("openarm_right_left_finger", "openarm_right_right_finger"),
    };

    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/permanent_contact_markers", rclcpp::QoS(1).transient_local().reliable());
    joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::JointState::SharedPtr message) { updateState(*message); });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), [this]() { publishMarkers(); });
    RCLCPP_INFO(node_->get_logger(), "Contact marker publisher ready on /permanent_contact_markers (FCL)");
  }

private:
  void updateState(const sensor_msgs::msg::JointState& message)
  {
    if (message.name.size() != message.position.size())
      return;
    std::scoped_lock lock(mutex_);
    for (std::size_t index = 0; index < message.name.size(); ++index)
    {
      const auto& variables = model_->getVariableNames();
      if (std::find(variables.begin(), variables.end(), message.name[index]) != variables.end())
        state_->setVariablePosition(message.name[index], message.position[index]);
    }
    state_->update();
  }

  void publishMarkers()
  {
    std::scoped_lock lock(mutex_);
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 100;
    request.max_contacts_per_pair = 20;
    collision_detection::CollisionResult result;
    scene_->checkSelfCollision(request, result, *state_);

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    int id = 0;

    for (const auto& entry : result.contacts)
    {
      const Pair pair = orderedPair(entry.first.first, entry.first.second);
      if (target_pairs_.count(pair) == 0)
        continue;
      const auto color = pairColor(pair);
      for (const auto& contact : entry.second)
      {
        visualization_msgs::msg::Marker point;
        point.header.frame_id = "world";
        point.header.stamp = node_->now();
        point.ns = "contact_points";
        point.id = id++;
        point.type = visualization_msgs::msg::Marker::SPHERE;
        point.action = visualization_msgs::msg::Marker::ADD;
        point.pose.position.x = contact.pos.x();
        point.pose.position.y = contact.pos.y();
        point.pose.position.z = contact.pos.z();
        point.pose.orientation.w = 1.0;
        point.scale.x = point.scale.y = point.scale.z = 0.018;
        point.color = color;
        array.markers.push_back(point);

        visualization_msgs::msg::Marker normal;
        normal.header = point.header;
        normal.ns = "contact_normals";
        normal.id = id++;
        normal.type = visualization_msgs::msg::Marker::ARROW;
        normal.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point start;
        start.x = contact.pos.x();
        start.y = contact.pos.y();
        start.z = contact.pos.z();
        geometry_msgs::msg::Point finish = start;
        finish.x += contact.normal.x() * 0.05;
        finish.y += contact.normal.y() * 0.05;
        finish.z += contact.normal.z() * 0.05;
        normal.points = { start, finish };
        normal.scale.x = 0.006;
        normal.scale.y = 0.012;
        normal.scale.z = 0.0;
        normal.color = color;
        array.markers.push_back(normal);

        visualization_msgs::msg::Marker label;
        label.header = point.header;
        label.ns = "contact_labels";
        label.id = id++;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position = point.pose.position;
        label.pose.position.z += 0.03;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.025;
        label.color = color;
        label.text = pair.first + " / " + pair.second + " depth=" + std::to_string(contact.depth) + " m";
        array.markers.push_back(label);
      }
    }
    marker_publisher_->publish(array);
  }

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScenePtr scene_;
  moveit::core::RobotStatePtr state_;
  std::set<Pair> target_pairs_;
  std::mutex mutex_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("contact_marker_publisher", options);
    auto publisher = std::make_shared<ContactMarkerPublisher>(node);
    rclcpp::spin(node);
  }
  catch (const std::exception& error)
  {
    std::cerr << "contact_marker_publisher fatal: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
