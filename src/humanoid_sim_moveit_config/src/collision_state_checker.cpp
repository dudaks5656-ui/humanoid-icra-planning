#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
std::string csvEscape(const std::string& value)
{
  std::string escaped = "\"";
  for (const char c : value)
  {
    if (c == '"')
      escaped += "\"\"";
    else
      escaped += c;
  }
  escaped += '"';
  return escaped;
}

std::string pairSetToString(const std::set<std::pair<std::string, std::string>>& pairs)
{
  std::ostringstream out;
  bool first = true;
  for (const auto& pair : pairs)
  {
    if (!first)
      out << ';';
    out << pair.first << "|" << pair.second;
    first = false;
  }
  return out.str();
}

std::string stateToString(const moveit::core::RobotState& state, const std::vector<std::string>& names)
{
  std::ostringstream out;
  out << std::setprecision(15);
  for (std::size_t index = 0; index < names.size(); ++index)
  {
    if (index != 0)
      out << ';';
    out << names[index] << '=' << state.getVariablePosition(names[index]);
  }
  return out.str();
}

struct CheckResult
{
  bool joint_limit_valid{ false };
  bool self_collision{ false };
  bool environment_collision{ false };
  std::set<std::pair<std::string, std::string>> collision_pairs;
  double minimum_distance{ std::numeric_limits<double>::infinity() };
  double elapsed_ms{ 0.0 };
};
}  // namespace

class CollisionStateChecker
{
public:
  explicit CollisionStateChecker(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    declareIfMissing<std::string>("output_csv", "/home/openarm/humanoid_sim_ws/validation/collision_results.csv");
    declareIfMissing<std::string>("trajectory_output_csv",
                                  "/home/openarm/humanoid_sim_ws/validation/trajectory_validation.csv");
    declareIfMissing<std::string>("input_topic", "/collision_state_input");
    declareIfMissing<std::string>("trajectory_topic", "/display_planned_path");
    declareIfMissing<std::string>("sampling_method", "external_full_robot_state");
    declareIfMissing<std::string>("target_pose", "");
    declareIfMissing<std::string>("collision_detector_name", "FCL");
    declareIfMissing<int>("coarse_subdivisions", 1);
    declareIfMissing<int>("medium_subdivisions", 5);
    declareIfMissing<int>("fine_subdivisions", 20);

    output_csv_ = node_->get_parameter("output_csv").as_string();
    trajectory_output_csv_ = node_->get_parameter("trajectory_output_csv").as_string();
    default_sampling_method_ = node_->get_parameter("sampling_method").as_string();
    target_pose_ = node_->get_parameter("target_pose").as_string();

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", false);
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_)
      throw std::runtime_error("RobotModelLoader failed to load robot_description and robot_description_semantic");

    whole_body_group_ = robot_model_->getJointModelGroup("whole_body");
    if (!whole_body_group_)
      throw std::runtime_error("SRDF group 'whole_body' is missing");
    independent_joint_names_ = whole_body_group_->getVariableNames();
    if (independent_joint_names_.size() != 19)
      throw std::runtime_error("whole_body must contain exactly 19 independent variables; got " +
                               std::to_string(independent_joint_names_.size()));

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, robot_model_loader_, "collision_state_checker_scene_monitor");
    if (!planning_scene_monitor_->getPlanningScene())
      throw std::runtime_error("PlanningSceneMonitor failed to create a PlanningScene");
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");

    initializeCsvFiles();

    state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        node_->get_parameter("input_topic").as_string(), rclcpp::QoS(10),
        [this](const sensor_msgs::msg::JointState::SharedPtr message) { checkJointStateMessage(*message); });
    trajectory_subscription_ = node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
        node_->get_parameter("trajectory_topic").as_string(), rclcpp::QoS(10),
        [this](const moveit_msgs::msg::DisplayTrajectory::SharedPtr message) { validateTrajectoryMessage(*message); });

    RCLCPP_INFO(node_->get_logger(),
                "collision_state_checker ready: detector=%s independent_variables=%zu execution=disabled",
                node_->get_parameter("collision_detector_name").as_string().c_str(), independent_joint_names_.size());
  }

private:
  template <typename T>
  void declareIfMissing(const std::string& name, const T& value)
  {
    if (!node_->has_parameter(name))
      node_->declare_parameter<T>(name, value);
  }

  void initializeCsvFiles()
  {
    {
      std::ofstream output(output_csv_, std::ios::app);
      if (!output)
        throw std::runtime_error("Cannot open collision CSV: " + output_csv_);
      if (output.tellp() == 0)
      {
        output << "timestamp,sample_id,sampling_method,all_joint_values,target_pose,joint_limit_valid,"
                  "self_collision,environment_collision,colliding_link_pairs,minimum_distance,"
                  "collision_check_time_ms,planning_success,trajectory_valid\n";
      }
    }
    {
      std::ofstream output(trajectory_output_csv_, std::ios::app);
      if (!output)
        throw std::runtime_error("Cannot open trajectory CSV: " + trajectory_output_csv_);
      if (output.tellp() == 0)
      {
        output << "trajectory_id,resolution,source_waypoint_index,interpolation_index,all_joint_values,"
                  "joint_limit_valid,self_collision,environment_collision,colliding_link_pairs,"
                  "minimum_distance,collision_check_time_ms,trajectory_valid\n";
      }
    }
  }

  CheckResult checkState(moveit::core::RobotState& state) const
  {
    const auto begin = std::chrono::steady_clock::now();
    state.update();

    CheckResult result;
    result.joint_limit_valid = state.satisfiesBounds(whole_body_group_);

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 10;
    request.distance = true;
    request.detailed_distance = true;

    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    collision_detection::CollisionResult self_result;
    scene->checkSelfCollision(request, self_result, state);
    result.self_collision = self_result.collision;

    collision_detection::CollisionResult full_result;
    scene->checkCollision(request, full_result, state);

    for (const auto& entry : self_result.contacts)
      result.collision_pairs.insert(entry.first);

    for (const auto& entry : full_result.contacts)
    {
      result.collision_pairs.insert(entry.first);
      for (const auto& contact : entry.second)
      {
        if (contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT ||
            contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
          result.environment_collision = true;
      }
    }

    result.minimum_distance = std::min(self_result.distance, full_result.distance);
    if (!std::isfinite(result.minimum_distance))
      result.minimum_distance = std::numeric_limits<double>::quiet_NaN();

    const auto end = std::chrono::steady_clock::now();
    result.elapsed_ms = std::chrono::duration<double, std::milli>(end - begin).count();
    return result;
  }

  void checkJointStateMessage(const sensor_msgs::msg::JointState& message)
  {
    if (message.name.size() != message.position.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "JointState names and positions have different sizes");
      return;
    }

    std::map<std::string, double> values;
    for (std::size_t index = 0; index < message.name.size(); ++index)
      values[message.name[index]] = message.position[index];

    std::vector<std::string> missing;
    for (const auto& name : independent_joint_names_)
      if (values.count(name) == 0)
        missing.push_back(name);
    if (!missing.empty())
    {
      std::ostringstream names;
      for (const auto& name : missing)
        names << name << ' ';
      RCLCPP_ERROR(node_->get_logger(), "Rejected incomplete RobotState; missing: %s", names.str().c_str());
      return;
    }

    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    for (const auto& name : independent_joint_names_)
      state.setVariablePosition(name, values.at(name));

    const CheckResult result = checkState(state);
    const std::string method = message.header.frame_id.empty() ? default_sampling_method_ : message.header.frame_id;
    const std::uint64_t id = ++sample_counter_;
    appendCollisionCsv(id, method, target_pose_, state, result, false,
                       result.joint_limit_valid && !result.self_collision && !result.environment_collision);
    RCLCPP_INFO(node_->get_logger(),
                "sample=%lu limits=%s self=%s environment=%s pairs=%zu min_distance=%.6f time_ms=%.3f",
                static_cast<unsigned long>(id), result.joint_limit_valid ? "valid" : "invalid",
                result.self_collision ? "collision" : "clear",
                result.environment_collision ? "collision" : "clear", result.collision_pairs.size(),
                result.minimum_distance, result.elapsed_ms);
  }

  void appendCollisionCsv(std::uint64_t id, const std::string& sampling_method, const std::string& target_pose,
                          const moveit::core::RobotState& state, const CheckResult& result, bool planning_success,
                          bool trajectory_valid)
  {
    std::ofstream output(output_csv_, std::ios::app);
    output << std::fixed << std::setprecision(9) << node_->now().seconds() << ',' << id << ','
           << csvEscape(sampling_method) << ',' << csvEscape(stateToString(state, independent_joint_names_)) << ','
           << csvEscape(target_pose) << ',' << (result.joint_limit_valid ? 1 : 0) << ','
           << (result.self_collision ? 1 : 0) << ',' << (result.environment_collision ? 1 : 0) << ','
           << csvEscape(pairSetToString(result.collision_pairs)) << ',' << result.minimum_distance << ','
           << result.elapsed_ms << ',' << (planning_success ? 1 : 0) << ',' << (trajectory_valid ? 1 : 0) << '\n';
  }

  struct TrajectoryRow
  {
    std::size_t source_waypoint{ 0 };
    int interpolation_index{ 0 };
    moveit::core::RobotState state;
    CheckResult result;

    explicit TrajectoryRow(const moveit::core::RobotModelConstPtr& model) : state(model)
    {
    }
  };

  void validateTrajectoryAtResolution(std::uint64_t trajectory_id, const std::string& resolution, int subdivisions,
                                      const moveit::core::RobotState& start,
                                      const trajectory_msgs::msg::JointTrajectory& trajectory)
  {
    subdivisions = std::max(1, subdivisions);
    moveit::core::RobotState previous = start;
    previous.update();
    std::vector<TrajectoryRow> rows;
    bool trajectory_valid = true;

    for (std::size_t waypoint = 0; waypoint < trajectory.points.size(); ++waypoint)
    {
      const auto& point = trajectory.points[waypoint];
      if (point.positions.size() != trajectory.joint_names.size())
      {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory waypoint has inconsistent joint vector size");
        return;
      }
      moveit::core::RobotState current = previous;
      current.setVariablePositions(trajectory.joint_names, point.positions);
      current.update();

      for (int interpolation = 1; interpolation <= subdivisions; ++interpolation)
      {
        TrajectoryRow row(robot_model_);
        row.source_waypoint = waypoint;
        row.interpolation_index = interpolation;
        previous.interpolate(current, static_cast<double>(interpolation) / subdivisions, row.state);
        row.result = checkState(row.state);
        const bool valid = row.result.joint_limit_valid && !row.result.self_collision &&
                           !row.result.environment_collision;
        trajectory_valid = trajectory_valid && valid;
        rows.push_back(std::move(row));
      }
      previous = current;
    }

    std::ofstream output(trajectory_output_csv_, std::ios::app);
    for (const auto& row : rows)
    {
      output << trajectory_id << ',' << csvEscape(resolution) << ',' << row.source_waypoint << ','
             << row.interpolation_index << ',' << csvEscape(stateToString(row.state, independent_joint_names_)) << ','
             << (row.result.joint_limit_valid ? 1 : 0) << ',' << (row.result.self_collision ? 1 : 0) << ','
             << (row.result.environment_collision ? 1 : 0) << ','
             << csvEscape(pairSetToString(row.result.collision_pairs)) << ',' << row.result.minimum_distance << ','
             << row.result.elapsed_ms << ',' << (trajectory_valid ? 1 : 0) << '\n';
    }

    RCLCPP_INFO(node_->get_logger(), "trajectory=%lu resolution=%s checked_states=%zu valid=%s",
                static_cast<unsigned long>(trajectory_id), resolution.c_str(), rows.size(),
                trajectory_valid ? "true" : "false");
  }

  void validateTrajectoryMessage(const moveit_msgs::msg::DisplayTrajectory& message)
  {
    moveit::core::RobotState start(robot_model_);
    start.setToDefaultValues();
    if (!moveit::core::robotStateMsgToRobotState(message.trajectory_start, start))
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not decode DisplayTrajectory start state");
      return;
    }

    for (const auto& robot_trajectory : message.trajectory)
    {
      const std::uint64_t id = ++trajectory_counter_;
      validateTrajectoryAtResolution(id, "coarse", node_->get_parameter("coarse_subdivisions").as_int(), start,
                                     robot_trajectory.joint_trajectory);
      validateTrajectoryAtResolution(id, "medium", node_->get_parameter("medium_subdivisions").as_int(), start,
                                     robot_trajectory.joint_trajectory);
      validateTrajectoryAtResolution(id, "fine", node_->get_parameter("fine_subdivisions").as_int(), start,
                                     robot_trajectory.joint_trajectory);
    }
  }

  rclcpp::Node::SharedPtr node_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* whole_body_group_{ nullptr };
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::vector<std::string> independent_joint_names_;
  std::string output_csv_;
  std::string trajectory_output_csv_;
  std::string default_sampling_method_;
  std::string target_pose_;
  std::uint64_t sample_counter_{ 0 };
  std::uint64_t trajectory_counter_{ 0 };
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("collision_state_checker", options);
    auto checker = std::make_shared<CollisionStateChecker>(node);
    rclcpp::spin(node);
  }
  catch (const std::exception& error)
  {
    std::cerr << "collision_state_checker fatal: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
