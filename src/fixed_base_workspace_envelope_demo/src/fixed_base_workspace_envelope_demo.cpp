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
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace fixed_base_workspace_envelope_demo
{
using Clock = std::chrono::steady_clock;
using Index = std::tuple<int, int, int>;
using CsvRow = std::unordered_map<std::string, std::string>;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::vector<std::string> parseCsvLine(const std::string& line)
{
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i)
  {
    const char c = line[i];
    if (quoted)
    {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') { field.push_back('"'); ++i; }
      else if (c == '"') quoted = false;
      else field.push_back(c);
    }
    else if (c == '"') quoted = true;
    else if (c == ',') { fields.push_back(field); field.clear(); }
    else field.push_back(c);
  }
  fields.push_back(field);
  return fields;
}

std::vector<CsvRow> readCsv(const std::string& path)
{
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read CSV: " + path);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("Empty CSV: " + path);
  if (!line.empty() && line.back() == '\r') line.pop_back();
  const auto header = parseCsvLine(line);
  std::vector<CsvRow> rows;
  while (std::getline(input, line))
  {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    const auto values = parseCsvLine(line);
    if (values.size() != header.size()) throw std::runtime_error("CSV field mismatch: " + path);
    CsvRow row;
    for (std::size_t i = 0; i < header.size(); ++i) row.emplace(header[i], values[i]);
    rows.push_back(std::move(row));
  }
  return rows;
}

std::string text(const CsvRow& row, const std::string& key)
{
  const auto found = row.find(key);
  if (found == row.end()) throw std::runtime_error("Missing CSV field: " + key);
  return found->second;
}

double number(const CsvRow& row, const std::string& key)
{
  const auto value = text(row, key);
  if (value.empty()) return kNaN;
  const double parsed = std::stod(value);
  return std::isfinite(parsed) ? parsed : kNaN;
}

int integer(const CsvRow& row, const std::string& key) { return std::stoi(text(row, key)); }

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA value;
  value.r = r; value.g = g; value.b = b; value.a = a;
  return value;
}

double halton(std::size_t index, int base)
{
  double fraction = 1.0;
  double result = 0.0;
  while (index)
  {
    fraction /= static_cast<double>(base);
    result += fraction * static_cast<double>(index % static_cast<std::size_t>(base));
    index /= static_cast<std::size_t>(base);
  }
  return result;
}

struct Point
{
  int id{};
  Eigen::Vector3d xyz{ Eigen::Vector3d::Zero() };
  std::array<bool, 4> reachable{};
  std::string classification;
};

struct Summary
{
  std::string name;
  int reachable{};
  double volume{};
  double increase{};
};

struct RawPose
{
  bool success{};
  double lift{ kNaN };
  double yaw{ kNaN };
  double pitch{ kNaN };
};

struct Scene
{
  std::string name;
  double duration{};
};

class EnvelopeDemo : public rclcpp::Node
{
public:
  EnvelopeDemo()
    : Node("fixed_base_workspace_envelope_demo",
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    loadParameters();
    requireSafetyContract();
    loadValidatedData();
    restoreGrid();
    reconstructOccupancy();
    loadRobotModel();
    prepareRepresentativeStates();
    writeMetrics();
    buildTimeline();

    auto transient = rclcpp::QoS(1).reliable().transient_local();
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fixed_base_workspace_envelope_demo/markers", transient);
    state_publisher_ = create_publisher<moveit_msgs::msg::DisplayRobotState>(
      "/display_robot_state", transient);
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/fixed_base_workspace_envelope_demo/status", transient);

    start_time_ = Clock::now();
    current_scene_ = "INITIALIZING";
    writeRuntime(current_scene_, 0.0);
    if (!preflight_only_)
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_hz_),
                                 std::bind(&EnvelopeDemo::tick, this));
    else
    {
      current_scene_ = "PREFLIGHT_PASS";
      writeRuntime(current_scene_, 0.0);
      RCLCPP_INFO(get_logger(),
        "ENVELOPE PREFLIGHT PASS grid=%zux%zux%zu spacing=(%.12f,%.12f,%.12f) "
        "occupied=[%zu,%zu,%zu,%zu] faces=[%zu,%zu,%zu,%zu]",
        axes_[0].size(), axes_[1].size(), axes_[2].size(), spacing_.x(), spacing_.y(), spacing_.z(),
        occupancy_[0].size(), occupancy_[1].size(), occupancy_[2].size(), occupancy_[3].size(),
        exposed_faces_[0], exposed_faces_[1], exposed_faces_[2], exposed_faces_[3]);
    }
  }

  bool preflightOnly() const { return preflight_only_; }

private:
  template <typename T>
  T parameter(const std::string& name) const { return get_parameter(name).get_value<T>(); }

  void loadParameters()
  {
    base_frame_ = parameter<std::string>("base_frame");
    tcp_frame_ = parameter<std::string>("tcp_frame");
    arm_group_name_ = parameter<std::string>("arm_group");
    target_q_ = Eigen::Quaterniond(parameter<double>("target_qw"), parameter<double>("target_qx"),
                                   parameter<double>("target_qy"), parameter<double>("target_qz"));
    target_q_.normalize();
    orientation_tolerance_ = parameter<double>("orientation_tolerance_rad");
    publish_hz_ = parameter<double>("marker_publish_hz");
    point_diameter_ = parameter<double>("point_diameter");
    surface_alpha_ = parameter<double>("surface_alpha");
    voxel_alpha_ = parameter<double>("voxel_alpha");
    startup_delay_ = parameter<double>("startup_delay_s");
    duration_scale_ = parameter<double>("duration_scale");
    animation_samples_ = parameter<int>("animation_samples");
    demo_scene_ = parameter<std::string>("demo_scene");
    visualization_mode_ = parameter<std::string>("visualization_mode");
    preflight_only_ = parameter<bool>("preflight_only");
    comparison_csv_ = parameter<std::string>("comparison_csv");
    summary_csv_ = parameter<std::string>("summary_csv");
    contributions_csv_ = parameter<std::string>("contributions_csv");
    fine_points_csv_ = parameter<std::string>("fine_points_csv");
    new_points_csv_ = parameter<std::string>("new_points_csv");
    stored_c3_state_csv_ = parameter<std::string>("stored_c3_state_csv");
    runtime_json_ = parameter<std::string>("runtime_status_json");
    metrics_csv_ = parameter<std::string>("metrics_csv");
    source_hash_ = parameter<std::string>("source_comparison_sha256");
    source_summary_hash_ = parameter<std::string>("source_summary_sha256");
    source_contributions_hash_ = parameter<std::string>("source_contributions_sha256");

    scene_durations_["ROBOT"] = parameter<double>("robot_scene_s");
    scene_durations_["C0"] = parameter<double>("c0_scene_s");
    scene_durations_["C1"] = parameter<double>("c1_scene_s");
    scene_durations_["C2"] = parameter<double>("c2_scene_s");
    scene_durations_["C3"] = parameter<double>("c3_scene_s");
    scene_durations_["C0_VS_C3"] = parameter<double>("c0_vs_c3_scene_s");
    scene_durations_["COMBINED_ONLY"] = parameter<double>("combined_only_scene_s");
    scene_durations_["FINAL"] = parameter<double>("final_scene_s");
    if (!std::set<std::string>{ "points", "voxels", "surface" }.count(visualization_mode_))
      throw std::runtime_error("visualization_mode must be points, voxels, or surface");
    if (publish_hz_ <= 0 || duration_scale_ <= 0 || animation_samples_ < 2 || animation_samples_ > 501)
      throw std::runtime_error("Invalid timing or animation parameters");
  }

  void requireSafetyContract() const
  {
    for (const auto& name : { "trajectory_execution", "controller_enabled", "ros2_control_enabled",
                              "hardware_enabled", "amr_motion_enabled" })
      if (parameter<bool>(name)) throw std::runtime_error(std::string("Forbidden safety flag: ") + name);
  }

  void loadValidatedData()
  {
    for (const auto& row : readCsv(summary_csv_))
      summary_.push_back({ text(row, "configuration"), integer(row, "reachable_points"),
                           number(row, "targeted_workspace_volume"),
                           number(row, "percent_delta_vs_lift_only") });
    if (summary_.size() != 4) throw std::runtime_error("Expected four summary rows");
    const std::array<int, 4> expected{ 833, 1030, 976, 1119 };
    for (int i = 0; i < 4; ++i)
      if (summary_[static_cast<std::size_t>(i)].reachable != expected[static_cast<std::size_t>(i)])
        throw std::runtime_error("Validated summary count drift");

    const auto contribution = readCsv(contributions_csv_);
    if (contribution.size() != 1) throw std::runtime_error("Expected one contribution row");
    combined_only_count_ = integer(contribution.front(), "combined_torso_only_count");
    combined_only_volume_ = number(contribution.front(), "combined_torso_only_volume");

    const auto rows = readCsv(comparison_csv_);
    if (rows.size() != 1440) throw std::runtime_error("Expected immutable 1,440-point comparison");
    std::set<int> ids;
    for (const auto& row : rows)
    {
      Point point;
      point.id = integer(row, "point_id");
      point.xyz = { number(row, "tcp_x"), number(row, "tcp_y"), number(row, "tcp_z") };
      point.reachable = { integer(row, "c0_lift_success") == 1,
                          integer(row, "c1_lift_yaw_success") == 1,
                          integer(row, "c2_lift_pitch_success") == 1,
                          integer(row, "c3_lift_yaw_pitch_success") == 1 };
      point.classification = text(row, "classification");
      if (!ids.insert(point.id).second || !point.xyz.allFinite())
        throw std::runtime_error("Duplicate point ID or invalid coordinate");
      points_.push_back(point);
      if (point.id == 1360) representative_1360_ = point;
    }
    if (representative_1360_.id != 1360 || representative_1360_.reachable != std::array<bool,4>{false,false,false,true})
      throw std::runtime_error("Point 1360 is not validated combined-only evidence");

    auto add_raw = [&](const std::string& path) {
      for (const auto& row : readCsv(path))
      {
        const std::string config = text(row, "configuration");
        int index = -1;
        if (config == "LIFT_ONLY") index = 0;
        else if (config == "LIFT_YAW") index = 1;
        else if (config == "LIFT_PITCH") index = 2;
        else if (config == "LIFT_YAW_PITCH") index = 3;
        if (index < 0) continue;
        raw_[{ index, integer(row, "point_id") }] = {
          integer(row, "success") == 1, number(row, "selected_lift"),
          number(row, "selected_yaw"), number(row, "selected_pitch") };
      }
    };
    add_raw(fine_points_csv_);
    add_raw(new_points_csv_);
  }

  void restoreGrid()
  {
    for (const auto& point : points_)
      for (int axis = 0; axis < 3; ++axis) axes_[static_cast<std::size_t>(axis)].push_back(point.xyz[axis]);
    for (auto& axis : axes_)
    {
      std::sort(axis.begin(), axis.end());
      axis.erase(std::unique(axis.begin(), axis.end(), [](double a, double b) { return std::abs(a-b) < 1e-12; }), axis.end());
    }
    if (axes_[0].size() != 12 || axes_[1].size() != 10 || axes_[2].size() != 12)
      throw std::runtime_error("Grid dimensions differ from validated 12x10x12");
    for (int axis = 0; axis < 3; ++axis)
    {
      const auto& values = axes_[static_cast<std::size_t>(axis)];
      const double delta = values[1] - values[0];
      for (std::size_t i = 1; i < values.size(); ++i)
        if (std::abs((values[i] - values[i-1]) - delta) > 1e-11)
          throw std::runtime_error("Non-uniform validated grid spacing");
      spacing_[axis] = delta;
    }
    voxel_volume_ = spacing_.x() * spacing_.y() * spacing_.z();
  }

  int axisIndex(int axis, double value) const
  {
    const auto& values = axes_[static_cast<std::size_t>(axis)];
    const auto found = std::lower_bound(values.begin(), values.end(), value - 1e-12);
    if (found == values.end() || std::abs(*found - value) > 1e-10)
      throw std::runtime_error("Point not on reconstructed grid");
    return static_cast<int>(std::distance(values.begin(), found));
  }

  Index indexOf(const Point& point) const
  {
    return { axisIndex(0, point.xyz.x()), axisIndex(1, point.xyz.y()), axisIndex(2, point.xyz.z()) };
  }

  Eigen::Vector3d center(const Index& index) const
  {
    const auto [i,j,k] = index;
    return { axes_[0].at(static_cast<std::size_t>(i)), axes_[1].at(static_cast<std::size_t>(j)),
             axes_[2].at(static_cast<std::size_t>(k)) };
  }

  std::size_t countFaces(const std::set<Index>& occupied) const
  {
    static const std::array<Index, 6> neighbors{
      Index{1,0,0}, Index{-1,0,0}, Index{0,1,0}, Index{0,-1,0}, Index{0,0,1}, Index{0,0,-1} };
    std::size_t count = 0;
    for (const auto& index : occupied)
    {
      const auto [i,j,k] = index;
      for (const auto& delta : neighbors)
      {
        const auto [di,dj,dk] = delta;
        if (!occupied.count({i+di,j+dj,k+dk})) ++count;
      }
    }
    return count;
  }

  void reconstructOccupancy()
  {
    for (const auto& point : points_)
    {
      const auto index = indexOf(point);
      for (int configuration = 0; configuration < 4; ++configuration)
        if (point.reachable[static_cast<std::size_t>(configuration)])
          occupancy_[static_cast<std::size_t>(configuration)].insert(index);
      if (point.reachable[3] && !point.reachable[0]) c3_minus_c0_.insert(index);
      if (point.reachable[1] && !point.reachable[0]) yaw_expansion_.insert(index);
      if (point.reachable[2] && !point.reachable[0]) pitch_expansion_.insert(index);
      if (point.classification == "COMBINED_TORSO_ONLY") combined_only_.insert(index);
    }
    for (int configuration = 0; configuration < 4; ++configuration)
    {
      const auto index = static_cast<std::size_t>(configuration);
      if (occupancy_[index].size() != static_cast<std::size_t>(summary_[index].reachable))
        throw std::runtime_error("Occupied voxel count differs from validated reachability");
      const double reconstructed = static_cast<double>(occupancy_[index].size()) * voxel_volume_;
      if (std::abs(reconstructed - summary_[index].volume) > 1e-12)
        throw std::runtime_error("Reconstructed volume differs from validated volume");
      exposed_faces_[index] = countFaces(occupancy_[index]);
    }
    if (combined_only_.size() != static_cast<std::size_t>(combined_only_count_) ||
        std::abs(combined_only_.size() * voxel_volume_ - combined_only_volume_) > 1e-12)
      throw std::runtime_error("Combined-only occupancy/volume mismatch");
    combined_only_faces_ = countFaces(combined_only_);
    selectBoundaryPoints();
  }

  void selectBoundaryPoints()
  {
    for (int configuration = 0; configuration < 4; ++configuration)
    {
      std::vector<const Point*> candidates;
      for (const auto& point : points_) if (point.reachable[static_cast<std::size_t>(configuration)]) candidates.push_back(&point);
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const auto* point : candidates) mean += point->xyz;
      mean /= static_cast<double>(candidates.size());
      auto extreme = [&](auto score) {
        return *std::max_element(candidates.begin(), candidates.end(), [&](const Point* a, const Point* b) {
          return score(*a) < score(*b);
        });
      };
      const std::array<const Point*, 5> chosen{
        extreme([](const Point& p) { return p.xyz.x(); }),
        extreme([](const Point& p) { return p.xyz.y(); }),
        extreme([](const Point& p) { return p.xyz.z(); }),
        extreme([](const Point& p) { return -p.xyz.z(); }),
        extreme([&](const Point& p) { return -(p.xyz - mean).squaredNorm(); }) };
      std::set<int> seen;
      for (const auto* point : chosen) if (seen.insert(point->id).second) representatives_[static_cast<std::size_t>(configuration)].push_back(*point);
    }
    auto& c3 = representatives_[3];
    c3.erase(std::remove_if(c3.begin(), c3.end(), [](const Point& p) { return p.id == 1360; }), c3.end());
    c3.insert(c3.begin(), representative_1360_);
  }

  void loadRobotModel()
  {
    auto alias = rclcpp::Node::SharedPtr(this, [](rclcpp::Node*) {});
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(alias, "robot_description", true);
    model_ = loader_->getModel();
    if (!model_) throw std::runtime_error("RobotModel load failed");
    arm_group_ = model_->getJointModelGroup(arm_group_name_);
    base_link_ = model_->getLinkModel(base_frame_);
    tcp_link_ = model_->getLinkModel(tcp_frame_);
    if (!arm_group_ || !base_link_ || !tcp_link_ || !arm_group_->getSolverInstance())
      throw std::runtime_error("Arm IK group or required links unavailable");
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    neutral_state_ = std::make_shared<moveit::core::RobotState>(model_);
    neutral_state_->setToDefaultValues();
    for (const std::string finger : { "openarm_left_finger_joint1", "openarm_right_finger_joint1" })
    {
      const auto& bounds = model_->getVariableBounds(finger);
      neutral_state_->setVariablePosition(finger, 0.5 * (bounds.min_position_ + bounds.max_position_));
    }
    neutral_state_->update();
  }

  bool collisionFree(const moveit::core::RobotState& state) const
  {
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    request.contacts = false;
    request.distance = false;
    planning_scene_->checkSelfCollision(request, result, state);
    return !result.collision;
  }

  geometry_msgs::msg::Pose targetPose(const Point& point, const moveit::core::RobotState& reference) const
  {
    Eigen::Isometry3d target_base = Eigen::Isometry3d::Identity();
    target_base.translation() = point.xyz;
    target_base.linear() = target_q_.toRotationMatrix();
    const Eigen::Isometry3d target_model = reference.getGlobalLinkTransform(base_link_) * target_base;
    const Eigen::Quaterniond q(target_model.rotation());
    geometry_msgs::msg::Pose pose;
    pose.position.x = target_model.translation().x(); pose.position.y = target_model.translation().y();
    pose.position.z = target_model.translation().z();
    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
    return pose;
  }

  bool loadStoredC3(moveit::core::RobotState& state) const
  {
    const auto rows = readCsv(stored_c3_state_csv_);
    bool correct_point = false;
    state.setToDefaultValues();
    for (const auto& row : rows)
    {
      const std::string key = text(row, "key");
      const std::string value = text(row, "value");
      if (key == "point_id") correct_point = std::stoi(value) == 1360;
      else if (key.rfind("joint.", 0) == 0) state.setVariablePosition(key.substr(6), std::stod(value));
    }
    state.update();
    return correct_point && state.satisfiesBounds() && collisionFree(state);
  }

  bool solveRepresentative(int configuration, const Point& point, moveit::core::RobotState& output) const
  {
    const auto raw = raw_.find({ configuration, point.id });
    if (raw == raw_.end() || !raw->second.success || !std::isfinite(raw->second.lift)) return false;
    static const int primes[] = {2,3,5,7,11,13,17};
    const auto& arm_names = arm_group_->getVariableNames();
    for (int attempt = 0; attempt < 300; ++attempt)
    {
      moveit::core::RobotState state(*neutral_state_);
      state.setVariablePosition("lift_joint", raw->second.lift);
      state.setVariablePosition("waist_yaw_joint", configuration == 1 || configuration == 3 ? raw->second.yaw : 0.0);
      state.setVariablePosition("waist_pitch_joint", configuration == 2 || configuration == 3 ? raw->second.pitch : 0.0);
      if (attempt % 2 == 1)
      {
        for (std::size_t joint = 0; joint < arm_names.size(); ++joint)
        {
          const auto& bounds = model_->getVariableBounds(arm_names[joint]);
          const double inset = std::max(1e-7, (bounds.max_position_ - bounds.min_position_) * 1e-6);
          const double u = halton(1 + static_cast<std::size_t>(attempt) +
                                  static_cast<std::size_t>(point.id) * 300U, primes[joint]);
          state.setVariablePosition(arm_names[joint], bounds.min_position_ + inset +
            u * (bounds.max_position_ - bounds.min_position_ - 2.0 * inset));
        }
      }
      else if (attempt > 0)
      {
        std::mt19937_64 engine(20260818ULL + static_cast<std::uint64_t>(point.id) * 1000003ULL +
                               static_cast<std::uint64_t>(attempt) * 9176ULL);
        for (const auto& name : arm_names)
        {
          const auto& bounds = model_->getVariableBounds(name);
          const double inset = std::max(1e-7, (bounds.max_position_ - bounds.min_position_) * 1e-6);
          std::uniform_real_distribution<double> distribution(bounds.min_position_ + inset,
                                                               bounds.max_position_ - inset);
          state.setVariablePosition(name, distribution(engine));
        }
      }
      state.update();
      if (!state.setFromIK(arm_group_, targetPose(point, state), tcp_frame_, 0.015)) continue;
      state.setVariablePosition("waist_yaw_joint", configuration == 1 || configuration == 3 ? raw->second.yaw : 0.0);
      state.setVariablePosition("waist_pitch_joint", configuration == 2 || configuration == 3 ? raw->second.pitch : 0.0);
      state.update();
      if (!state.satisfiesBounds() || !collisionFree(state)) continue;
      const Eigen::Isometry3d tcp_base = state.getGlobalLinkTransform(base_link_).inverse() * state.getGlobalLinkTransform(tcp_link_);
      const double position_error = (tcp_base.translation() - point.xyz).norm();
      const Eigen::Quaterniond actual(tcp_base.rotation());
      const double dot = std::clamp(std::abs(actual.normalized().dot(target_q_)), 0.0, 1.0);
      const double orientation_error = 2.0 * std::acos(dot);
      if (position_error > 1e-4 || std::abs(orientation_error) > orientation_tolerance_) continue;
      output = state;
      return true;
    }
    return false;
  }

  void prepareRepresentativeStates()
  {
    goal_states_.resize(4);
    animations_.resize(4);
    animation_safe_.assign(4, false);
    for (int configuration = 0; configuration < 4; ++configuration)
    {
      auto goal = std::make_shared<moveit::core::RobotState>(model_);
      bool solved = configuration == 3 && loadStoredC3(*goal);
      if (!solved)
      {
        for (const auto& point : representatives_[static_cast<std::size_t>(configuration)])
          if (solveRepresentative(configuration, point, *goal)) { solved = true; animated_point_ids_[configuration] = point.id; break; }
      }
      else animated_point_ids_[configuration] = 1360;
      if (!solved)
      {
        RCLCPP_WARN(get_logger(), "Representative IK reproduction failed for C%d; neutral/static markers used", configuration);
        goal = std::make_shared<moveit::core::RobotState>(*neutral_state_);
      }
      goal_states_[static_cast<std::size_t>(configuration)] = goal;
      if (!solved) continue;
      bool safe = true;
      for (int sample = 0; sample < animation_samples_; ++sample)
      {
        moveit::core::RobotState state(model_);
        neutral_state_->interpolate(*goal, static_cast<double>(sample) / (animation_samples_ - 1), state);
        state.update();
        if (!state.satisfiesBounds() || !collisionFree(state)) { safe = false; break; }
        animations_[static_cast<std::size_t>(configuration)].push_back(state);
      }
      animation_safe_[static_cast<std::size_t>(configuration)] = safe;
      if (!safe) animations_[static_cast<std::size_t>(configuration)].clear();
    }
    if (animated_point_ids_[3] != 1360 || !animation_safe_[3])
      throw std::runtime_error("Required point 1360 C3 collision-free visualization animation unavailable");
  }

  void writeMetrics() const
  {
    std::filesystem::create_directories(std::filesystem::path(metrics_csv_).parent_path());
    const std::string temporary = metrics_csv_ + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    out << std::setprecision(15)
        << "configuration,grid_x,grid_y,grid_z,dx,dy,dz,voxel_volume,occupied_voxels,exposed_faces,triangles,reconstructed_volume,validated_volume,absolute_difference,representative_point_ids,animated_point_id,animation_collision_free\n";
    for (int configuration = 0; configuration < 4; ++configuration)
    {
      const auto index = static_cast<std::size_t>(configuration);
      std::ostringstream ids;
      for (std::size_t i = 0; i < representatives_[index].size(); ++i)
      { if (i) ids << ';'; ids << representatives_[index][i].id; }
      const double reconstructed = occupancy_[index].size() * voxel_volume_;
      out << "C" << configuration << ',' << axes_[0].size() << ',' << axes_[1].size() << ',' << axes_[2].size()
          << ',' << spacing_.x() << ',' << spacing_.y() << ',' << spacing_.z() << ',' << voxel_volume_
          << ',' << occupancy_[index].size() << ',' << exposed_faces_[index] << ',' << exposed_faces_[index] * 2
          << ',' << reconstructed << ',' << summary_[index].volume << ',' << std::abs(reconstructed-summary_[index].volume)
          << ',' << ids.str() << ',' << animated_point_ids_[configuration] << ',' << std::boolalpha << animation_safe_[index] << '\n';
    }
    out << "COMBINED_ONLY," << axes_[0].size() << ',' << axes_[1].size() << ',' << axes_[2].size()
        << ',' << spacing_.x() << ',' << spacing_.y() << ',' << spacing_.z() << ',' << voxel_volume_
        << ',' << combined_only_.size() << ',' << combined_only_faces_ << ',' << combined_only_faces_ * 2
        << ',' << combined_only_.size() * voxel_volume_ << ',' << combined_only_volume_ << ','
        << std::abs(combined_only_.size()*voxel_volume_-combined_only_volume_) << ",1360,1360," << animation_safe_[3] << '\n';
    out.close();
    std::filesystem::rename(temporary, metrics_csv_);
  }

  void buildTimeline()
  {
    for (const auto& name : { "ROBOT", "C0", "C1", "C2", "C3", "C0_VS_C3", "COMBINED_ONLY", "FINAL" })
      timeline_.push_back({ name, scene_durations_.at(name) * duration_scale_ });
    const std::set<std::string> scenes{ "robot", "c0_volume", "c1_volume", "c2_volume", "c3_volume",
      "c0_vs_c3", "all_four", "combined_only", "yaw_expansion", "pitch_expansion", "auto" };
    if (!scenes.count(demo_scene_)) throw std::runtime_error("Unknown demo_scene: " + demo_scene_);
  }

  visualization_msgs::msg::Marker baseMarker(int id, const std::string& ns, int type,
      const std_msgs::msg::ColorRGBA& color) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = ns; marker.id = id; marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.color = color;
    return marker;
  }

  void appendPoint(visualization_msgs::msg::Marker& marker, const Eigen::Vector3d& value) const
  {
    geometry_msgs::msg::Point point;
    point.x = value.x(); point.y = value.y(); point.z = value.z();
    marker.points.push_back(point);
  }

  void appendFace(visualization_msgs::msg::Marker& marker, const Eigen::Vector3d& c, int direction) const
  {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    normal[direction / 2] = direction % 2 == 0 ? 1.0 : -1.0;
    const int axis = direction / 2;
    const int u_axis = (axis + 1) % 3;
    const int v_axis = (axis + 2) % 3;
    Eigen::Vector3d face_center = c + normal.cwiseProduct(spacing_) * 0.5;
    Eigen::Vector3d u = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
    u[u_axis] = spacing_[u_axis] * 0.5;
    v[v_axis] = spacing_[v_axis] * 0.5;
    const std::array<Eigen::Vector3d, 4> corners{ face_center-u-v, face_center+u-v, face_center+u+v, face_center-u+v };
    for (const int index : {0,1,2,0,2,3}) appendPoint(marker, corners[static_cast<std::size_t>(index)]);
  }

  void addOccupancy(visualization_msgs::msg::MarkerArray& array, const std::set<Index>& occupied,
                    int id, const std::string& ns, const std_msgs::msg::ColorRGBA& color) const
  {
    (void)ns;
    if (visualization_mode_ == "points")
    {
      auto marker = baseMarker(id, "workspace_envelopes", visualization_msgs::msg::Marker::SPHERE_LIST, color);
      marker.scale.x = marker.scale.y = marker.scale.z = point_diameter_;
      for (const auto& index : occupied) appendPoint(marker, center(index));
      array.markers.push_back(std::move(marker));
      return;
    }
    if (visualization_mode_ == "voxels")
    {
      auto marker = baseMarker(id, "workspace_envelopes", visualization_msgs::msg::Marker::CUBE_LIST, color);
      marker.scale.x = spacing_.x(); marker.scale.y = spacing_.y(); marker.scale.z = spacing_.z();
      for (const auto& index : occupied) appendPoint(marker, center(index));
      array.markers.push_back(std::move(marker));
      return;
    }
    auto marker = baseMarker(id, "workspace_envelopes", visualization_msgs::msg::Marker::TRIANGLE_LIST, color);
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    static const std::array<Index, 6> neighbors{
      Index{1,0,0}, Index{-1,0,0}, Index{0,1,0}, Index{0,-1,0}, Index{0,0,1}, Index{0,0,-1} };
    for (const auto& index : occupied)
    {
      const auto [i,j,k] = index;
      for (int direction = 0; direction < 6; ++direction)
      {
        const auto [di,dj,dk] = neighbors[static_cast<std::size_t>(direction)];
        if (!occupied.count({i+di,j+dj,k+dk})) appendFace(marker, center(index), direction);
      }
    }
    array.markers.push_back(std::move(marker));
  }

  void addRepresentativeMarkers(visualization_msgs::msg::MarkerArray& array, int configuration) const
  {
    auto marker = baseMarker(80 + configuration, "boundary_representatives",
      visualization_msgs::msg::Marker::SPHERE_LIST, rgba(1.0F, 1.0F, 1.0F, 0.95F));
    marker.scale.x = marker.scale.y = marker.scale.z = point_diameter_ * 1.8;
    for (const auto& point : representatives_[static_cast<std::size_t>(configuration)]) appendPoint(marker, point.xyz);
    array.markers.push_back(std::move(marker));
  }

  visualization_msgs::msg::MarkerArray markersFor(const std::string& scene) const
  {
    visualization_msgs::msg::MarkerArray array;
    auto clear = baseMarker(0, "clear", visualization_msgs::msg::Marker::SPHERE, rgba(0,0,0,0));
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    for (int id = 10; id <= 15; ++id)
    {
      auto remove = baseMarker(id, "workspace_envelopes", visualization_msgs::msg::Marker::SPHERE,
                               rgba(0,0,0,0));
      remove.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(remove);
    }
    for (int id = 80; id <= 83; ++id)
    {
      auto remove = baseMarker(id, "boundary_representatives", visualization_msgs::msg::Marker::SPHERE,
                               rgba(0,0,0,0));
      remove.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(remove);
    }
    auto remove_target = baseMarker(90, "point_1360", visualization_msgs::msg::Marker::SPHERE,
                                    rgba(0,0,0,0));
    remove_target.action = visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(remove_target);
    const float alpha = static_cast<float>(visualization_mode_ == "surface" ? surface_alpha_ : voxel_alpha_);
    const std::array<std_msgs::msg::ColorRGBA,4> colors{
      rgba(0.12F,0.55F,1.0F,alpha), rgba(1.0F,0.64F,0.08F,alpha),
      rgba(0.95F,0.22F,0.78F,alpha), rgba(0.12F,0.92F,0.88F,alpha) };
    if (scene == "C0") { addOccupancy(array, occupancy_[0], 10, "c0_envelope", colors[0]); addRepresentativeMarkers(array,0); }
    else if (scene == "C1")
    {
      addOccupancy(array, occupancy_[0], 10, "c0_reference", rgba(0.12F,0.55F,1.0F,0.07F));
      addOccupancy(array, yaw_expansion_, 11, "yaw_expansion", rgba(1.0F,0.64F,0.08F,0.62F));
      addRepresentativeMarkers(array,1);
    }
    else if (scene == "C2")
    {
      addOccupancy(array, occupancy_[0], 10, "c0_reference", rgba(0.12F,0.55F,1.0F,0.07F));
      addOccupancy(array, pitch_expansion_, 12, "pitch_expansion", rgba(0.95F,0.22F,0.78F,0.62F));
      addRepresentativeMarkers(array,2);
    }
    else if (scene == "C3" || scene == "FINAL")
    { addOccupancy(array, occupancy_[3], 13, "c3_envelope", colors[3]); addRepresentativeMarkers(array,3); }
    else if (scene == "C0_VS_C3")
    {
      addOccupancy(array, occupancy_[0], 10, "c0_envelope", rgba(0.12F,0.55F,1.0F,0.24F));
      addOccupancy(array, c3_minus_c0_, 14, "c3_minus_c0", rgba(0.20F,1.0F,0.42F,0.60F));
    }
    else if (scene == "COMBINED_ONLY")
    {
      addOccupancy(array, occupancy_[3], 13, "c3_context", rgba(0.12F,0.92F,0.88F,0.025F));
      addOccupancy(array, combined_only_, 15, "combined_only", rgba(0.76F,0.16F,1.0F,1.0F));
      auto target = baseMarker(90, "point_1360", visualization_msgs::msg::Marker::SPHERE, rgba(1.0F,0.95F,0.15F,1.0F));
      target.pose.position.x = representative_1360_.xyz.x(); target.pose.position.y = representative_1360_.xyz.y();
      target.pose.position.z = representative_1360_.xyz.z();
      target.scale.x = target.scale.y = target.scale.z = 0.055;
      array.markers.push_back(target);
    }
    else if (scene == "ALL_FOUR")
      for (int i=0;i<4;++i) addOccupancy(array, occupancy_[static_cast<std::size_t>(i)], 10+i, "all_four", colors[static_cast<std::size_t>(i)]);
    else if (scene == "YAW_EXPANSION") addOccupancy(array, yaw_expansion_, 11, "yaw_expansion", rgba(1.0F,0.64F,0.08F,0.68F));
    else if (scene == "PITCH_EXPANSION") addOccupancy(array, pitch_expansion_, 12, "pitch_expansion", rgba(0.95F,0.22F,0.78F,0.68F));
    return array;
  }

  std::pair<std::string,double> sceneAt(double elapsed) const
  {
    if (demo_scene_ != "auto")
    {
      if (demo_scene_ == "robot") return {"ROBOT",elapsed};
      if (demo_scene_ == "c0_volume") return {"C0",elapsed};
      if (demo_scene_ == "c1_volume") return {"C1",elapsed};
      if (demo_scene_ == "c2_volume") return {"C2",elapsed};
      if (demo_scene_ == "c3_volume") return {"C3",elapsed};
      if (demo_scene_ == "c0_vs_c3") return {"C0_VS_C3",elapsed};
      if (demo_scene_ == "all_four") return {"ALL_FOUR",elapsed};
      if (demo_scene_ == "yaw_expansion") return {"YAW_EXPANSION",elapsed};
      if (demo_scene_ == "pitch_expansion") return {"PITCH_EXPANSION",elapsed};
      return {"COMBINED_ONLY",elapsed};
    }
    double offset = startup_delay_;
    if (elapsed < offset) return {"WAITING_FOR_RVIZ",0.0};
    for (const auto& scene : timeline_)
    {
      if (elapsed < offset + scene.duration) return {scene.name,elapsed-offset};
      offset += scene.duration;
    }
    return {"FINAL",0.0};
  }

  const moveit::core::RobotState& stateFor(const std::string& scene, double scene_elapsed) const
  {
    int configuration = -1;
    if (scene == "C0") configuration=0; else if (scene == "C1") configuration=1;
    else if (scene == "C2") configuration=2; else if (scene == "C3" || scene == "COMBINED_ONLY" || scene == "FINAL") configuration=3;
    if (configuration < 0) return *neutral_state_;
    const auto index = static_cast<std::size_t>(configuration);
    if (!animation_safe_[index] || animations_[index].empty()) return *goal_states_[index];
    const double duration = scene == "COMBINED_ONLY" ? scene_durations_.at("COMBINED_ONLY") * duration_scale_ :
      scene_durations_.at("C" + std::to_string(configuration)) * duration_scale_;
    const double phase = std::clamp(scene_elapsed / std::max(1e-9,duration), 0.0, 1.0);
    const double ping_pong = phase <= 0.5 ? phase * 2.0 : (1.0-phase) * 2.0;
    const std::size_t state_index = std::min(animations_[index].size()-1,
      static_cast<std::size_t>(ping_pong * static_cast<double>(animations_[index].size()-1)));
    return animations_[index][state_index];
  }

  void publishState(const moveit::core::RobotState& state)
  {
    moveit_msgs::msg::RobotState state_message;
    moveit::core::robotStateToRobotStateMsg(state, state_message, false);
    state_message.joint_state.header.stamp = now();
    moveit_msgs::msg::DisplayRobotState display;
    display.state = state_message;
    state_publisher_->publish(display);
    joint_publisher_->publish(state_message.joint_state);
  }

  void writeRuntime(const std::string& scene, double elapsed) const
  {
    std::filesystem::create_directories(std::filesystem::path(runtime_json_).parent_path());
    const std::string temporary = runtime_json_ + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    out << std::boolalpha << std::setprecision(15)
        << "{\n  \"state\": \"RUNNING_OR_READY\",\n  \"scene\": \"" << scene << "\",\n"
        << "  \"elapsed_s\": " << elapsed << ",\n  \"visualization_mode\": \"" << visualization_mode_ << "\",\n"
        << "  \"grid_dimensions\": [" << axes_[0].size() << ',' << axes_[1].size() << ',' << axes_[2].size() << "],\n"
        << "  \"spacing\": [" << spacing_.x() << ',' << spacing_.y() << ',' << spacing_.z() << "],\n"
        << "  \"voxel_volume\": " << voxel_volume_ << ",\n"
        << "  \"occupied_voxels\": [" << occupancy_[0].size() << ',' << occupancy_[1].size() << ','
        << occupancy_[2].size() << ',' << occupancy_[3].size() << "],\n"
        << "  \"exposed_faces\": [" << exposed_faces_[0] << ',' << exposed_faces_[1] << ','
        << exposed_faces_[2] << ',' << exposed_faces_[3] << "],\n"
        << "  \"combined_only_voxels\": " << combined_only_.size() << ",\n"
        << "  \"combined_only_faces\": " << combined_only_faces_ << ",\n"
        << "  \"point_1360\": [" << representative_1360_.xyz.x() << ',' << representative_1360_.xyz.y() << ',' << representative_1360_.xyz.z() << "],\n"
        << "  \"animated_point_ids\": [" << animated_point_ids_[0] << ',' << animated_point_ids_[1] << ','
        << animated_point_ids_[2] << ',' << animated_point_ids_[3] << "],\n"
        << "  \"animation_collision_free\": [" << animation_safe_[0] << ',' << animation_safe_[1] << ','
        << animation_safe_[2] << ',' << animation_safe_[3] << "],\n"
        << "  \"convex_hull_used\": false,\n  \"holes_preserved\": true,\n"
        << "  \"trajectory_execution\": false,\n  \"controller\": false,\n  \"ros2_control\": false,\n"
        << "  \"hardware\": false,\n  \"amr_motion\": false\n}\n";
    out.close();
    std::filesystem::rename(temporary, runtime_json_);
  }

  void tick()
  {
    const double elapsed = std::chrono::duration<double>(Clock::now()-start_time_).count();
    const auto selected = sceneAt(elapsed);
    if (selected.first != current_scene_)
    {
      current_scene_ = selected.first;
      writeRuntime(current_scene_, elapsed);
      std_msgs::msg::String status; status.data = current_scene_; status_publisher_->publish(status);
      RCLCPP_INFO(get_logger(), "ENVELOPE SCENE %s mode=%s", current_scene_.c_str(), visualization_mode_.c_str());
    }
    if (selected.first == "WAITING_FOR_RVIZ") { publishState(*neutral_state_); return; }
    publishState(stateFor(selected.first, selected.second));
    marker_publisher_->publish(markersFor(selected.first));
  }

  std::string base_frame_, tcp_frame_, arm_group_name_;
  std::string comparison_csv_, summary_csv_, contributions_csv_, fine_points_csv_, new_points_csv_;
  std::string stored_c3_state_csv_, runtime_json_, metrics_csv_;
  std::string source_hash_, source_summary_hash_, source_contributions_hash_;
  std::string demo_scene_, visualization_mode_, current_scene_;
  Eigen::Quaterniond target_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d spacing_{Eigen::Vector3d::Zero()};
  double orientation_tolerance_{}, publish_hz_{}, point_diameter_{}, surface_alpha_{}, voxel_alpha_{};
  double startup_delay_{}, duration_scale_{}, voxel_volume_{};
  int animation_samples_{}, combined_only_count_{};
  double combined_only_volume_{};
  bool preflight_only_{};
  std::array<std::vector<double>,3> axes_;
  std::vector<Summary> summary_;
  std::vector<Point> points_;
  Point representative_1360_;
  std::map<std::pair<int,int>,RawPose> raw_;
  std::array<std::set<Index>,4> occupancy_;
  std::set<Index> c3_minus_c0_, yaw_expansion_, pitch_expansion_, combined_only_;
  std::array<std::size_t,4> exposed_faces_{};
  std::size_t combined_only_faces_{};
  std::array<std::vector<Point>,4> representatives_;
  std::array<int,4> animated_point_ids_{-1,-1,-1,-1};
  std::map<std::string,double> scene_durations_;
  std::vector<Scene> timeline_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup* arm_group_{};
  const moveit::core::LinkModel* base_link_{};
  const moveit::core::LinkModel* tcp_link_{};
  planning_scene::PlanningScenePtr planning_scene_;
  std::shared_ptr<moveit::core::RobotState> neutral_state_;
  std::vector<std::shared_ptr<moveit::core::RobotState>> goal_states_;
  std::vector<std::vector<moveit::core::RobotState>> animations_;
  std::vector<bool> animation_safe_;

  Clock::time_point start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};
}  // namespace fixed_base_workspace_envelope_demo

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto demo = std::make_shared<fixed_base_workspace_envelope_demo::EnvelopeDemo>();
    if (!demo->preflightOnly()) rclcpp::spin(demo);
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("fixed_base_workspace_envelope_demo"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
