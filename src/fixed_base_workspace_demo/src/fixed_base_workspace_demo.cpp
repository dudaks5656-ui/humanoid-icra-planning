#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
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
#include <geometry_msgs/msg/pose.hpp>
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

namespace fixed_base_workspace_demo
{
using Clock = std::chrono::steady_clock;
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
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"')
      {
        field.push_back('"');
        ++i;
      }
      else if (c == '"') quoted = false;
      else field.push_back(c);
    }
    else if (c == '"') quoted = true;
    else if (c == ',')
    {
      fields.push_back(field);
      field.clear();
    }
    else field.push_back(c);
  }
  fields.push_back(field);
  return fields;
}

using CsvRow = std::unordered_map<std::string, std::string>;

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
    if (values.size() != header.size())
      throw std::runtime_error("CSV field-count mismatch in " + path);
    CsvRow row;
    for (std::size_t i = 0; i < header.size(); ++i) row.emplace(header[i], values[i]);
    rows.push_back(std::move(row));
  }
  return rows;
}

double number(const CsvRow& row, const std::string& key)
{
  const auto it = row.find(key);
  if (it == row.end() || it->second.empty()) return kNaN;
  const double value = std::stod(it->second);
  return std::isfinite(value) ? value : kNaN;
}

int integer(const CsvRow& row, const std::string& key)
{
  const auto it = row.find(key);
  if (it == row.end() || it->second.empty()) throw std::runtime_error("Missing integer field: " + key);
  return std::stoi(it->second);
}

std::string text(const CsvRow& row, const std::string& key)
{
  const auto it = row.find(key);
  if (it == row.end()) throw std::runtime_error("Missing text field: " + key);
  return it->second;
}

std::string formatted(double value, int precision)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

double halton(std::size_t index, int base)
{
  double fraction = 1.0;
  double result = 0.0;
  while (index > 0)
  {
    fraction /= static_cast<double>(base);
    result += fraction * static_cast<double>(index % static_cast<std::size_t>(base));
    index /= static_cast<std::size_t>(base);
  }
  return result;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F)
{
  std_msgs::msg::ColorRGBA value;
  value.r = r; value.g = g; value.b = b; value.a = a;
  return value;
}

struct Summary
{
  std::string configuration;
  int total{};
  int reachable{};
  double volume{};
  double increase_percent{};
  double max_x{};
  double mean_manipulability{};
  double mean_joint_margin{};
};

struct PointRecord
{
  std::size_t id{};
  Eigen::Vector3d xyz{ Eigen::Vector3d::Zero() };
  bool success[4]{};
  std::string failure[4];
  double c3_joint_margin{ kNaN };
  double c3_self_clearance{ kNaN };
  double c3_manipulability{ kNaN };
  double c3_yaw{ kNaN };
  double c3_pitch{ kNaN };
  std::string classification;
  std::string nested_status;
};

struct RawC3
{
  bool success{};
  int seeds{};
  int valid_count{};
  double lift{ kNaN };
  double yaw{ kNaN };
  double pitch{ kNaN };
  std::string collision_pairs;
  std::string source;
};

struct Representative
{
  PointRecord point;
  RawC3 raw;
  std::string confidence;
};

struct Scene
{
  std::string name;
  double duration{};
};

class Demo : public rclcpp::Node
{
public:
  Demo() : Node("fixed_base_workspace_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    loadParameters();
    requireSafetyContract();
    loadEvidence();
    selectRepresentative();
    loadRobotModel();
    prepareRepresentativeState();
    prepareAnimation();
    buildTimeline();

    auto transient = rclcpp::QoS(1).reliable().transient_local();
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fixed_base_workspace_demo/markers", transient);
    display_state_publisher_ = create_publisher<moveit_msgs::msg::DisplayRobotState>(
      "/display_robot_state", transient);
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/fixed_base_workspace_demo/status", transient);

    start_time_ = Clock::now();
    current_scene_ = "INITIALIZING";
    writeRuntimeStatus(current_scene_, 0.0);
    if (!preflight_only_)
    {
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_hz_),
                                 std::bind(&Demo::tick, this));
    }
    else
    {
      current_scene_ = "PREFLIGHT_PASS";
      writeRuntimeStatus(current_scene_, 0.0);
      RCLCPP_INFO(get_logger(),
        "FIXED_BASE_WORKSPACE_DEMO PREFLIGHT PASS representative=%zu xyz=(%.6f,%.6f,%.6f) "
        "animation_collision_free=%s",
        representative_.point.id, representative_.point.xyz.x(), representative_.point.xyz.y(),
        representative_.point.xyz.z(), animation_safe_ ? "YES" : "NO_STATIC_ONLY");
    }
  }

  bool preflightOnly() const { return preflight_only_; }

private:
  template <typename T>
  T parameter(const std::string& name) const
  {
    return get_parameter(name).get_value<T>();
  }

  void loadParameters()
  {
    base_frame_ = parameter<std::string>("base_frame");
    tcp_frame_ = parameter<std::string>("tcp_frame");
    arm_group_name_ = parameter<std::string>("arm_group");
    full_group_name_ = parameter<std::string>("full_group");
    target_q_ = Eigen::Quaterniond(parameter<double>("target_qw"), parameter<double>("target_qx"),
                                   parameter<double>("target_qy"), parameter<double>("target_qz"));
    target_q_.normalize();
    orientation_tolerance_ = parameter<double>("orientation_tolerance_rad");
    exact_bound_epsilon_ = parameter<double>("exact_bound_epsilon");
    ik_seeds_ = parameter<int>("representative_ik_seeds");
    required_valid_ = parameter<int>("representative_required_valid_solutions");
    ik_timeout_ = parameter<double>("ik_timeout_s");
    random_seed_ = parameter<int>("random_seed");
    marker_diameter_ = parameter<double>("marker_diameter");
    publish_hz_ = parameter<double>("marker_publish_hz");
    startup_delay_ = parameter<double>("startup_delay_s");
    duration_scale_ = parameter<double>("duration_scale");
    demo_scene_ = parameter<std::string>("demo_scene");
    preflight_only_ = parameter<bool>("preflight_only");
    world_text_enabled_ = parameter<bool>("world_text_enabled");
    animation_samples_ = parameter<int>("animation_samples");

    comparison_csv_ = parameter<std::string>("comparison_csv");
    summary_csv_ = parameter<std::string>("summary_csv");
    contributions_csv_ = parameter<std::string>("contributions_csv");
    fine_points_csv_ = parameter<std::string>("fine_points_csv");
    c3_overrides_csv_ = parameter<std::string>("c3_overrides_csv");
    selected_state_csv_ = parameter<std::string>("selected_state_csv");
    runtime_status_json_ = parameter<std::string>("runtime_status_json");
    source_comparison_hash_ = parameter<std::string>("source_comparison_sha256");
    source_summary_hash_ = parameter<std::string>("source_summary_sha256");
    source_contributions_hash_ = parameter<std::string>("source_contributions_sha256");

    scene_durations_["ROBOT"] = parameter<double>("robot_scene_s");
    scene_durations_["C0"] = parameter<double>("c0_scene_s");
    scene_durations_["C1"] = parameter<double>("c1_scene_s");
    scene_durations_["C2"] = parameter<double>("c2_scene_s");
    scene_durations_["C3"] = parameter<double>("c3_scene_s");
    scene_durations_["COMBINED_C0"] = parameter<double>("failed_config_scene_s");
    scene_durations_["COMBINED_C1"] = parameter<double>("failed_config_scene_s");
    scene_durations_["COMBINED_C2"] = parameter<double>("failed_config_scene_s");
    scene_durations_["COMBINED_C3"] = parameter<double>("combined_reachable_scene_s");
    scene_durations_["ANIMATION"] = parameter<double>("animation_scene_s");
    scene_durations_["FINAL"] = parameter<double>("final_scene_s");
    if (ik_seeds_ <= 0 || ik_seeds_ > 300 || required_valid_ <= 0 || required_valid_ > 3)
      throw std::runtime_error("Representative IK hard limit violated");
    if (animation_samples_ < 2 || animation_samples_ > 501 || publish_hz_ <= 0.0 || duration_scale_ <= 0.0)
      throw std::runtime_error("Invalid demo timing/animation parameter");
  }

  void requireSafetyContract() const
  {
    const std::vector<std::string> flags{ "trajectory_execution", "controller_enabled", "ros2_control_enabled",
                                          "hardware_enabled", "amr_motion_enabled" };
    for (const auto& name : flags)
      if (parameter<bool>(name)) throw std::runtime_error("Forbidden demo safety flag enabled: " + name);
  }

  void loadEvidence()
  {
    static const std::vector<std::string> expected_names{
      "LIFT_ONLY", "LIFT_YAW", "LIFT_PITCH", "LIFT_YAW_PITCH" };
    const auto summary_rows = readCsv(summary_csv_);
    if (summary_rows.size() != 4) throw std::runtime_error("Summary must contain exactly four configurations");
    for (const auto& row : summary_rows)
    {
      Summary s;
      s.configuration = text(row, "configuration");
      s.total = integer(row, "total_points");
      s.reachable = integer(row, "reachable_points");
      s.volume = number(row, "targeted_workspace_volume");
      s.increase_percent = number(row, "percent_delta_vs_lift_only");
      s.max_x = number(row, "x_max");
      s.mean_manipulability = number(row, "mean_manipulability");
      s.mean_joint_margin = number(row, "mean_joint_margin");
      summary_.push_back(s);
    }
    for (std::size_t i = 0; i < expected_names.size(); ++i)
      if (summary_[i].configuration != expected_names[i] || summary_[i].total != 1440)
        throw std::runtime_error("Summary configuration order/point count mismatch");

    const auto contribution_rows = readCsv(contributions_csv_);
    if (contribution_rows.size() != 1) throw std::runtime_error("Contribution summary must contain one row");
    const auto& c = contribution_rows.front();
    yaw_unique_ = integer(c, "yaw_expanded_unique_count");
    pitch_unique_ = integer(c, "pitch_expanded_unique_count");
    overlap_ = integer(c, "yaw_pitch_overlap_count");
    combined_only_ = integer(c, "combined_torso_only_count");
    combined_only_volume_ = number(c, "combined_torso_only_volume");

    const auto comparison_rows = readCsv(comparison_csv_);
    if (comparison_rows.size() != 1440) throw std::runtime_error("Comparison must contain 1,440 rows");
    std::set<std::size_t> ids;
    std::map<std::string, int> class_counts;
    int success_counts[4]{};
    points_.reserve(comparison_rows.size());
    for (const auto& row : comparison_rows)
    {
      PointRecord p;
      p.id = static_cast<std::size_t>(integer(row, "point_id"));
      p.xyz = Eigen::Vector3d(number(row, "tcp_x"), number(row, "tcp_y"), number(row, "tcp_z"));
      p.success[0] = integer(row, "c0_lift_success") == 1;
      p.success[1] = integer(row, "c1_lift_yaw_success") == 1;
      p.success[2] = integer(row, "c2_lift_pitch_success") == 1;
      p.success[3] = integer(row, "c3_lift_yaw_pitch_success") == 1;
      p.failure[0] = text(row, "c0_failure_reason");
      p.failure[1] = text(row, "c1_failure_reason");
      p.failure[2] = text(row, "c2_failure_reason");
      p.failure[3] = text(row, "c3_failure_reason");
      p.c3_joint_margin = number(row, "c3_joint_margin");
      p.c3_self_clearance = number(row, "c3_self_clearance");
      p.c3_manipulability = number(row, "c3_manipulability");
      p.c3_yaw = number(row, "c3_selected_yaw");
      p.c3_pitch = number(row, "c3_selected_pitch");
      p.classification = text(row, "classification");
      p.nested_status = text(row, "nested_consistency_status");
      if (!ids.insert(p.id).second || !p.xyz.allFinite())
        throw std::runtime_error("Duplicate ID or invalid comparison coordinate");
      for (int i = 0; i < 4; ++i) success_counts[i] += p.success[i] ? 1 : 0;
      ++class_counts[p.classification];
      points_.push_back(std::move(p));
    }
    for (int i = 0; i < 4; ++i)
      if (success_counts[i] != summary_[static_cast<std::size_t>(i)].reachable)
        throw std::runtime_error("Point-cloud count differs from validated summary");
    if (class_counts["YAW_EXPANDED"] != yaw_unique_ ||
        class_counts["PITCH_EXPANDED"] != pitch_unique_ ||
        class_counts["YAW_AND_PITCH_INDIVIDUALLY_CAPABLE"] != overlap_ ||
        class_counts["COMBINED_TORSO_ONLY"] != combined_only_)
      throw std::runtime_error("Classification counts differ from contribution summary");

    auto load_raw = [&](const std::string& path, const std::string& source, bool filter_c3) {
      for (const auto& row : readCsv(path))
      {
        if (filter_c3 && text(row, "configuration") != "LIFT_YAW_PITCH") continue;
        RawC3 raw;
        raw.success = integer(row, "success") == 1;
        raw.seeds = integer(row, "ik_seeds_tested");
        raw.valid_count = integer(row, "valid_ik_count");
        raw.lift = number(row, "selected_lift");
        raw.yaw = number(row, "selected_yaw");
        raw.pitch = number(row, "selected_pitch");
        raw.collision_pairs = text(row, "collision_pairs");
        raw.source = source;
        raw_c3_[static_cast<std::size_t>(integer(row, "point_id"))] = raw;
      }
    };
    load_raw(fine_points_csv_, "FINE_PRIMARY", true);
    load_raw(c3_overrides_csv_, "SPECIAL_OVERRIDE", false);
    if (raw_c3_.size() != 1440) throw std::runtime_error("C3 raw-state evidence does not cover 1,440 points");

    if (summary_[0].reachable != 833 || summary_[1].reachable != 1030 || summary_[2].reachable != 976 ||
        summary_[3].reachable != 1119 || yaw_unique_ != 78 || pitch_unique_ != 24 || overlap_ != 119 ||
        combined_only_ != 65)
      throw std::runtime_error("Validated presentation constants drifted from source CSV");
    RCLCPP_INFO(get_logger(),
      "CSV SOURCE PASS clouds=[%d,%d,%d,%d] yaw_unique=%d pitch_unique=%d overlap=%d combined_only=%d",
      summary_[0].reachable, summary_[1].reachable, summary_[2].reachable, summary_[3].reachable,
      yaw_unique_, pitch_unique_, overlap_, combined_only_);
  }

  void selectRepresentative()
  {
    std::vector<double> margins;
    for (const auto& point : points_)
      if (point.classification == "COMBINED_TORSO_ONLY" && std::isfinite(point.c3_joint_margin))
        margins.push_back(point.c3_joint_margin);
    if (margins.empty()) throw std::runtime_error("No combined-torso-only candidates");
    std::sort(margins.begin(), margins.end());
    const double median_margin = margins[margins.size() / 2];

    const PointRecord* selected = nullptr;
    const RawC3* selected_raw = nullptr;
    for (const auto& point : points_)
    {
      if (point.classification != "COMBINED_TORSO_ONLY" || point.success[0] || point.success[1] ||
          point.success[2] || !point.success[3]) continue;
      const auto found = raw_c3_.find(point.id);
      if (found == raw_c3_.end()) continue;
      const auto& raw = found->second;
      const bool high = raw.success && raw.valid_count >= 2 && raw.collision_pairs.empty() &&
                        point.nested_status == "CONSISTENT_OR_RESOLVED" &&
                        std::isfinite(point.c3_joint_margin) && point.c3_joint_margin >= median_margin &&
                        std::isfinite(point.c3_self_clearance) && point.c3_self_clearance > 0.0 &&
                        std::isfinite(point.c3_manipulability);
      if (!high) continue;
      if (!selected || std::make_tuple(point.xyz.x(), point.c3_manipulability, point.c3_joint_margin) >
                       std::make_tuple(selected->xyz.x(), selected->c3_manipulability, selected->c3_joint_margin))
      {
        selected = &point;
        selected_raw = &raw;
      }
    }
    if (!selected || !selected_raw) throw std::runtime_error("No HIGH-confidence combined-only representative");
    representative_.point = *selected;
    representative_.raw = *selected_raw;
    representative_.confidence = "HIGH";
    RCLCPP_INFO(get_logger(),
      "REPRESENTATIVE selected point=%zu xyz=(%.9f,%.9f,%.9f) C0/C1/C2=FAIL C3=PASS "
      "lift=%.9f yaw=%.9f pitch=%.9f margin=%.9f manip=%.9f confidence=HIGH",
      representative_.point.id, representative_.point.xyz.x(), representative_.point.xyz.y(),
      representative_.point.xyz.z(), representative_.raw.lift, representative_.raw.yaw,
      representative_.raw.pitch, representative_.point.c3_joint_margin,
      representative_.point.c3_manipulability);
  }

  void loadRobotModel()
  {
    // The loader does not retain ownership beyond this node's lifetime.  An
    // alias avoids shared_from_this() during construction.
    auto node_alias = rclcpp::Node::SharedPtr(this, [](rclcpp::Node*) {});
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_alias, "robot_description", true);
    model_ = loader_->getModel();
    if (!model_) throw std::runtime_error("RobotModel/SRDF load failed");
    arm_group_ = model_->getJointModelGroup(arm_group_name_);
    full_group_ = model_->getJointModelGroup(full_group_name_);
    base_link_ = model_->getLinkModel(base_frame_);
    tcp_link_ = model_->getLinkModel(tcp_frame_);
    if (!arm_group_ || !full_group_ || !base_link_ || !tcp_link_ || !arm_group_->getSolverInstance())
      throw std::runtime_error("Required MoveIt group/link/IK solver unavailable");
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    neutral_state_ = std::make_shared<moveit::core::RobotState>(model_);
    neutral_state_->setToDefaultValues();
    for (const std::string finger : { "openarm_left_finger_joint1", "openarm_right_finger_joint1" })
    {
      const auto& bound = model_->getVariableBounds(finger);
      neutral_state_->setVariablePosition(finger, 0.5 * (bound.min_position_ + bound.max_position_));
    }
    neutral_state_->update();
  }

  geometry_msgs::msg::Pose targetPose(const moveit::core::RobotState& reference) const
  {
    Eigen::Isometry3d target_base = Eigen::Isometry3d::Identity();
    target_base.translation() = representative_.point.xyz;
    target_base.linear() = target_q_.toRotationMatrix();
    const Eigen::Isometry3d target_model = reference.getGlobalLinkTransform(base_link_) * target_base;
    const Eigen::Quaterniond q(target_model.rotation());
    geometry_msgs::msg::Pose pose;
    pose.position.x = target_model.translation().x();
    pose.position.y = target_model.translation().y();
    pose.position.z = target_model.translation().z();
    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
    return pose;
  }

  void setTorso(moveit::core::RobotState& state) const
  {
    state.setVariablePosition("lift_joint", representative_.raw.lift);
    state.setVariablePosition("waist_yaw_joint", representative_.raw.yaw);
    state.setVariablePosition("waist_pitch_joint", representative_.raw.pitch);
  }

  void seedArm(moveit::core::RobotState& state, int attempt) const
  {
    static const int primes[] = { 2, 3, 5, 7, 11, 13, 17 };
    const auto& names = arm_group_->getVariableNames();
    if (attempt % 2 == 1)
    {
      for (std::size_t i = 0; i < names.size(); ++i)
      {
        const auto& b = model_->getVariableBounds(names[i]);
        const double inset = std::max(exact_bound_epsilon_ * 10.0,
                                      (b.max_position_ - b.min_position_) * 1e-6);
        const double u = halton(1 + static_cast<std::size_t>(attempt) +
                                representative_.point.id * static_cast<std::size_t>(ik_seeds_), primes[i]);
        state.setVariablePosition(names[i], b.min_position_ + inset + u *
          (b.max_position_ - b.min_position_ - 2.0 * inset));
      }
    }
    else if (attempt > 0)
    {
      std::mt19937_64 engine(static_cast<std::uint64_t>(random_seed_) +
        representative_.point.id * 1000003ULL + static_cast<std::uint64_t>(attempt) * 9176ULL);
      for (const auto& name : names)
      {
        const auto& b = model_->getVariableBounds(name);
        const double inset = std::max(exact_bound_epsilon_ * 10.0,
                                      (b.max_position_ - b.min_position_) * 1e-6);
        std::uniform_real_distribution<double> distribution(b.min_position_ + inset, b.max_position_ - inset);
        state.setVariablePosition(name, distribution(engine));
      }
    }
  }

  double jointMargin(const moveit::core::RobotState& state) const
  {
    double margin = std::numeric_limits<double>::infinity();
    for (const auto& name : full_group_->getVariableNames())
    {
      const auto& b = model_->getVariableBounds(name);
      if (!b.position_bounded_) continue;
      const double q = state.getVariablePosition(name);
      margin = std::min(margin, std::min(q - b.min_position_, b.max_position_ - q));
    }
    return margin;
  }

  std::pair<double, double> poseErrors(const moveit::core::RobotState& state) const
  {
    const Eigen::Isometry3d tcp_base = state.getGlobalLinkTransform(base_link_).inverse() *
                                       state.getGlobalLinkTransform(tcp_link_);
    const double position = (tcp_base.translation() - representative_.point.xyz).norm();
    const Eigen::Quaterniond actual(tcp_base.rotation());
    const double dot = std::clamp(std::abs(actual.normalized().dot(target_q_)), 0.0, 1.0);
    return { position, 2.0 * std::acos(dot) };
  }

  bool collisionFree(const moveit::core::RobotState& state, std::string* pairs = nullptr) const
  {
    collision_detection::CollisionRequest request;
    request.contacts = pairs != nullptr;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 50;
    collision_detection::CollisionResult result;
    scene_->checkSelfCollision(request, result, state);
    if (pairs)
    {
      std::set<std::pair<std::string, std::string>> unique;
      for (const auto& item : result.contacts) unique.insert(item.first);
      std::ostringstream out;
      bool first = true;
      for (const auto& item : unique)
      {
        if (!first) out << ';';
        out << item.first << '|' << item.second;
        first = false;
      }
      *pairs = out.str();
    }
    return !result.collision;
  }

  bool validateGoal(moveit::core::RobotState& state, bool strict, std::string& reason)
  {
    state.update();
    if (!state.satisfiesBounds()) { reason = "JOINT_LIMIT_VIOLATION"; return false; }
    computed_joint_margin_ = jointMargin(state);
    if (!(computed_joint_margin_ > exact_bound_epsilon_)) { reason = "ACTIVE_JOINT_AT_BOUND"; return false; }
    const auto errors = poseErrors(state);
    computed_position_error_ = errors.first;
    computed_orientation_error_ = errors.second;
    if (errors.first > 1e-4) { reason = "TCP_POSITION_ERROR"; return false; }
    if (errors.second > orientation_tolerance_) { reason = "ORIENTATION_ERROR"; return false; }
    std::string pairs;
    if (!collisionFree(state, &pairs)) { reason = "SELF_COLLISION:" + pairs; return false; }
    computed_self_clearance_ = scene_->getCollisionEnv()->distanceSelf(state, scene_->getAllowedCollisionMatrix());
    if (strict && !(computed_self_clearance_ >= 0.0)) { reason = "INVALID_SELF_CLEARANCE"; return false; }
    reason = "VALID";
    return true;
  }

  bool loadStoredState(moveit::core::RobotState& state)
  {
    if (!std::filesystem::exists(selected_state_csv_)) return false;
    const auto rows = readCsv(selected_state_csv_);
    std::map<std::string, std::string> values;
    for (const auto& row : rows) values[text(row, "key")] = text(row, "value");
    if (values["source_comparison_sha256"] != source_comparison_hash_ ||
        static_cast<std::size_t>(std::stoull(values["point_id"])) != representative_.point.id)
      throw std::runtime_error("Stored representative state source/point mismatch");
    state = *neutral_state_;
    for (const auto& name : model_->getVariableNames())
    {
      const auto found = values.find("joint." + name);
      if (found != values.end()) state.setVariablePosition(name, std::stod(found->second));
    }
    setTorso(state);
    std::string reason;
    if (!validateGoal(state, true, reason))
      throw std::runtime_error("Stored representative state is no longer valid: " + reason);
    RCLCPP_INFO(get_logger(), "Loaded and revalidated cached representative RobotState");
    return true;
  }

  void solveRepresentative(moveit::core::RobotState& output)
  {
    int valid_count = 0;
    bool found = false;
    double best_margin = -std::numeric_limits<double>::infinity();
    for (int attempt = 0; attempt < ik_seeds_; ++attempt)
    {
      moveit::core::RobotState candidate = *neutral_state_;
      setTorso(candidate);
      seedArm(candidate, attempt);
      candidate.update();
      const auto target = targetPose(candidate);
      if (!candidate.setFromIK(arm_group_, target, tcp_frame_, ik_timeout_)) continue;
      setTorso(candidate);
      candidate.update();
      std::string reason;
      if (!validateGoal(candidate, true, reason)) continue;
      ++valid_count;
      if (!found || computed_joint_margin_ > best_margin)
      {
        output = candidate;
        best_margin = computed_joint_margin_;
        found = true;
      }
      if (valid_count >= required_valid_) break;
    }
    if (!found) throw std::runtime_error("Representative C3 RobotState could not be reproduced within 300 seeds");
    std::string reason;
    if (!validateGoal(output, true, reason)) throw std::runtime_error("Selected C3 RobotState invalid: " + reason);
    RCLCPP_INFO(get_logger(), "Representative IK PASS valid=%d margin=%.9f clearance=%.9f position_error=%.3g orientation_error=%.3g",
                valid_count, computed_joint_margin_, computed_self_clearance_, computed_position_error_,
                computed_orientation_error_);
  }

  void writeStoredState(const moveit::core::RobotState& state) const
  {
    std::filesystem::create_directories(std::filesystem::path(selected_state_csv_).parent_path());
    const std::string temporary = selected_state_csv_ + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    if (!out) throw std::runtime_error("Cannot write representative state CSV");
    out << "key,value\n";
    out << "source_comparison_sha256," << source_comparison_hash_ << '\n';
    out << "source_summary_sha256," << source_summary_hash_ << '\n';
    out << "source_contributions_sha256," << source_contributions_hash_ << '\n';
    out << "point_id," << representative_.point.id << '\n';
    out << std::setprecision(15);
    out << "tcp_x," << representative_.point.xyz.x() << '\n';
    out << "tcp_y," << representative_.point.xyz.y() << '\n';
    out << "tcp_z," << representative_.point.xyz.z() << '\n';
    out << "classification," << representative_.point.classification << '\n';
    out << "confidence," << representative_.confidence << '\n';
    out << "selected_lift," << representative_.raw.lift << '\n';
    out << "selected_yaw," << representative_.raw.yaw << '\n';
    out << "selected_pitch," << representative_.raw.pitch << '\n';
    out << "computed_joint_margin," << computed_joint_margin_ << '\n';
    out << "computed_self_clearance," << computed_self_clearance_ << '\n';
    out << "tcp_position_error," << computed_position_error_ << '\n';
    out << "orientation_error," << computed_orientation_error_ << '\n';
    for (const auto& name : model_->getVariableNames())
      out << "joint." << name << ',' << state.getVariablePosition(name) << '\n';
    out.close();
    std::filesystem::rename(temporary, selected_state_csv_);
  }

  void prepareRepresentativeState()
  {
    goal_state_ = std::make_shared<moveit::core::RobotState>(model_);
    if (!loadStoredState(*goal_state_))
    {
      solveRepresentative(*goal_state_);
      writeStoredState(*goal_state_);
    }
    else
    {
      writeStoredState(*goal_state_);
    }
  }

  void prepareAnimation()
  {
    animation_states_.clear();
    animation_safe_ = true;
    animation_failure_index_ = -1;
    animation_failure_pairs_.clear();
    for (int i = 0; i < animation_samples_; ++i)
    {
      moveit::core::RobotState state(model_);
      neutral_state_->interpolate(*goal_state_, static_cast<double>(i) / (animation_samples_ - 1), state);
      state.update();
      std::string pairs;
      if (!state.satisfiesBounds() || !collisionFree(state, &pairs))
      {
        animation_safe_ = false;
        animation_failure_index_ = i;
        animation_failure_pairs_ = pairs;
        animation_states_.clear();
        break;
      }
      animation_states_.push_back(state);
    }
    if (animation_safe_)
      RCLCPP_INFO(get_logger(), "Visualization-only interpolation PASS samples=%zu", animation_states_.size());
    else
      RCLCPP_WARN(get_logger(),
        "Visualization interpolation rejected at sample=%d pairs=%s; static before/after visualization will be used",
        animation_failure_index_, animation_failure_pairs_.c_str());
  }

  void buildTimeline()
  {
    const std::vector<std::string> names{ "ROBOT", "C0", "C1", "C2", "C3", "COMBINED_C0",
      "COMBINED_C1", "COMBINED_C2", "COMBINED_C3", "ANIMATION", "FINAL" };
    for (const auto& name : names) timeline_.push_back({ name, scene_durations_.at(name) * duration_scale_ });
    const std::set<std::string> manual{ "robot", "c0", "c1", "c2", "c3", "combined_only", "auto" };
    if (!manual.count(demo_scene_)) throw std::runtime_error("Unknown demo_scene: " + demo_scene_);
  }

  const Summary& summary(int index) const { return summary_.at(static_cast<std::size_t>(index)); }

  visualization_msgs::msg::Marker sphereList(int id, const std::string& ns,
                                              const std_msgs::msg::ColorRGBA& rgba, double scale) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = scale;
    marker.color = rgba;
    return marker;
  }

  void addPoint(visualization_msgs::msg::Marker& marker, const Eigen::Vector3d& xyz) const
  {
    geometry_msgs::msg::Point point;
    point.x = xyz.x(); point.y = xyz.y(); point.z = xyz.z();
    marker.points.push_back(point);
  }

  visualization_msgs::msg::Marker textMarker(int id, const std::string& ns, const std::string& value,
      double x, double y, double z, double size, const std_msgs::msg::ColorRGBA& rgba) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x; marker.pose.position.y = y; marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    // TEXT_VIEW_FACING formally uses scale.z for glyph height, but explicitly
    // initializing all axes avoids a zero-width OGRE scaling bug observed on
    // the high-DPI presentation display.
    marker.scale.x = marker.scale.y = marker.scale.z = size;
    marker.color = rgba;
    // RViz/OGRE on the presentation laptop renders whitespace (including an
    // underscore glyph) with an abnormally large advance under desktop
    // fractional scaling. Keep labels compact and deterministic in recordings;
    // punctuation already separates every field in these presentation labels.
    marker.text = value;
    marker.text.erase(std::remove(marker.text.begin(), marker.text.end(), ' '), marker.text.end());
    return marker;
  }

  void addTextBlock(visualization_msgs::msg::MarkerArray& array, int first_id, const std::string& ns,
                    const std::string& value, double x, double y, double z, double line_spacing,
                    double size, const std_msgs::msg::ColorRGBA& rgba) const
  {
    std::istringstream input(value);
    std::string line;
    int index = 0;
    while (std::getline(input, line))
    {
      array.markers.push_back(textMarker(first_id + index, ns, line, x, y,
        z - static_cast<double>(index) * line_spacing, size, rgba));
      ++index;
    }
  }

  void addCurrentCloud(visualization_msgs::msg::MarkerArray& array, int configuration) const
  {
    static const std::vector<std_msgs::msg::ColorRGBA> colors{
      color(0.12F, 0.55F, 1.0F, 0.64F), color(1.0F, 0.64F, 0.08F, 0.64F),
      color(0.95F, 0.22F, 0.78F, 0.64F), color(0.12F, 0.92F, 0.88F, 0.58F) };
    auto marker = sphereList(10 + configuration, "configuration_workspace", colors[configuration], marker_diameter_);
    for (const auto& point : points_) if (point.success[configuration]) addPoint(marker, point.xyz);
    array.markers.push_back(std::move(marker));
  }

  void addC3Categories(visualization_msgs::msg::MarkerArray& array, bool combined_only_mode) const
  {
    struct Category { std::string label; std_msgs::msg::ColorRGBA rgba; };
    const std::vector<Category> categories{
      { "BASELINE_REACHABLE", color(0.12F, 0.55F, 1.0F, combined_only_mode ? 0.06F : 0.40F) },
      { "YAW_EXPANDED", color(1.0F, 0.66F, 0.02F, combined_only_mode ? 0.08F : 0.95F) },
      { "PITCH_EXPANDED", color(1.0F, 0.20F, 0.78F, combined_only_mode ? 0.08F : 0.95F) },
      { "YAW_AND_PITCH_INDIVIDUALLY_CAPABLE", color(0.18F, 0.95F, 0.35F, combined_only_mode ? 0.08F : 0.95F) },
      { "COMBINED_TORSO_ONLY", color(0.76F, 0.16F, 1.0F, 1.0F) },
      { "UNREACHABLE_ALL", color(0.60F, 0.62F, 0.66F, combined_only_mode ? 0.025F : 0.08F) },
    };
    int id = 30;
    for (const auto& category : categories)
    {
      auto marker = sphereList(id++, "workspace_categories", category.rgba,
        category.label == "COMBINED_TORSO_ONLY" ? marker_diameter_ * 1.18 : marker_diameter_ * 0.86);
      for (const auto& point : points_)
        if (point.classification == category.label) addPoint(marker, point.xyz);
      array.markers.push_back(std::move(marker));
    }
  }

  void addTargetAndFrame(visualization_msgs::msg::MarkerArray& array, bool reachable) const
  {
    visualization_msgs::msg::Marker target;
    target.header.frame_id = base_frame_;
    target.header.stamp = now();
    target.ns = "representative_target";
    target.id = 70;
    target.type = visualization_msgs::msg::Marker::SPHERE;
    target.action = visualization_msgs::msg::Marker::ADD;
    target.pose.position.x = representative_.point.xyz.x();
    target.pose.position.y = representative_.point.xyz.y();
    target.pose.position.z = representative_.point.xyz.z();
    target.pose.orientation.w = 1.0;
    target.scale.x = target.scale.y = target.scale.z = 0.055;
    target.color = reachable ? color(0.15F, 1.0F, 0.25F, 1.0F) : color(1.0F, 0.12F, 0.10F, 1.0F);
    array.markers.push_back(target);

    visualization_msgs::msg::Marker axes;
    axes.header = target.header;
    axes.ns = "representative_tcp_frame";
    axes.id = 71;
    axes.type = visualization_msgs::msg::Marker::LINE_LIST;
    axes.action = visualization_msgs::msg::Marker::ADD;
    axes.pose.orientation.w = 1.0;
    axes.scale.x = 0.008;
    const Eigen::Vector3d origin = representative_.point.xyz;
    const std::vector<Eigen::Vector3d> unit{ Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ() };
    const std::vector<std_msgs::msg::ColorRGBA> colors{ color(1,0,0,1), color(0,1,0,1), color(0.1F,0.4F,1,1) };
    for (std::size_t i = 0; i < unit.size(); ++i)
    {
      geometry_msgs::msg::Point a, b;
      a.x = origin.x(); a.y = origin.y(); a.z = origin.z();
      const Eigen::Vector3d tip = origin + target_q_.toRotationMatrix() * unit[i] * 0.10;
      b.x = tip.x(); b.y = tip.y(); b.z = tip.z();
      axes.points.push_back(a); axes.points.push_back(b);
      axes.colors.push_back(colors[i]); axes.colors.push_back(colors[i]);
    }
    array.markers.push_back(std::move(axes));
  }

  std::string statisticsText() const
  {
    std::ostringstream out;
    out << "Fixed-base Workspace Ablation\n"
        << "C0 Lift: " << formatted(summary(0).volume, 6) << " m³  (" << summary(0).reachable << "/1440)\n"
        << "C1 +Yaw: " << formatted(summary(1).volume, 6) << " m³  (+" << formatted(summary(1).increase_percent, 2) << "%)\n"
        << "C2 +Pitch: " << formatted(summary(2).volume, 6) << " m³  (+" << formatted(summary(2).increase_percent, 2) << "%)\n"
        << "C3 +Yaw+Pitch: " << formatted(summary(3).volume, 6) << " m³  (+" << formatted(summary(3).increase_percent, 2) << "%)\n"
        << "Combined-torso-only: " << combined_only_ << " points, " << formatted(combined_only_volume_, 6) << " m³";
    return out.str();
  }

  std::string heading(const std::string& scene) const
  {
    if (scene == "ROBOT") return "FIXED-BASE HUMANOID\nAMR MOTION DISABLED";
    if (scene == "C0") return "C0: ARM + LIFT\nReachable: " + std::to_string(summary(0).reachable) +
      " / 1440   Volume: " + formatted(summary(0).volume, 6) + " m³   Max X: " + formatted(summary(0).max_x, 4) + " m";
    if (scene == "C1") return "C1: ARM + LIFT + YAW\nWorkspace increase: +" +
      formatted(summary(1).increase_percent, 2) + "%   Max X: " + formatted(summary(1).max_x, 4) + " m";
    if (scene == "C2") return "C2: ARM + LIFT + PITCH\nWorkspace increase: +" +
      formatted(summary(2).increase_percent, 2) + "%   Max X: " + formatted(summary(2).max_x, 4) + " m";
    if (scene == "C3") return "C3: ARM + LIFT + YAW + PITCH\nWorkspace increase: +" +
      formatted(summary(3).increase_percent, 2) + "%   Max X: " + formatted(summary(3).max_x, 4) + " m";
    if (scene == "COMBINED_C0") return "SAME TARGET — C0: UNREACHABLE";
    if (scene == "COMBINED_C1") return "SAME TARGET — C1: UNREACHABLE";
    if (scene == "COMBINED_C2") return "SAME TARGET — C2: UNREACHABLE";
    if (scene == "COMBINED_C3") return "SAME TARGET — C3: REACHABLE\nYAW + PITCH REQUIRED";
    if (scene == "ANIMATION") return animation_safe_ ?
      "C3 VALID ROBOTSTATE\nCOLLISION-CHECKED VISUALIZATION INTERPOLATION" :
      "C3 VALID ROBOTSTATE\nSTATIC VIEW — LINEAR INTERPOLATION REJECTED";
    return "FIXED-BASE WORKSPACE RESULT\nYaw +23.65%   Pitch +17.17%   Yaw+Pitch +34.33%";
  }

  visualization_msgs::msg::MarkerArray markersFor(const std::string& scene) const
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = base_frame_;
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    if (scene == "C0") addCurrentCloud(array, 0);
    else if (scene == "C1") addCurrentCloud(array, 1);
    else if (scene == "C2") addCurrentCloud(array, 2);
    else if (scene == "C3" || scene == "FINAL")
    {
      addCurrentCloud(array, 3);
      addC3Categories(array, false);
    }
    else if (scene.rfind("COMBINED_", 0) == 0 || scene == "ANIMATION")
    {
      addC3Categories(array, true);
      addTargetAndFrame(array, scene == "COMBINED_C3" || scene == "ANIMATION");
    }

    if (world_text_enabled_)
    {
      addTextBlock(array, 100, "presentation_heading", heading(scene),
        0.32, -0.35, 1.82, 0.060, 0.034, color(1.0F, 1.0F, 1.0F, 1.0F));
      addTextBlock(array, 110, "presentation_statistics", statisticsText(),
        0.30, 0.49, 1.82, 0.050, 0.022, color(0.88F, 0.94F, 1.0F, 1.0F));
      addTextBlock(array, 130, "presentation_safety",
        "VISUALIZATION-ONLY ROBOTSTATE\nNO TRAJECTORY EXECUTION\nNO CONTROLLER / ROS2_CONTROL / HARDWARE",
        0.31, -0.29, 0.52, 0.040, 0.019, color(1.0F, 0.80F, 0.25F, 1.0F));
      if (scene.rfind("COMBINED_", 0) == 0 || scene == "ANIMATION")
      {
        const auto& p = representative_.point.xyz;
        const std::string point_text = "Point " + std::to_string(representative_.point.id) + "  TCP=(" +
          formatted(p.x(), 4) + ", " + formatted(p.y(), 4) + ", " + formatted(p.z(), 4) + ") m";
        array.markers.push_back(textMarker(140, "target_label", point_text,
          p.x(), p.y(), p.z() + 0.15, 0.021, color(1.0F, 1.0F, 1.0F, 1.0F)));
      }
    }
    return array;
  }

  void publishState(const moveit::core::RobotState& state)
  {
    moveit_msgs::msg::RobotState state_message;
    moveit::core::robotStateToRobotStateMsg(state, state_message, false);
    state_message.joint_state.header.stamp = now();
    moveit_msgs::msg::DisplayRobotState display;
    display.state = state_message;
    display_state_publisher_->publish(display);
    joint_publisher_->publish(state_message.joint_state);
  }

  std::pair<std::string, double> currentAutoScene(double elapsed) const
  {
    double offset = startup_delay_;
    if (elapsed < offset) return { "WAITING_FOR_RVIZ", 0.0 };
    for (const auto& scene : timeline_)
    {
      if (elapsed < offset + scene.duration) return { scene.name, elapsed - offset };
      offset += scene.duration;
    }
    return { "FINAL", scene_durations_.at("FINAL") * duration_scale_ };
  }

  std::pair<std::string, double> sceneAt(double elapsed) const
  {
    if (demo_scene_ == "auto") return currentAutoScene(elapsed);
    if (demo_scene_ == "robot") return { "ROBOT", elapsed };
    if (demo_scene_ == "c0") return { "C0", elapsed };
    if (demo_scene_ == "c1") return { "C1", elapsed };
    if (demo_scene_ == "c2") return { "C2", elapsed };
    if (demo_scene_ == "c3") return { "C3", elapsed };
    return { "COMBINED_C3", elapsed };
  }

  const moveit::core::RobotState& stateFor(const std::string& scene, double scene_elapsed) const
  {
    if (scene == "COMBINED_C3" || scene == "FINAL") return *goal_state_;
    if (scene == "ANIMATION")
    {
      if (!animation_safe_ || animation_states_.empty()) return *goal_state_;
      const double duration = scene_durations_.at("ANIMATION") * duration_scale_;
      const double normalized = std::clamp(scene_elapsed / std::max(1e-9, duration), 0.0, 1.0);
      const std::size_t index = std::min(animation_states_.size() - 1,
        static_cast<std::size_t>(normalized * static_cast<double>(animation_states_.size())));
      return animation_states_[index];
    }
    return *neutral_state_;
  }

  void writeRuntimeStatus(const std::string& scene, double elapsed) const
  {
    std::filesystem::create_directories(std::filesystem::path(runtime_status_json_).parent_path());
    const std::string temporary = runtime_status_json_ + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    out << std::boolalpha << std::setprecision(15)
        << "{\n"
        << "  \"state\": \"RUNNING_OR_READY\",\n"
        << "  \"scene\": \"" << scene << "\",\n"
        << "  \"elapsed_s\": " << elapsed << ",\n"
        << "  \"representative_point_id\": " << representative_.point.id << ",\n"
        << "  \"representative_xyz\": [" << representative_.point.xyz.x() << ", "
        << representative_.point.xyz.y() << ", " << representative_.point.xyz.z() << "],\n"
        << "  \"representative_classification\": \"" << representative_.point.classification << "\",\n"
        << "  \"representative_confidence\": \"" << representative_.confidence << "\",\n"
        << "  \"c0_success\": " << representative_.point.success[0] << ",\n"
        << "  \"c1_success\": " << representative_.point.success[1] << ",\n"
        << "  \"c2_success\": " << representative_.point.success[2] << ",\n"
        << "  \"c3_success\": " << representative_.point.success[3] << ",\n"
        << "  \"animation_collision_free\": " << animation_safe_ << ",\n"
        << "  \"animation_samples\": " << (animation_safe_ ? animation_states_.size() : 0) << ",\n"
        << "  \"animation_failure_index\": " << animation_failure_index_ << ",\n"
        << "  \"animation_failure_pairs\": \"" << animation_failure_pairs_ << "\",\n"
        << "  \"trajectory_execution\": false,\n"
        << "  \"controller\": false,\n"
        << "  \"ros2_control\": false,\n"
        << "  \"hardware\": false,\n"
        << "  \"amr_motion\": false\n"
        << "}\n";
    out.close();
    std::filesystem::rename(temporary, runtime_status_json_);
  }

  void tick()
  {
    const double elapsed = std::chrono::duration<double>(Clock::now() - start_time_).count();
    const auto selected = sceneAt(elapsed);
    const std::string scene = selected.first;
    if (scene != current_scene_)
    {
      current_scene_ = scene;
      writeRuntimeStatus(scene, elapsed);
      std_msgs::msg::String status;
      status.data = scene;
      status_publisher_->publish(status);
      RCLCPP_INFO(get_logger(), "DEMO SCENE %s elapsed=%.3f", scene.c_str(), elapsed);
    }
    if (scene == "WAITING_FOR_RVIZ")
    {
      publishState(*neutral_state_);
      return;
    }
    publishState(stateFor(scene, selected.second));
    marker_publisher_->publish(markersFor(scene));
  }

  std::string base_frame_, tcp_frame_, arm_group_name_, full_group_name_;
  Eigen::Quaterniond target_q_{ Eigen::Quaterniond::Identity() };
  double orientation_tolerance_{}, exact_bound_epsilon_{}, ik_timeout_{}, marker_diameter_{};
  double publish_hz_{}, startup_delay_{}, duration_scale_{};
  int ik_seeds_{}, required_valid_{}, random_seed_{}, animation_samples_{};
  bool preflight_only_{}, world_text_enabled_{};
  std::string demo_scene_;
  std::string comparison_csv_, summary_csv_, contributions_csv_, fine_points_csv_, c3_overrides_csv_;
  std::string selected_state_csv_, runtime_status_json_;
  std::string source_comparison_hash_, source_summary_hash_, source_contributions_hash_;
  std::map<std::string, double> scene_durations_;

  std::vector<Summary> summary_;
  std::vector<PointRecord> points_;
  std::map<std::size_t, RawC3> raw_c3_;
  int yaw_unique_{}, pitch_unique_{}, overlap_{}, combined_only_{};
  double combined_only_volume_{};
  Representative representative_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup* arm_group_{};
  const moveit::core::JointModelGroup* full_group_{};
  const moveit::core::LinkModel* base_link_{};
  const moveit::core::LinkModel* tcp_link_{};
  planning_scene::PlanningScenePtr scene_;
  std::shared_ptr<moveit::core::RobotState> neutral_state_, goal_state_;
  std::vector<moveit::core::RobotState> animation_states_;
  bool animation_safe_{};
  int animation_failure_index_{ -1 };
  std::string animation_failure_pairs_;
  double computed_joint_margin_{}, computed_self_clearance_{}, computed_position_error_{}, computed_orientation_error_{};

  std::vector<Scene> timeline_;
  Clock::time_point start_time_;
  std::string current_scene_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr display_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};
}  // namespace fixed_base_workspace_demo

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto demo = std::make_shared<fixed_base_workspace_demo::Demo>();
    if (!demo->preflightOnly()) rclcpp::spin(demo);
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("fixed_base_workspace_demo"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
