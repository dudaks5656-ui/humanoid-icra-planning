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
#include <mutex>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::string csvEscape(const std::string& value)
{
  std::string result = "\"";
  for (const char c : value)
    result += (c == '"') ? "\"\"" : std::string(1, c);
  return result + "\"";
}

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

std::vector<double> vector3(const YAML::Node& node, const std::string& key)
{
  const YAML::Node value = node[key];
  if (!value || !value.IsSequence() || value.size() != 3)
    throw std::runtime_error("Expected three values at YAML key: " + key);
  return { value[0].as<double>(), value[1].as<double>(), value[2].as<double>() };
}

std::vector<double> vectorN(const YAML::Node& node, const std::string& key)
{
  const YAML::Node value = node[key];
  if (!value || !value.IsSequence() || value.size() == 0)
    throw std::runtime_error("Expected a non-empty sequence at YAML key: " + key);
  std::vector<double> result;
  result.reserve(value.size());
  for (const auto& entry : value)
    result.push_back(entry.as<double>());
  return result;
}

struct SceneConfig
{
  std::string frame_id;
  std::vector<double> box_center;
  double box_width{};
  double box_depth{};
  double box_height{};
  double wall_thickness{};
  double floor_thickness{};
  std::vector<double> target_position;
  std::vector<double> target_size;
  double tcp_to_grasp_center{};
  double pre_grasp_scan_min{};
  double pre_grasp_scan_max{};
  double pre_grasp_scan_step{};
  double pre_grasp_safety_margin{};
  double insertion_offset{};
  double lift_distance{};
  double legacy_extraction_distance{};
  std::vector<double> extraction_clearances;
  std::vector<double> scene_translation_candidates;
  std::vector<double> eef_rpy;
  double planning_time{};
  int planning_attempts{};
  double ik_timeout{};
  double position_tolerance{};
  double orientation_tolerance{};
  double lift_start{};
  double left_finger{};
  double right_finger{};
  std::vector<double> search_x;
  std::vector<double> search_y;
  double search_z{};
  double lift_step{};
  double lift_radius{};
  int ik_multistart_count{};
  std::uint64_t ik_rng_seed{};
};

SceneConfig loadSceneConfig(const std::string& path)
{
  const YAML::Node root = YAML::LoadFile(path);
  SceneConfig config;
  config.frame_id = root["frame_id"].as<std::string>();
  config.box_center = vector3(root["box"], "center_xyz");
  config.box_width = root["box"]["interior_width"].as<double>();
  config.box_depth = root["box"]["interior_depth"].as<double>();
  config.box_height = root["box"]["interior_height"].as<double>();
  config.wall_thickness = root["box"]["wall_thickness"].as<double>();
  config.floor_thickness = root["box"]["floor_thickness"].as<double>();
  config.target_position = vector3(root["target"], "position_xyz");
  config.target_size = vector3(root["target"], "size_xyz");
  config.tcp_to_grasp_center = root["task"]["tcp_to_grasp_center"].as<double>();
  config.pre_grasp_scan_min = root["task"]["pre_grasp_clearance_scan_min"].as<double>();
  config.pre_grasp_scan_max = root["task"]["pre_grasp_clearance_scan_max"].as<double>();
  config.pre_grasp_scan_step = root["task"]["pre_grasp_clearance_scan_step"].as<double>();
  config.pre_grasp_safety_margin = root["task"]["pre_grasp_clearance_safety_margin"].as<double>();
  config.insertion_offset = root["task"]["insertion_offset"].as<double>();
  config.lift_distance = root["task"]["lift_distance"].as<double>();
  config.legacy_extraction_distance = root["task"]["legacy_extraction_distance"].as<double>();
  config.extraction_clearances = vectorN(root["task"], "extraction_clearance_candidates");
  config.scene_translation_candidates = vectorN(root["task"], "scene_translation_x_candidates");
  config.eef_rpy = vector3(root["task"], "end_effector_rpy");
  config.planning_time = root["task"]["planning_time_s"].as<double>();
  config.planning_attempts = root["task"]["planning_attempts"].as<int>();
  config.ik_timeout = root["task"]["ik_timeout_s"].as<double>();
  config.position_tolerance = root["task"]["position_tolerance_m"].as<double>();
  config.orientation_tolerance = root["task"]["orientation_tolerance_rad"].as<double>();
  config.lift_start = root["robot_start"]["lift"].as<double>();
  config.left_finger = root["robot_start"]["left_finger_joint"].as<double>();
  config.right_finger = root["robot_start"]["right_finger_joint"].as<double>();
  config.search_x = vectorN(root["boundary_search"], "target_x_candidates");
  config.search_y = vectorN(root["boundary_search"], "target_y_candidates");
  config.search_z = root["boundary_search"]["target_z"].as<double>();
  config.lift_step = root["boundary_search"]["lift_step"].as<double>();
  config.lift_radius = root["boundary_search"]["lift_local_radius"].as<double>();
  config.ik_multistart_count = root["boundary_search"]["ik_multistart_count"].as<int>();
  config.ik_rng_seed = root["boundary_search"]["ik_rng_seed"].as<std::uint64_t>();
  return config;
}

struct Candidate
{
  std::string id;
  double yaw_deg{};
  double pitch_deg{};
  double yaw{};
  double pitch{};
  double cost{};
  double lift{};
};

struct CollisionStatus
{
  bool joint_limit_valid{ false };
  bool self_collision{ false };
  bool environment_collision{ false };
  std::set<std::pair<std::string, std::string>> pairs;
  std::set<std::pair<std::string, std::string>> self_pairs;
  std::set<std::pair<std::string, std::string>> environment_pairs;
};

enum class ObjectPhase
{
  WORLD_STRICT,
  WORLD_GRASP_CONTACT,
  ATTACHED
};

struct StageResult
{
  std::string stage;
  bool joint_limit_valid{ false };
  bool ik_success{ false };
  bool collision_free{ false };
  bool planning_success{ false };
  std::string failure_reason;
  double planning_time_ms{ 0.0 };
  std::size_t trajectory_points{ 0 };
  double joint_path_length{ 0.0 };
  double position_error{ std::numeric_limits<double>::quiet_NaN() };
  double orientation_error{ std::numeric_limits<double>::quiet_NaN() };
  int ik_seeds_tested{ 0 };
  int collision_free_ik_count{ 0 };
  std::set<std::pair<std::string, std::string>> ik_failure_pairs;
  double minimum_environment_clearance{ std::numeric_limits<double>::infinity() };
  double minimum_self_clearance{ std::numeric_limits<double>::infinity() };
  bool attached_object_box_collision{ false };
};

struct CandidateResult
{
  Candidate candidate;
  std::string mode;
  bool success{ false };
  std::string first_failure_stage;
  double total_planning_time_ms{ 0.0 };
  moveit_msgs::msg::RobotState start_state;
  moveit::core::RobotState final_state;
  std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
  std::vector<StageResult> stages;

  explicit CandidateResult(const moveit::core::RobotModelConstPtr& model) : final_state(model)
  {
  }
};

struct ModeEvaluation
{
  bool success{ false };
  bool structural_failure{ false };
  std::string first_failure_stage;
  std::string failure_pairs;
  std::size_t candidates_tested{ 0 };
  CandidateResult best;

  explicit ModeEvaluation(const moveit::core::RobotModelConstPtr& model) : best(model)
  {
  }
};

struct GeometricEvaluation
{
  bool feasible{ true };
  std::string reason;
  std::string stage;
  std::string pairs;
  int ik_seeds_tested{ 0 };
};

struct DifferentialTarget
{
  std::string id;
  double x{};
  double y{};
  double z{};
  double lift{};
  double proposed_yaw{};
  double proposed_pitch{};
};

struct TrialMetric
{
  bool success{ false };
  double planning_time_ms{ 0.0 };
  double path_length{ 0.0 };
  double min_environment_clearance{ std::numeric_limits<double>::infinity() };
  double min_self_clearance{ std::numeric_limits<double>::infinity() };
};

std::string pairString(const std::set<std::pair<std::string, std::string>>& pairs)
{
  std::ostringstream out;
  bool first = true;
  for (const auto& pair : pairs)
  {
    if (!first)
      out << ';';
    out << pair.first << '|' << pair.second;
    first = false;
  }
  return out.str();
}
}  // namespace

class DifferentialRepeatTrials
{
public:
  explicit DifferentialRepeatTrials(const rclcpp::Node::SharedPtr& node)
    : node_(node), scene_config_(loadSceneConfig(parameter<std::string>("scene_config"))),
      candidate_config_path_(parameter<std::string>("candidate_config")),
      output_csv_(parameter<std::string>("output_csv")), summary_path_(parameter<std::string>("summary_path")),
      target_history_csv_(parameter<std::string>("target_history_csv")),
      extraction_clearance_csv_(parameter<std::string>("extraction_clearance_csv")),
      scene_translation_csv_(parameter<std::string>("scene_translation_csv")),
      planning_attempt_id_(parameter<std::string>("planning_attempt_id")),
      boundary_csv_(parameter<std::string>("boundary_csv")),
      comparison_csv_(parameter<std::string>("comparison_csv")),
      ik_audit_csv_(parameter<std::string>("ik_audit_csv")),
      geometric_csv_(parameter<std::string>("geometric_csv")),
      comparison_input_csv_(parameter<std::string>("comparison_input_csv")),
      repeat_trials_csv_(parameter<std::string>("repeat_trials_csv")),
      budget_sensitivity_csv_(parameter<std::string>("budget_sensitivity_csv")),
      analysis_path_(parameter<std::string>("analysis_path")),
      hold_for_rviz_(parameter<bool>("hold_for_rviz"))
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_)
      throw std::runtime_error("Robot model or SRDF could not be loaded");

    whole_body_group_ = requiredGroup("whole_body");
    left_arm_group_ = requiredGroup("left_arm");
    if (whole_body_group_->getVariableNames().size() != 19)
      throw std::runtime_error("whole_body does not contain exactly 19 independent variables");
    if (!left_arm_group_->getSolverInstance())
      throw std::runtime_error("left_arm KDL kinematics solver is not available");

    local_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", rclcpp::QoS(1).transient_local().reliable());
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    joint_state_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(50), [this]() { publishBufferedState(); });

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node_, moveit::planning_interface::MoveGroupInterface::Options("left_arm", "robot_description", ""),
        nullptr, rclcpp::Duration::from_seconds(20.0));
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setEndEffectorLink(left_tcp_link_);
    move_group_->setPoseReferenceFrame(scene_config_.frame_id);
    move_group_->setPlanningTime(scene_config_.planning_time);
    move_group_->setNumPlanningAttempts(scene_config_.planning_attempts);
    move_group_->setGoalPositionTolerance(scene_config_.position_tolerance);
    move_group_->setGoalOrientationTolerance(scene_config_.orientation_tolerance);

    scene_objects_ = makeSceneObjects();
    for (const auto& object : scene_objects_)
      if (!local_scene_->processCollisionObjectMsg(object))
        throw std::runtime_error("Local PlanningScene rejected object " + object.id);

    resolved_extraction_clearance_ = selectExtractionClearance();

    scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    if (!scene_interface_->applyCollisionObjects(scene_objects_))
      throw std::runtime_error("move_group PlanningScene rejected confined-space objects");

    resolved_pregrasp_clearance_ = selectPreGraspClearance();
    initializeCsv();
    initializeRepeatCsvs();
    RCLCPP_INFO(node_->get_logger(),
                "Differential repeat trials ready: group=left_arm eef=%s pipeline=ompl "
                "planner=RRTConnectkConfigDefault pre_grasp_clearance=%.3f extraction_clearance=%.3f",
                left_tcp_link_.c_str(), resolved_pregrasp_clearance_, resolved_extraction_clearance_);
  }

  bool run()
  {
    return runDifferentialTrials();
  }

  bool holdForRviz() const
  {
    return hold_for_rviz_;
  }

private:
  std::vector<std::string> parseCsvLine(const std::string& line) const
  {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i)
    {
      const char c = line[i];
      if (c == '"')
      {
        if (quoted && i + 1 < line.size() && line[i + 1] == '"')
        {
          field += '"';
          ++i;
        }
        else
          quoted = !quoted;
      }
      else if (c == ',' && !quoted)
      {
        fields.push_back(field);
        field.clear();
      }
      else
        field += c;
    }
    fields.push_back(field);
    return fields;
  }

  std::vector<DifferentialTarget> extractDifferentialTargets() const
  {
    struct Aggregate
    {
      double x{}, y{}, z{};
      bool baseline_seen{ false };
      bool baseline_success{ false };
      bool baseline_collision_free_ik{ false };
      bool proposed_success{ false };
      double lift{}, yaw{}, pitch{};
    };
    std::ifstream input(comparison_input_csv_);
    if (!input)
      throw std::runtime_error("Cannot read comparison input CSV: " + comparison_input_csv_);
    std::string line;
    std::getline(input, line);
    std::map<std::string, Aggregate> aggregates;
    while (std::getline(input, line))
    {
      const auto fields = parseCsvLine(line);
      if (fields.size() != 16)
        throw std::runtime_error("Malformed comparison CSV row");
      const std::string key = fields[0] + "|" + fields[1] + "|" + fields[2];
      auto& aggregate = aggregates[key];
      aggregate.x = std::stod(fields[0]);
      aggregate.y = std::stod(fields[1]);
      aggregate.z = std::stod(fields[2]);
      const std::string& mode = fields[3];
      const bool planning_success = std::stoi(fields[12]) != 0;
      const int collision_free_ik = std::stoi(fields[9]);
      if (mode == "LIFT_ONLY")
      {
        aggregate.baseline_seen = true;
        aggregate.baseline_success = aggregate.baseline_success || planning_success;
        aggregate.baseline_collision_free_ik = aggregate.baseline_collision_free_ik || collision_free_ik > 0;
      }
      else if (mode == "LIFT_YAW_PITCH" && planning_success && !aggregate.proposed_success)
      {
        aggregate.proposed_success = true;
        aggregate.lift = std::stod(fields[4]);
        aggregate.yaw = std::stod(fields[5]);
        aggregate.pitch = std::stod(fields[6]);
      }
    }
    std::vector<DifferentialTarget> targets;
    for (const auto& item : aggregates)
    {
      const auto& value = item.second;
      if (value.baseline_seen && !value.baseline_success && value.baseline_collision_free_ik && value.proposed_success)
      {
        DifferentialTarget target;
        target.x = value.x;
        target.y = value.y;
        target.z = value.z;
        target.lift = value.lift;
        target.proposed_yaw = value.yaw;
        target.proposed_pitch = value.pitch;
        targets.push_back(target);
      }
    }
    std::sort(targets.begin(), targets.end(), [](const auto& a, const auto& b) {
      return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
    });
    for (std::size_t i = 0; i < targets.size(); ++i)
      targets[i].id = "target_" + std::to_string(i + 1);
    if (targets.size() != 3)
      throw std::runtime_error("Expected exactly 3 differential targets, extracted " + std::to_string(targets.size()));
    return targets;
  }

  Candidate trialCandidate(const DifferentialTarget& target, const std::string& mode) const
  {
    Candidate candidate;
    candidate.lift = target.lift;
    candidate.yaw = mode == "LIFT_ONLY" ? 0.0 : target.proposed_yaw;
    candidate.pitch = mode == "LIFT_ONLY" ? 0.0 : target.proposed_pitch;
    candidate.yaw_deg = candidate.yaw * 180.0 / kPi;
    candidate.pitch_deg = candidate.pitch * 180.0 / kPi;
    candidate.cost = std::abs(candidate.yaw_deg) + std::abs(candidate.pitch_deg);
    candidate.id = target.id + "_" + mode;
    return candidate;
  }

  std::string metricKey(const std::string& target_id, const std::string& mode, double budget) const
  {
    std::ostringstream out;
    out << target_id << '|' << mode << '|' << std::fixed << std::setprecision(0) << budget;
    return out.str();
  }

  TrialMetric appendRepeatTrial(const DifferentialTarget& target, const std::string& mode, double budget,
                                int repeat_id, const CandidateResult& result) const
  {
    TrialMetric metric;
    metric.success = result.success;
    std::ofstream out(repeat_trials_csv_, std::ios::app);
    for (const auto& stage : result.stages)
    {
      metric.planning_time_ms += stage.planning_time_ms;
      metric.path_length += stage.joint_path_length;
      metric.min_environment_clearance = std::min(metric.min_environment_clearance,
                                                   stage.minimum_environment_clearance);
      metric.min_self_clearance = std::min(metric.min_self_clearance, stage.minimum_self_clearance);
      out << csvEscape(timestampNow()) << ',' << csvEscape(target.id) << ',' << target.x << ',' << target.y << ','
          << target.z << ',' << csvEscape(mode) << ',' << budget << ',' << repeat_id << ','
          << scene_config_.planning_attempts << ',' << csvEscape(stage.stage) << ','
          << (stage.planning_success ? 1 : 0) << ',' << stage.planning_time_ms << ','
          << csvEscape(result.first_failure_stage) << ',' << stage.trajectory_points << ','
          << stage.joint_path_length << ',' << stage.minimum_environment_clearance << ','
          << stage.minimum_self_clearance << ',' << result.candidate.lift << ',' << result.candidate.yaw << ','
          << result.candidate.pitch << ',' << (stage.attached_object_box_collision ? 1 : 0) << ','
          << csvEscape(stage.failure_reason) << ',' << csvEscape(pairString(stage.ik_failure_pairs)) << '\n';
    }
    return metric;
  }

  double mean(const std::vector<double>& values) const
  {
    if (values.empty())
      return std::numeric_limits<double>::quiet_NaN();
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  }

  double median(std::vector<double> values) const
  {
    if (values.empty())
      return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 ? values[middle] : (values[middle - 1] + values[middle]) / 2.0;
  }

  double standardDeviation(const std::vector<double>& values) const
  {
    if (values.size() < 2)
      return 0.0;
    const double average = mean(values);
    double squared = 0.0;
    for (const double value : values)
      squared += (value - average) * (value - average);
    return std::sqrt(squared / static_cast<double>(values.size() - 1));
  }

  std::pair<double, double> wilson95(int successes, int trials) const
  {
    constexpr double z = 1.959963984540054;
    const double n = static_cast<double>(trials);
    const double p = static_cast<double>(successes) / n;
    const double denominator = 1.0 + z * z / n;
    const double center = (p + z * z / (2.0 * n)) / denominator;
    const double half = z * std::sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denominator;
    return { std::max(0.0, center - half), std::min(1.0, center + half) };
  }

  double successRate(const std::string& target_id, const std::string& mode, double budget) const
  {
    const auto it = trial_metrics_.find(metricKey(target_id, mode, budget));
    if (it == trial_metrics_.end() || it->second.empty())
      return 0.0;
    return static_cast<double>(std::count_if(it->second.begin(), it->second.end(),
                                             [](const auto& value) { return value.success; })) /
           static_cast<double>(it->second.size());
  }

  std::string classifyTarget(const DifferentialTarget& target) const
  {
    const double baseline_2 = successRate(target.id, "LIFT_ONLY", 2.0);
    const double baseline_5 = successRate(target.id, "LIFT_ONLY", 5.0);
    const double baseline_10 = successRate(target.id, "LIFT_ONLY", 10.0);
    const double proposed_10 = successRate(target.id, "LIFT_YAW_PITCH", 10.0);
    if (baseline_2 < 0.8 && std::max(baseline_5, baseline_10) >= 0.8 &&
        std::abs(proposed_10 - baseline_10) <= 0.15)
      return "STOCHASTIC_SHORT_BUDGET_FAILURE";
    if (baseline_10 < 0.5 && proposed_10 - baseline_10 >= 0.25)
      return "PERSISTENT_PLANNING_CONSTRAINT";
    return "NO_MEANINGFUL_TORSO_ADVANTAGE";
  }

  void writeStatistics(const std::vector<DifferentialTarget>& targets) const
  {
    std::ofstream out(budget_sensitivity_csv_, std::ios::trunc);
    out << "target_id,target_x,target_y,target_z,mode,time_budget,trials,successes,success_rate,wilson95_low,"
           "wilson95_high,mean_planning_time_ms,median_planning_time_ms,stddev_planning_time_ms,"
           "mean_joint_path_length,mean_min_environment_clearance,mean_min_self_clearance,classification\n";
    for (const auto& target : targets)
      for (const auto& mode : { std::string("LIFT_ONLY"), std::string("LIFT_YAW_PITCH") })
        for (const double budget : { 2.0, 5.0, 10.0 })
        {
          const auto& metrics = trial_metrics_.at(metricKey(target.id, mode, budget));
          const int successes = std::count_if(metrics.begin(), metrics.end(), [](const auto& m) { return m.success; });
          std::vector<double> times;
          std::vector<double> paths;
          std::vector<double> environment;
          std::vector<double> self;
          for (const auto& metric : metrics)
          {
            times.push_back(metric.planning_time_ms);
            if (metric.success)
            {
              paths.push_back(metric.path_length);
              if (std::isfinite(metric.min_environment_clearance))
                environment.push_back(metric.min_environment_clearance);
              if (std::isfinite(metric.min_self_clearance))
                self.push_back(metric.min_self_clearance);
            }
          }
          const auto interval = wilson95(successes, metrics.size());
          out << target.id << ',' << target.x << ',' << target.y << ',' << target.z << ',' << mode << ',' << budget
              << ',' << metrics.size() << ',' << successes << ','
              << static_cast<double>(successes) / metrics.size() << ',' << interval.first << ',' << interval.second
              << ',' << mean(times) << ',' << median(times) << ',' << standardDeviation(times) << ',' << mean(paths)
              << ',' << mean(environment) << ',' << mean(self) << ',' << classifyTarget(target) << '\n';
        }
  }

  void writeDifferentialAnalysis(const std::vector<DifferentialTarget>& targets) const
  {
    std::ofstream out(analysis_path_, std::ios::trunc);
    out << "# Differential planning-budget analysis\n\nGenerated: " << timestampNow() << "\n\n";
    out << "Targets were extracted automatically from `" << comparison_input_csv_
        << "`: LIFT_ONLY had no full five-stage success, collision-free IK existed, and LIFT_YAW_PITCH succeeded.\n\n";
    out << "Each target used both modes, budgets 2/5/10 s, 20 repeats, and " << scene_config_.planning_attempts
        << " planning attempts. No fixed OMPL seed is claimed. The explicit seed applies only to the deterministic "
           "30-start IK audit.\n\n";
    out << "| Target | XYZ (m) | Proposed Lift/Yaw/Pitch | Classification | LIFT_ONLY rates 2/5/10 s | Proposed rates 2/5/10 s |\n";
    out << "|---|---|---|---|---|---|\n";
    for (const auto& target : targets)
      out << '|' << target.id << "|" << target.x << ' ' << target.y << ' ' << target.z << "|" << target.lift << " / "
          << target.proposed_yaw << " / " << target.proposed_pitch << "|" << classifyTarget(target) << "|"
          << successRate(target.id, "LIFT_ONLY", 2.0) << '/' << successRate(target.id, "LIFT_ONLY", 5.0) << '/'
          << successRate(target.id, "LIFT_ONLY", 10.0) << "|"
          << successRate(target.id, "LIFT_YAW_PITCH", 2.0) << '/'
          << successRate(target.id, "LIFT_YAW_PITCH", 5.0) << '/'
          << successRate(target.id, "LIFT_YAW_PITCH", 10.0) << "|\n";
    out << "\nClassification thresholds used for this focused diagnostic: short-budget convergence requires >=0.8 at 5 or "
           "10 s; persistent advantage requires baseline 10 s <0.5 and Proposed advantage >=0.25. Other outcomes are "
           "NO_MEANINGFUL_TORSO_ADVANTAGE. These are time-budgeted planning outcomes, not structural impossibility.\n\n";
    out << "No trajectory was executed. No controller, ros2_control, hardware interface, serial, CAN, USB, or real "
           "robot node was used.\n";
  }

  bool runDifferentialTrials()
  {
    const auto targets = extractDifferentialTargets();
    constexpr int repeats = 20;
    for (const auto& target : targets)
    {
      configureTarget(target.x, target.y);
      for (const double budget : { 2.0, 5.0, 10.0 })
      {
        scene_config_.planning_time = budget;
        move_group_->setPlanningTime(budget);
        for (int repeat_id = 1; repeat_id <= repeats; ++repeat_id)
        {
          for (const auto& mode : { std::string("LIFT_ONLY"), std::string("LIFT_YAW_PITCH") })
          {
            const Candidate candidate = trialCandidate(target, mode);
            CandidateResult result = runCandidate(mode, candidate);
            trial_metrics_[metricKey(target.id, mode, budget)].push_back(
                appendRepeatTrial(target, mode, budget, repeat_id, result));
          }
        }
        RCLCPP_INFO(node_->get_logger(), "REPEAT_PROGRESS target=%s budget=%.0f completed=%d trials",
                    target.id.c_str(), budget, repeats * 2);
      }
    }
    writeStatistics(targets);
    writeDifferentialAnalysis(targets);
    return true;
  }

  std::vector<double> liftCandidates() const
  {
    const auto& bounds = robot_model_->getVariableBounds("lift_joint");
    const double lower = bounds.min_position_;
    const double upper = bounds.max_position_;
    const double start = scene_config_.lift_start;
    const double local_lower = std::max(lower, start - scene_config_.lift_radius);
    const double local_upper = std::min(upper, start + scene_config_.lift_radius);
    std::vector<double> values;
    for (double value = local_lower; value <= local_upper + 1e-9; value += scene_config_.lift_step)
      values.push_back(value);
    return values;
  }

  double jointMargin(const Candidate& candidate) const
  {
    double margin = std::numeric_limits<double>::infinity();
    for (const auto& item : std::array<std::pair<std::string, double>, 3>{ {
             { "lift_joint", candidate.lift },
             { "waist_yaw_joint", candidate.yaw },
             { "waist_pitch_joint", candidate.pitch },
         } })
    {
      const auto& bounds = robot_model_->getVariableBounds(item.first);
      if (bounds.position_bounded_)
        margin = std::min(margin, std::min(item.second - bounds.min_position_, bounds.max_position_ - item.second));
    }
    return margin;
  }

  std::vector<Candidate> modeCandidates(bool yaw_pitch_enabled) const
  {
    std::vector<Candidate> result;
    const auto lifts = liftCandidates();
    if (!yaw_pitch_enabled)
    {
      for (const double lift : lifts)
      {
        Candidate candidate{ "lift_" + std::to_string(lift) + "_yaw_0_pitch_0", 0.0, 0.0, 0.0, 0.0, 0.0 };
        candidate.lift = lift;
        result.push_back(candidate);
      }
      return result;
    }
    for (const double lift : lifts)
    {
      for (auto torso : loadCandidates())
      {
        torso.lift = lift;
        torso.id = "lift_" + std::to_string(lift) + "_" + torso.id;
        if (candidateWithinLimits(torso))
          result.push_back(torso);
      }
    }
    std::stable_sort(result.begin(), result.end(), [this](const Candidate& a, const Candidate& b) {
      const double lift_a = std::abs(a.lift - scene_config_.lift_start);
      const double lift_b = std::abs(b.lift - scene_config_.lift_start);
      if (std::abs(lift_a - lift_b) > 1e-12)
        return lift_a < lift_b;
      if (std::abs(a.cost - b.cost) > 1e-12)
        return a.cost < b.cost;
      const double margin_a = jointMargin(a);
      const double margin_b = jointMargin(b);
      if (std::abs(margin_a - margin_b) > 1e-12)
        return margin_a > margin_b;
      return a.id < b.id;
    });
    return result;
  }

  bool isBoxObject(const std::string& name) const
  {
    return name.rfind("box_", 0) == 0;
  }

  bool isLeftToolLink(const std::string& name) const
  {
    return name == left_finger_links_[0] || name == left_finger_links_[1] || name == "openarm_left_link7" ||
           name == left_tcp_link_;
  }

  bool directToolBoxPair(const std::pair<std::string, std::string>& pair) const
  {
    return (isBoxObject(pair.first) && isLeftToolLink(pair.second)) ||
           (isBoxObject(pair.second) && isLeftToolLink(pair.first));
  }

  void configureTarget(double x, double y)
  {
    // Clear any prior task-scoped attachment/ACM before replacing only the experiment scene.
    resetSceneForCandidate();
    scene_config_.target_position = { x, y, scene_config_.search_z };
    scene_objects_ = makeSceneObjects();
    local_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    for (const auto& object : scene_objects_)
      if (!local_scene_->processCollisionObjectMsg(object))
        throw std::runtime_error("Target configuration rejected object " + object.id);
    object_phase_ = ObjectPhase::WORLD_STRICT;
    if (!scene_interface_->applyCollisionObjects(scene_objects_))
      throw std::runtime_error("move_group rejected target configuration");
    resolved_pregrasp_clearance_ = selectPreGraspClearance();
    resolved_extraction_clearance_ = scene_config_.extraction_clearances.front();
  }

  bool targetInsideBox() const
  {
    const double front = boxFrontX();
    const double back = scene_config_.box_center[0] + scene_config_.box_depth / 2.0;
    const double y_min = scene_config_.box_center[1] - scene_config_.box_width / 2.0;
    const double y_max = scene_config_.box_center[1] + scene_config_.box_width / 2.0;
    const double z_min = scene_config_.box_center[2] - scene_config_.box_height / 2.0;
    const double z_max = scene_config_.box_center[2] + scene_config_.box_height / 2.0;
    return scene_config_.target_position[0] - scene_config_.target_size[0] / 2.0 >= front - 1e-12 &&
           scene_config_.target_position[0] + scene_config_.target_size[0] / 2.0 <= back + 1e-12 &&
           scene_config_.target_position[1] - scene_config_.target_size[1] / 2.0 >= y_min - 1e-12 &&
           scene_config_.target_position[1] + scene_config_.target_size[1] / 2.0 <= y_max + 1e-12 &&
           scene_config_.target_position[2] - scene_config_.target_size[2] / 2.0 >= z_min - 1e-12 &&
           scene_config_.target_position[2] + scene_config_.target_size[2] / 2.0 <= z_max + 1e-12;
  }

  GeometricEvaluation geometricFeasibility()
  {
    GeometricEvaluation evaluation;
    if (!targetInsideBox())
    {
      evaluation.feasible = false;
      evaluation.reason = "TARGET_OBJECT_OUTSIDE_INTERIOR_BOUNDS";
      return evaluation;
    }
    Candidate probe{ "geometric_probe", 0.0, 0.0, 0.0, 0.0, 0.0 };
    probe.lift = liftCandidates().front();
    moveit::core::RobotState base = initialState(probe);
    for (const auto& target : preExtractionStagePoses())
    {
      int ik_count = 0;
      int direct_count = 0;
      std::set<std::pair<std::string, std::string>> direct_pairs;
      const std::size_t stage_hash = std::hash<std::string>{}("geometric_" + target.first);
      for (int seed_id = 0; seed_id < scene_config_.ik_multistart_count; ++seed_id)
      {
        moveit::core::RobotState state = base;
        std::mt19937_64 rng(scene_config_.ik_rng_seed + stage_hash + static_cast<std::uint64_t>(seed_id));
        for (const auto& variable : left_arm_group_->getVariableNames())
        {
          const auto& bounds = robot_model_->getVariableBounds(variable);
          std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
          state.setVariablePosition(variable, distribution(rng));
        }
        state.update();
        const bool ik = state.setFromIK(left_arm_group_, target.second, left_tcp_link_, scene_config_.ik_timeout);
        ++evaluation.ik_seeds_tested;
        if (!ik)
          continue;
        ++ik_count;
        const CollisionStatus status = checkState(state);
        bool has_direct = false;
        for (const auto& pair : status.environment_pairs)
          if (directToolBoxPair(pair))
          {
            has_direct = true;
            direct_pairs.insert(pair);
          }
        if (has_direct)
          ++direct_count;
      }
      if (ik_count > 0 && direct_count == ik_count)
      {
        evaluation.feasible = false;
        evaluation.reason = "DIRECT_TOOL_BOX_COLLISION_IN_ALL_IK";
        evaluation.stage = target.first;
        evaluation.pairs = pairString(direct_pairs);
        return evaluation;
      }
    }
    return evaluation;
  }

  void appendGeometric(const GeometricEvaluation& value) const
  {
    std::ofstream out(geometric_csv_, std::ios::app);
    out << scene_config_.target_position[0] << ',' << scene_config_.target_position[1] << ','
        << scene_config_.target_position[2] << ',' << (value.feasible ? 1 : 0) << ',' << csvEscape(value.reason)
        << ',' << csvEscape(value.stage) << ',' << value.ik_seeds_tested << ',' << csvEscape(value.pairs) << '\n';
  }

  bool resultStructural(const CandidateResult& result) const
  {
    for (const auto& stage : result.stages)
      if (stage.failure_reason.rfind("STRUCTURAL_STAGE_FAILURE", 0) == 0 &&
          stage.ik_seeds_tested == scene_config_.ik_multistart_count && stage.collision_free_ik_count == 0)
        return true;
    return false;
  }

  void appendComparison(const std::string& mode, const CandidateResult& result, bool structural) const
  {
    int seeds = 0;
    int valid_ik = 0;
    std::set<std::pair<std::string, std::string>> pairs;
    for (const auto& stage : result.stages)
    {
      seeds += stage.ik_seeds_tested;
      valid_ik += stage.collision_free_ik_count;
      pairs.insert(stage.ik_failure_pairs.begin(), stage.ik_failure_pairs.end());
    }
    std::ofstream out(comparison_csv_, std::ios::app);
    out << scene_config_.target_position[0] << ',' << scene_config_.target_position[1] << ','
        << scene_config_.target_position[2] << ',' << csvEscape(mode) << ',' << result.candidate.lift << ','
        << result.candidate.yaw << ',' << result.candidate.pitch << ',' << csvEscape(result.candidate.id) << ','
        << seeds << ',' << valid_ik << ',' << scene_config_.planning_attempts << ',' << scene_config_.planning_time << ','
        << (result.success ? 1 : 0) << ',' << (structural ? 1 : 0) << ','
        << csvEscape(result.first_failure_stage) << ',' << csvEscape(pairString(pairs)) << '\n';
  }

  ModeEvaluation evaluateMode(const std::string& mode, const std::vector<Candidate>& candidates)
  {
    ModeEvaluation evaluation(robot_model_);
    bool all_structural = true;
    for (const auto& candidate : candidates)
    {
      CandidateResult result = runCandidate(mode, candidate);
      ++evaluation.candidates_tested;
      const bool structural = resultStructural(result);
      appendComparison(mode, result, structural);
      if (result.success)
      {
        evaluation.success = true;
        evaluation.best = std::move(result);
        evaluation.structural_failure = false;
        return evaluation;
      }
      all_structural = all_structural && structural;
      if (evaluation.first_failure_stage.empty())
        evaluation.first_failure_stage = result.first_failure_stage;
      for (const auto& stage : result.stages)
        if (!stage.ik_failure_pairs.empty())
          evaluation.failure_pairs = pairString(stage.ik_failure_pairs);
    }
    evaluation.structural_failure = !evaluation.success && all_structural && evaluation.candidates_tested > 0;
    return evaluation;
  }

  void appendBoundary(const GeometricEvaluation& geometric, const ModeEvaluation* baseline,
                      const ModeEvaluation* proposed, bool recovery, const std::string& search_phase) const
  {
    std::ofstream out(boundary_csv_, std::ios::app);
    out << scene_config_.target_position[0] << ',' << scene_config_.target_position[1] << ','
        << scene_config_.target_position[2] << ',' << csvEscape(search_phase) << ','
        << (geometric.feasible ? 1 : 0) << ',' << csvEscape(geometric.reason) << ','
        << csvEscape(geometric.pairs) << ',';
    if (!baseline || !proposed)
      out << ",,,,,,,,,0\n";
    else
      out << (baseline->success ? 1 : 0) << ',' << (baseline->structural_failure ? 1 : 0) << ','
          << csvEscape(baseline->first_failure_stage) << ',' << (proposed->success ? 1 : 0) << ','
          << csvEscape(proposed->first_failure_stage) << ',' << proposed->candidates_tested << ','
          << (proposed->success ? proposed->best.candidate.lift : 0.0) << ','
          << (proposed->success ? proposed->best.candidate.yaw : 0.0) << ','
          << (proposed->success ? proposed->best.candidate.pitch : 0.0) << ',' << (recovery ? 1 : 0) << '\n';
  }

  bool evaluateTarget(double x, double y, const std::string& phase, std::vector<double>& feasible_y)
  {
    configureTarget(x, y);
    const GeometricEvaluation geometric = geometricFeasibility();
    appendGeometric(geometric);
    if (!geometric.feasible)
    {
      appendBoundary(geometric, nullptr, nullptr, false, phase);
      RCLCPP_INFO(node_->get_logger(), "TARGET x=%.3f y=%.3f GEOMETRICALLY_INFEASIBLE stage=%s pairs=%s", x, y,
                  geometric.stage.c_str(), geometric.pairs.c_str());
      return false;
    }
    if (phase == "Y_BOUNDARY")
      feasible_y.push_back(y);
    ModeEvaluation baseline = evaluateMode("LIFT_ONLY", modeCandidates(false));
    ModeEvaluation proposed = evaluateMode("LIFT_YAW_PITCH", modeCandidates(true));
    const bool recovery = baseline.structural_failure && proposed.success;
    appendBoundary(geometric, &baseline, &proposed, recovery, phase);
    RCLCPP_INFO(node_->get_logger(),
                "TARGET x=%.3f y=%.3f baseline_success=%s baseline_structural=%s proposed_success=%s recovery=%s",
                x, y, baseline.success ? "true" : "false", baseline.structural_failure ? "true" : "false",
                proposed.success ? "true" : "false", recovery ? "true" : "false");
    if (recovery)
    {
      writeBoundarySummary(true, phase, baseline, proposed);
      selected_result_ = std::make_unique<CandidateResult>(std::move(proposed.best));
      selected_target_ = scene_config_.target_position;
      publishResult(*selected_result_);
      return true;
    }
    return false;
  }

  bool runBoundarySearch()
  {
    std::vector<double> feasible_y;
    for (const double y : scene_config_.search_y)
      if (evaluateTarget(scene_config_.search_x.front(), y, "Y_BOUNDARY", feasible_y))
        return true;
    for (const double x : scene_config_.search_x)
    {
      if (std::abs(x - scene_config_.search_x.front()) < 1e-12)
        continue;
      for (const double y : feasible_y)
        if (evaluateTarget(x, y, "XY_BOUNDARY", feasible_y))
          return true;
    }
    ModeEvaluation empty_baseline(robot_model_);
    ModeEvaluation empty_proposed(robot_model_);
    writeBoundarySummary(false, "SEARCH_EXHAUSTED", empty_baseline, empty_proposed);
    return false;
  }

  template <typename T>
  T parameter(const std::string& name)
  {
    if (!node_->has_parameter(name))
      throw std::runtime_error("Required parameter is missing: " + name);
    return node_->get_parameter(name).get_value<T>();
  }

  const moveit::core::JointModelGroup* requiredGroup(const std::string& name) const
  {
    const auto* group = robot_model_->getJointModelGroup(name);
    if (!group)
      throw std::runtime_error("Required SRDF group missing: " + name);
    return group;
  }

  moveit_msgs::msg::CollisionObject boxObject(const std::string& id, const std::vector<double>& dimensions,
                                               const std::vector<double>& position) const
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = scene_config_.frame_id;
    object.id = id;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.assign(dimensions.begin(), dimensions.end());
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    object.primitives.push_back(box);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
  }

  std::vector<moveit_msgs::msg::CollisionObject> makeSceneObjects() const
  {
    const auto& c = scene_config_.box_center;
    const double w = scene_config_.box_width;
    const double d = scene_config_.box_depth;
    const double h = scene_config_.box_height;
    const double t = scene_config_.wall_thickness;
    const double f = scene_config_.floor_thickness;
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(boxObject("box_bottom", { d + t, w + 2.0 * t, f }, { c[0], c[1], c[2] - h / 2.0 - f / 2.0 }));
    objects.push_back(boxObject("box_left_wall", { d, t, h }, { c[0], c[1] + w / 2.0 + t / 2.0, c[2] }));
    objects.push_back(boxObject("box_right_wall", { d, t, h }, { c[0], c[1] - w / 2.0 - t / 2.0, c[2] }));
    objects.push_back(boxObject("box_back_wall", { t, w + 2.0 * t, h }, { c[0] + d / 2.0 + t / 2.0, c[1], c[2] }));
    objects.push_back(boxObject("target_object", scene_config_.target_size, scene_config_.target_position));
    return objects;
  }

  moveit_msgs::msg::AttachedCollisionObject makeAttachedTargetFromTransform(
      const Eigen::Isometry3d& target_in_tcp) const
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = left_tcp_link_;
    attached.touch_links = { left_finger_links_[0], left_finger_links_[1] };
    attached.object.header.frame_id = left_tcp_link_;
    attached.object.id = target_object_id_;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.assign(scene_config_.target_size.begin(), scene_config_.target_size.end());
    attached.object.primitives.push_back(box);

    geometry_msgs::msg::Pose pose;
    pose.position.x = target_in_tcp.translation().x();
    pose.position.y = target_in_tcp.translation().y();
    pose.position.z = target_in_tcp.translation().z();
    const Eigen::Quaterniond q(target_in_tcp.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    attached.object.primitive_poses.push_back(pose);
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return attached;
  }

  Eigen::Isometry3d nominalTargetInTcp() const
  {
    const geometry_msgs::msg::Pose grasp = graspPose();
    Eigen::Isometry3d tcp_world = Eigen::Isometry3d::Identity();
    tcp_world.translation() = Eigen::Vector3d(grasp.position.x, grasp.position.y, grasp.position.z);
    tcp_world.linear() = Eigen::Quaterniond(grasp.orientation.w, grasp.orientation.x, grasp.orientation.y,
                                            grasp.orientation.z).toRotationMatrix();
    Eigen::Isometry3d target_world = Eigen::Isometry3d::Identity();
    target_world.translation() = Eigen::Vector3d(scene_config_.target_position[0], scene_config_.target_position[1],
                                                 scene_config_.target_position[2]);
    return tcp_world.inverse() * target_world;
  }

  void applyAcmToMoveGroup()
  {
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    local_scene_->getAllowedCollisionMatrix().getMessage(diff.allowed_collision_matrix);
    if (!scene_interface_->applyPlanningScene(diff))
      throw std::runtime_error("move_group rejected task-scoped ACM update");
  }

  void setFingerTargetContactAllowed(bool allowed)
  {
    auto& acm = local_scene_->getAllowedCollisionMatrixNonConst();
    for (const auto& finger : left_finger_links_)
      acm.setEntry(finger, target_object_id_, allowed);
    applyAcmToMoveGroup();
    object_phase_ = allowed ? ObjectPhase::WORLD_GRASP_CONTACT : ObjectPhase::WORLD_STRICT;
  }

  void resetSceneForCandidate()
  {
    const bool was_attached = object_phase_ == ObjectPhase::ATTACHED;
    local_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    for (const auto& object : scene_objects_)
      if (!local_scene_->processCollisionObjectMsg(object))
        throw std::runtime_error("Local PlanningScene reset rejected object " + object.id);
    object_phase_ = ObjectPhase::WORLD_STRICT;

    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.world.collision_objects = scene_objects_;
    diff.robot_state.is_diff = true;
    if (was_attached)
    {
      moveit_msgs::msg::AttachedCollisionObject remove_attached;
      remove_attached.link_name = left_tcp_link_;
      remove_attached.object.id = target_object_id_;
      remove_attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      diff.robot_state.attached_collision_objects.push_back(remove_attached);
    }
    local_scene_->getAllowedCollisionMatrix().getMessage(diff.allowed_collision_matrix);
    if (!scene_interface_->applyPlanningScene(diff))
      throw std::runtime_error("move_group rejected candidate scene reset");
  }

  void attachTargetAtomically(const moveit::core::RobotState& state)
  {
    local_scene_->setCurrentState(state);
    auto& acm = local_scene_->getAllowedCollisionMatrixNonConst();
    for (const auto& finger : left_finger_links_)
      acm.setEntry(finger, target_object_id_, false);

    Eigen::Isometry3d target_world = Eigen::Isometry3d::Identity();
    target_world.translation() = Eigen::Vector3d(scene_config_.target_position[0], scene_config_.target_position[1],
                                                 scene_config_.target_position[2]);
    attached_target_in_tcp_ = state.getGlobalLinkTransform(left_tcp_link_).inverse() * target_world;
    const auto attached = makeAttachedTargetFromTransform(attached_target_in_tcp_);
    // ADD of an attached object with the same ID atomically removes it from the world.
    if (!local_scene_->processAttachedCollisionObjectMsg(attached))
      throw std::runtime_error("Local PlanningScene failed atomic target attachment transition");

    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    moveit::core::robotStateToRobotStateMsg(state, diff.robot_state);
    diff.robot_state.is_diff = true;
    diff.robot_state.attached_collision_objects.push_back(attached);
    local_scene_->getAllowedCollisionMatrix().getMessage(diff.allowed_collision_matrix);
    if (!scene_interface_->applyPlanningScene(diff))
      throw std::runtime_error("move_group rejected atomic world-to-attached target transition");
    object_phase_ = ObjectPhase::ATTACHED;
  }

  std::vector<Candidate> loadCandidates() const
  {
    const YAML::Node root = YAML::LoadFile(candidate_config_path_);
    std::vector<Candidate> candidates;
    for (const auto& yaw_node : root["yaw_degrees"])
    {
      for (const auto& pitch_node : root["pitch_degrees"])
      {
        Candidate candidate;
        candidate.yaw_deg = yaw_node.as<double>();
        candidate.pitch_deg = pitch_node.as<double>();
        candidate.yaw = candidate.yaw_deg * kPi / 180.0;
        candidate.pitch = candidate.pitch_deg * kPi / 180.0;
        candidate.cost = std::abs(candidate.yaw_deg) + std::abs(candidate.pitch_deg);
        std::ostringstream id;
        id << "yaw_" << std::showpos << candidate.yaw_deg << "_pitch_" << candidate.pitch_deg;
        candidate.id = id.str();
        candidates.push_back(candidate);
      }
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
      if (std::abs(a.cost - b.cost) > 1e-12)
        return a.cost < b.cost;
      if (std::abs(std::abs(a.yaw_deg) - std::abs(b.yaw_deg)) > 1e-12)
        return std::abs(a.yaw_deg) < std::abs(b.yaw_deg);
      if (a.yaw_deg != b.yaw_deg)
        return a.yaw_deg < b.yaw_deg;
      return a.pitch_deg < b.pitch_deg;
    });
    return candidates;
  }

  bool variableWithinBounds(const std::string& name, double value) const
  {
    const auto& bounds = robot_model_->getVariableBounds(name);
    return !bounds.position_bounded_ || (value >= bounds.min_position_ && value <= bounds.max_position_);
  }

  bool candidateWithinLimits(const Candidate& candidate) const
  {
    return variableWithinBounds("lift_joint", candidate.lift) &&
           variableWithinBounds("waist_yaw_joint", candidate.yaw) &&
           variableWithinBounds("waist_pitch_joint", candidate.pitch);
  }

  moveit::core::RobotState initialState(const Candidate& candidate) const
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setVariablePosition("lift_joint", candidate.lift);
    state.setVariablePosition("waist_yaw_joint", candidate.yaw);
    state.setVariablePosition("waist_pitch_joint", candidate.pitch);
    state.setVariablePosition("openarm_left_finger_joint1", scene_config_.left_finger);
    state.setVariablePosition("openarm_right_finger_joint1", scene_config_.right_finger);
    state.update();
    return state;
  }

  geometry_msgs::msg::Pose makePose(double x, double y, double z) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    tf2::Quaternion orientation;
    orientation.setRPY(scene_config_.eef_rpy[0], scene_config_.eef_rpy[1], scene_config_.eef_rpy[2]);
    orientation.normalize();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
  }

  geometry_msgs::msg::Pose graspPose() const
  {
    // The verified approach direction is world +X and the measured grasp-center vector is TCP local +Z.
    return makePose(scene_config_.target_position[0] - scene_config_.tcp_to_grasp_center,
                    scene_config_.target_position[1], scene_config_.target_position[2]);
  }

  geometry_msgs::msg::Pose approachPose(double clearance) const
  {
    geometry_msgs::msg::Pose pose = graspPose();
    pose.position.x -= clearance;
    return pose;
  }

  double selectPreGraspClearance()
  {
    const Candidate locked{ "clearance_scan", 0.0, 0.0, 0.0, 0.0, 0.0 };
    const moveit::core::RobotState seed = initialState(locked);
    for (double clearance = scene_config_.pre_grasp_scan_min;
         clearance <= scene_config_.pre_grasp_scan_max + 1e-9;
         clearance += scene_config_.pre_grasp_scan_step)
    {
      moveit::core::RobotState ik_state = seed;
      const bool ik = ik_state.setFromIK(left_arm_group_, approachPose(clearance), left_tcp_link_,
                                        scene_config_.ik_timeout);
      CollisionStatus status;
      if (ik)
        status = checkState(ik_state);
      RCLCPP_INFO(node_->get_logger(), "PREGRASP_SCAN clearance=%.3f ik=%s collision_free=%s pairs=%s",
                  clearance, ik ? "true" : "false",
                  (ik && status.joint_limit_valid && !status.self_collision && !status.environment_collision) ?
                      "true" : "false",
                  ik ? pairString(status.pairs).c_str() : "IK_FAILURE");
      if (!ik || !status.joint_limit_valid || status.self_collision || status.environment_collision)
        continue;

      const double selected = clearance + scene_config_.pre_grasp_safety_margin;
      if (selected > scene_config_.pre_grasp_scan_max + 1e-9)
        break;
      moveit::core::RobotState safety_state = seed;
      const bool safety_ik = safety_state.setFromIK(left_arm_group_, approachPose(selected), left_tcp_link_,
                                                    scene_config_.ik_timeout);
      if (!safety_ik)
        continue;
      const CollisionStatus safety_status = checkState(safety_state);
      if (safety_status.joint_limit_valid && !safety_status.self_collision && !safety_status.environment_collision)
      {
        RCLCPP_INFO(node_->get_logger(),
                    "PREGRASP_SELECTED first_collision_free=%.3f safety_margin=%.3f selected=%.3f",
                    clearance, scene_config_.pre_grasp_safety_margin, selected);
        return selected;
      }
    }
    throw std::runtime_error("No collision-free pre-grasp clearance in configured FCL scan range");
  }

  double boxFrontX() const
  {
    return scene_config_.box_center[0] - scene_config_.box_depth / 2.0;
  }

  Eigen::Vector3d extractedObjectCenter(double clearance) const
  {
    // The open front is the minimum-X interior plane. Inside is +X and outside is -X.
    // Therefore the object's box-side (+X) face is clearance metres outside that plane.
    return Eigen::Vector3d(boxFrontX() - scene_config_.target_size[0] / 2.0 - clearance,
                           scene_config_.target_position[1],
                           scene_config_.target_position[2] + scene_config_.lift_distance);
  }

  geometry_msgs::msg::Pose extractionPose(double clearance, const Eigen::Isometry3d& target_in_tcp) const
  {
    Eigen::Isometry3d target_goal = Eigen::Isometry3d::Identity();
    target_goal.translation() = extractedObjectCenter(clearance);
    const Eigen::Isometry3d tcp_goal = target_goal * target_in_tcp.inverse();
    geometry_msgs::msg::Pose pose;
    pose.position.x = tcp_goal.translation().x();
    pose.position.y = tcp_goal.translation().y();
    pose.position.z = tcp_goal.translation().z();
    Eigen::Quaterniond q(tcp_goal.rotation());
    q.normalize();
    if (q.w() < 0.0)
      q.coeffs() *= -1.0;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  bool fullyOutsideFront(double clearance) const
  {
    const double box_side_face_x = extractedObjectCenter(clearance).x() + scene_config_.target_size[0] / 2.0;
    return box_side_face_x <= boxFrontX() - clearance + 1e-12;
  }

  double selectExtractionClearance()
  {
    std::ofstream output(extraction_clearance_csv_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot create extraction clearance CSV: " + extraction_clearance_csv_);
    output << "timestamp,planning_attempt_id,clearance,box_front_x,object_center_x,object_center_y,object_center_z,"
              "tcp_x,tcp_y,tcp_z,fully_outside,mode,candidate_id,yaw,pitch,ik_success,joint_limit_valid,"
              "self_collision,robot_box_collision,attached_object_box_collision,colliding_pairs\n";

    const Eigen::Isometry3d target_in_tcp = nominalTargetInTcp();
    std::vector<std::pair<std::string, Candidate>> cases;
    cases.push_back({ "TORSO_LOCKED", Candidate{ "locked_yaw_0_pitch_0", 0.0, 0.0, 0.0, 0.0, 0.0 } });
    for (const auto& candidate : loadCandidates())
      if (candidateWithinLimits(candidate))
        cases.push_back({ "TORSO_CANDIDATE_SEARCH", candidate });

    double selected = std::numeric_limits<double>::quiet_NaN();
    for (const double clearance : scene_config_.extraction_clearances)
    {
      const Eigen::Vector3d center = extractedObjectCenter(clearance);
      const geometry_msgs::msg::Pose tcp = extractionPose(clearance, target_in_tcp);
      bool candidate_ik_exists = false;
      for (const auto& mode_candidate : cases)
      {
        auto audit_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
        for (const auto& object : scene_objects_)
          if (object.id != target_object_id_ && !audit_scene->processCollisionObjectMsg(object))
            throw std::runtime_error("Extraction audit rejected object " + object.id);
        audit_scene->setCurrentState(initialState(mode_candidate.second));
        if (!audit_scene->processAttachedCollisionObjectMsg(makeAttachedTargetFromTransform(target_in_tcp)))
          throw std::runtime_error("Extraction audit failed to attach target");
        moveit::core::RobotState& state = audit_scene->getCurrentStateNonConst();
        const bool ik = state.setFromIK(left_arm_group_, tcp, left_tcp_link_, scene_config_.ik_timeout);
        CollisionStatus status;
        if (ik)
          status = checkStateInScene(audit_scene, state);
        bool robot_box = false;
        bool attached_box = false;
        for (const auto& pair : status.environment_pairs)
        {
          if (pairContains(pair, target_object_id_) && pairContainsBox(pair))
            attached_box = true;
          else if (pairContainsBox(pair))
            robot_box = true;
        }
        if (mode_candidate.first == "TORSO_CANDIDATE_SEARCH" && ik)
          candidate_ik_exists = true;
        output << csvEscape(timestampNow()) << ',' << csvEscape(planning_attempt_id_) << ','
               << std::setprecision(15) << clearance << ',' << boxFrontX() << ',' << center.x() << ',' << center.y()
               << ',' << center.z() << ',' << tcp.position.x << ',' << tcp.position.y << ',' << tcp.position.z << ','
               << (fullyOutsideFront(clearance) ? 1 : 0) << ',' << csvEscape(mode_candidate.first) << ','
               << csvEscape(mode_candidate.second.id) << ',' << mode_candidate.second.yaw << ','
               << mode_candidate.second.pitch << ',' << (ik ? 1 : 0) << ','
               << (ik && status.joint_limit_valid ? 1 : 0) << ',' << (ik && status.self_collision ? 1 : 0) << ','
               << (robot_box ? 1 : 0) << ',' << (attached_box ? 1 : 0) << ','
               << csvEscape(ik ? pairString(status.pairs) : "IK_FAILURE") << '\n';
      }
      if (std::isnan(selected) && fullyOutsideFront(clearance) && candidate_ik_exists)
        selected = clearance;
    }

    std::ofstream scene_output(scene_translation_csv_, std::ios::trunc);
    scene_output << "timestamp,planning_attempt_id,translation_x,status,reason\n";
    if (!std::isnan(selected))
    {
      scene_output << csvEscape(timestampNow()) << ',' << csvEscape(planning_attempt_id_)
                   << ",0,NOT_REQUIRED,EXTRACTION_ENDPOINT_REACHABLE_AT_ORIGINAL_SCENE\n";
      return selected;
    }
    scene_output << csvEscape(timestampNow()) << ',' << csvEscape(planning_attempt_id_)
                 << ",0,REQUIRED,EXTRACTION_ENDPOINT_UNREACHABLE_DUE_TO_BOX_PLACEMENT\n";
    throw std::runtime_error("EXTRACTION_ENDPOINT_UNREACHABLE_DUE_TO_BOX_PLACEMENT");
  }

  std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> preExtractionStagePoses() const
  {
    const geometry_msgs::msg::Pose grasp = graspPose();
    geometry_msgs::msg::Pose insertion = grasp;
    insertion.position.x -= scene_config_.insertion_offset;
    geometry_msgs::msg::Pose lift = grasp;
    lift.position.z += scene_config_.lift_distance;
    return {
      { "APPROACH", approachPose(resolved_pregrasp_clearance_) },
      { "INSERTION", insertion },
      { "GRASP_POSE", grasp },
      { "LIFT", lift },
    };
  }

  CollisionStatus checkStateInScene(const planning_scene::PlanningSceneConstPtr& scene,
                                    moveit::core::RobotState& state) const
  {
    state.update();
    CollisionStatus status;
    status.joint_limit_valid = state.satisfiesBounds(whole_body_group_);
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 50;

    collision_detection::CollisionResult self_result;
    scene->checkSelfCollision(request, self_result, state);
    status.self_collision = self_result.collision;

    collision_detection::CollisionResult full_result;
    scene->checkCollision(request, full_result, state);
    for (const auto& entry : full_result.contacts)
    {
      for (const auto& contact : entry.second)
      {
        if (contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT ||
            contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
          status.environment_collision = true;
      }
    }
    for (const auto& entry : self_result.contacts)
    {
      status.pairs.insert(entry.first);
      status.self_pairs.insert(entry.first);
    }
    for (const auto& entry : full_result.contacts)
    {
      bool world_contact = false;
      for (const auto& contact : entry.second)
        if (contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT ||
            contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
          world_contact = true;
      if (world_contact)
      {
        status.pairs.insert(entry.first);
        status.environment_pairs.insert(entry.first);
      }
    }
    return status;
  }

  CollisionStatus checkState(moveit::core::RobotState& state) const
  {
    return checkStateInScene(local_scene_, state);
  }

  bool pairContains(const std::pair<std::string, std::string>& pair, const std::string& name) const
  {
    return pair.first == name || pair.second == name;
  }

  bool pairContainsFinger(const std::pair<std::string, std::string>& pair) const
  {
    return pairContains(pair, left_finger_links_[0]) || pairContains(pair, left_finger_links_[1]);
  }

  bool pairContainsBox(const std::pair<std::string, std::string>& pair) const
  {
    return pair.first.rfind("box_", 0) == 0 || pair.second.rfind("box_", 0) == 0;
  }

  std::string collisionFailure(const CollisionStatus& status) const
  {
    for (const auto& pair : status.environment_pairs)
    {
      if (pairContains(pair, target_object_id_) && pairContainsBox(pair) && object_phase_ == ObjectPhase::ATTACHED)
        return "ATTACHED_OBJECT_BOX_COLLISION:" + pairString(status.environment_pairs);
      if (pairContains(pair, target_object_id_) && pairContainsFinger(pair))
        return "FINGER_TARGET_PREMATURE_COLLISION:" + pairString(status.environment_pairs);
      if (pairContains(pair, target_object_id_))
        return "NON_FINGER_TARGET_COLLISION:" + pairString(status.environment_pairs);
      if (pairContainsBox(pair))
        return "ROBOT_BOX_COLLISION:" + pairString(status.environment_pairs);
    }
    for (const auto& pair : status.self_pairs)
    {
      if (pairContains(pair, target_object_id_))
        return "NON_FINGER_TARGET_COLLISION:" + pairString(status.self_pairs);
    }
    if (status.self_collision)
      return "ROBOT_SELF_COLLISION:" + pairString(status.self_pairs);
    if (status.environment_collision)
      return "ROBOT_BOX_COLLISION:" + pairString(status.environment_pairs);
    return "";
  }

  std::pair<double, double> stateClearances(const moveit::core::RobotState& state) const
  {
    const auto& acm = local_scene_->getAllowedCollisionMatrix();
    const double environment = local_scene_->getCollisionEnv()->distanceRobot(state, acm);
    const double self = local_scene_->getCollisionEnv()->distanceSelf(state, acm);
    return { environment, self };
  }

  bool hasAttachedObjectBoxCollision(const CollisionStatus& status) const
  {
    if (object_phase_ != ObjectPhase::ATTACHED)
      return false;
    for (const auto& pair : status.environment_pairs)
      if (pairContains(pair, target_object_id_) && pairContainsBox(pair))
        return true;
    return false;
  }

  bool trajectoryCollisionFree(const moveit::core::RobotState& start,
                               const moveit_msgs::msg::RobotTrajectory& trajectory,
                               moveit::core::RobotState& final_state, std::string& failure,
                               double& minimum_environment_clearance, double& minimum_self_clearance,
                               bool& attached_object_box_collision) const
  {
    final_state = start;
    for (const auto& point : trajectory.joint_trajectory.points)
    {
      if (point.positions.size() != trajectory.joint_trajectory.joint_names.size())
      {
        failure = "TRAJECTORY_DIMENSION_MISMATCH";
        return false;
      }
      final_state.setVariablePositions(trajectory.joint_trajectory.joint_names, point.positions);
      const CollisionStatus status = checkState(final_state);
      const auto clearance = stateClearances(final_state);
      minimum_environment_clearance = std::min(minimum_environment_clearance, clearance.first);
      minimum_self_clearance = std::min(minimum_self_clearance, clearance.second);
      attached_object_box_collision = attached_object_box_collision || hasAttachedObjectBoxCollision(status);
      if (!status.joint_limit_valid)
      {
        failure = "TRAJECTORY_JOINT_LIMIT";
        return false;
      }
      if (status.self_collision)
      {
        failure = collisionFailure(status);
        return false;
      }
      if (status.environment_collision)
      {
        failure = collisionFailure(status);
        return false;
      }
    }
    return true;
  }

  double pathLength(const moveit_msgs::msg::RobotTrajectory& trajectory) const
  {
    const auto& points = trajectory.joint_trajectory.points;
    double length = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i)
    {
      double squared = 0.0;
      for (std::size_t j = 0; j < points[i].positions.size(); ++j)
      {
        const double delta = points[i].positions[j] - points[i - 1].positions[j];
        squared += delta * delta;
      }
      length += std::sqrt(squared);
    }
    return length;
  }

  void poseError(const moveit::core::RobotState& state, const geometry_msgs::msg::Pose& desired,
                 double& position_error, double& orientation_error) const
  {
    const Eigen::Isometry3d& actual = state.getGlobalLinkTransform(left_tcp_link_);
    const Eigen::Vector3d desired_position(desired.position.x, desired.position.y, desired.position.z);
    position_error = (actual.translation() - desired_position).norm();
    const Eigen::Quaterniond actual_q(actual.rotation());
    const Eigen::Quaterniond desired_q(desired.orientation.w, desired.orientation.x, desired.orientation.y,
                                       desired.orientation.z);
    orientation_error = actual_q.angularDistance(desired_q);
  }

  void appendIkAudit(const std::string& mode, const Candidate& candidate, const std::string& stage, int seed_id,
                     bool ik_success, const CollisionStatus& status) const
  {
    std::ofstream out(ik_audit_csv_, std::ios::app);
    out << csvEscape(timestampNow()) << ',' << scene_config_.target_position[0] << ','
        << scene_config_.target_position[1] << ',' << scene_config_.target_position[2] << ','
        << csvEscape(mode) << ',' << candidate.lift << ',' << candidate.yaw << ',' << candidate.pitch << ','
        << csvEscape(stage) << ',' << seed_id << ',' << scene_config_.ik_rng_seed << ','
        << (ik_success ? 1 : 0) << ',' << (ik_success && status.joint_limit_valid ? 1 : 0) << ','
        << (ik_success && status.self_collision ? 1 : 0) << ','
        << (ik_success && status.environment_collision ? 1 : 0) << ','
        << (ik_success && status.joint_limit_valid && !status.self_collision && !status.environment_collision ? 1 : 0)
        << ',' << csvEscape(ik_success ? pairString(status.pairs) : "IK_FAILURE") << '\n';
  }

  bool findCollisionFreeIk(const std::string& mode, const Candidate& candidate, const std::string& stage,
                           const geometry_msgs::msg::Pose& pose, const moveit::core::RobotState& current,
                           StageResult& result) const
  {
    bool any_ik = false;
    bool any_valid = false;
    const std::size_t stage_hash = std::hash<std::string>{}(stage) +
                                   static_cast<std::size_t>(std::llround(candidate.lift * 1e6)) * 17U +
                                   static_cast<std::size_t>(std::llround((candidate.yaw + kPi) * 1e6)) * 31U +
                                   static_cast<std::size_t>(std::llround((candidate.pitch + kPi) * 1e6)) * 47U;
    for (int seed_id = 0; seed_id < scene_config_.ik_multistart_count; ++seed_id)
    {
      moveit::core::RobotState ik_state = current;
      if (seed_id > 0)
      {
        std::mt19937_64 rng(scene_config_.ik_rng_seed + stage_hash + static_cast<std::uint64_t>(seed_id));
        for (const auto& variable : left_arm_group_->getVariableNames())
        {
          const auto& bounds = robot_model_->getVariableBounds(variable);
          const double lower = bounds.position_bounded_ ? bounds.min_position_ : -kPi;
          const double upper = bounds.position_bounded_ ? bounds.max_position_ : kPi;
          std::uniform_real_distribution<double> distribution(lower, upper);
          ik_state.setVariablePosition(variable, distribution(rng));
        }
        ik_state.update();
      }
      const bool ik = ik_state.setFromIK(left_arm_group_, pose, left_tcp_link_, scene_config_.ik_timeout);
      CollisionStatus status;
      if (ik)
      {
        any_ik = true;
        status = checkState(ik_state);
        result.ik_failure_pairs.insert(status.pairs.begin(), status.pairs.end());
        if (status.joint_limit_valid && !status.self_collision && !status.environment_collision)
        {
          any_valid = true;
          ++result.collision_free_ik_count;
          const auto clearance = stateClearances(ik_state);
          result.minimum_environment_clearance = std::min(result.minimum_environment_clearance, clearance.first);
          result.minimum_self_clearance = std::min(result.minimum_self_clearance, clearance.second);
        }
      }
      ++result.ik_seeds_tested;
      appendIkAudit(mode, candidate, stage, seed_id, ik, status);
    }
    result.ik_success = any_ik;
    result.collision_free = any_valid;
    result.joint_limit_valid = any_valid;
    return any_valid;
  }

  StageResult planStage(const std::string& mode, const Candidate& candidate, const std::string& stage,
                        const geometry_msgs::msg::Pose& pose, moveit::core::RobotState& current,
                        moveit_msgs::msg::RobotTrajectory& output_trajectory) const
  {
    StageResult result;
    result.stage = stage;
    CollisionStatus start_status = checkState(current);
    const auto start_clearance = stateClearances(current);
    result.minimum_environment_clearance = std::min(result.minimum_environment_clearance, start_clearance.first);
    result.minimum_self_clearance = std::min(result.minimum_self_clearance, start_clearance.second);
    result.joint_limit_valid = start_status.joint_limit_valid;
    if (!start_status.joint_limit_valid)
    {
      result.failure_reason = "START_JOINT_LIMIT";
      return result;
    }
    if (start_status.self_collision)
    {
      result.failure_reason = collisionFailure(start_status);
      return result;
    }
    if (start_status.environment_collision)
    {
      result.failure_reason = collisionFailure(start_status);
      return result;
    }

    if (!findCollisionFreeIk(mode, candidate, stage, pose, current, result))
    {
      result.failure_reason = "STRUCTURAL_STAGE_FAILURE:" + pairString(result.ik_failure_pairs);
      return result;
    }

    move_group_->clearPoseTargets();
    move_group_->setStartState(current);
    move_group_->setPoseTarget(pose, left_tcp_link_);
    publishStateOnce(current);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto begin = std::chrono::steady_clock::now();
    const moveit::core::MoveItErrorCode error = move_group_->plan(plan);
    const auto end = std::chrono::steady_clock::now();
    result.planning_time_ms = std::chrono::duration<double, std::milli>(end - begin).count();
    result.planning_success = error == moveit::core::MoveItErrorCode::SUCCESS;
    if (!result.planning_success)
    {
      result.failure_reason = "MOTION_PLANNING_FAILURE:CODE_" + std::to_string(error.val);
      return result;
    }
    if (plan.trajectory_.joint_trajectory.points.empty())
    {
      result.planning_success = false;
      result.failure_reason = "EMPTY_TRAJECTORY";
      return result;
    }

    moveit::core::RobotState final_state(robot_model_);
    std::string trajectory_failure;
    if (!trajectoryCollisionFree(current, plan.trajectory_, final_state, trajectory_failure,
                                 result.minimum_environment_clearance, result.minimum_self_clearance,
                                 result.attached_object_box_collision))
    {
      result.planning_success = false;
      result.collision_free = false;
      result.failure_reason = trajectory_failure;
      return result;
    }

    result.trajectory_points = plan.trajectory_.joint_trajectory.points.size();
    result.joint_path_length = pathLength(plan.trajectory_);
    poseError(final_state, pose, result.position_error, result.orientation_error);
    result.collision_free = true;
    result.failure_reason.clear();
    current = final_state;
    output_trajectory = plan.trajectory_;
    return result;
  }

  CandidateResult runCandidate(const std::string& mode, const Candidate& candidate)
  {
    resetSceneForCandidate();
    CandidateResult result(robot_model_);
    result.mode = mode;
    result.candidate = candidate;
    moveit::core::RobotState current = initialState(candidate);
    moveit::core::robotStateToRobotStateMsg(current, result.start_state);

    const Eigen::Isometry3d& initial_tcp = current.getGlobalLinkTransform(left_tcp_link_);
    RCLCPP_INFO(node_->get_logger(), "candidate=%s mode=%s lift=%.3f yaw=%.3f pitch=%.3f initial_tcp=[%.3f %.3f %.3f]",
                candidate.id.c_str(), mode.c_str(), candidate.lift, candidate.yaw_deg, candidate.pitch_deg,
                initial_tcp.translation().x(), initial_tcp.translation().y(), initial_tcp.translation().z());

    for (const auto& target : preExtractionStagePoses())
    {
      if (target.first == "GRASP_POSE")
        setFingerTargetContactAllowed(true);
      moveit_msgs::msg::RobotTrajectory trajectory;
      StageResult stage = planStage(mode, candidate, target.first, target.second, current, trajectory);
      result.total_planning_time_ms += stage.planning_time_ms;
      appendStageCsv(mode, candidate, stage);
      result.stages.push_back(stage);
      if (!stage.planning_success)
      {
        result.first_failure_stage = stage.stage;
        result.final_state = current;
        return result;
      }
      result.trajectories.push_back(std::move(trajectory));
      if (target.first == "GRASP_POSE")
        attachTargetAtomically(current);
    }

    // The endpoint is derived from the required final object pose and the actual transform captured at attachment.
    // It is intentionally not a fixed subtraction from the GRASP or LIFT TCP position.
    moveit_msgs::msg::RobotTrajectory extraction_trajectory;
    const geometry_msgs::msg::Pose extraction =
        extractionPose(resolved_extraction_clearance_, attached_target_in_tcp_);
    StageResult extraction_stage = planStage(mode, candidate, "EXTRACTION", extraction, current, extraction_trajectory);
    result.total_planning_time_ms += extraction_stage.planning_time_ms;
    appendStageCsv(mode, candidate, extraction_stage);
    result.stages.push_back(extraction_stage);
    if (!extraction_stage.planning_success)
    {
      result.first_failure_stage = extraction_stage.stage;
      result.final_state = current;
      return result;
    }
    result.trajectories.push_back(std::move(extraction_trajectory));
    result.success = true;
    result.final_state = current;
    return result;
  }

  void initializeCsv() const
  {
    std::ofstream output(output_csv_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot create output CSV: " + output_csv_);
    output << "timestamp,planning_attempt_id,planning_time_limit,number_of_attempts,mode,candidate_id,lift,yaw,pitch,stage,joint_limit_valid,ik_success,collision_free,"
              "planning_success,failure_reason,planning_time_ms,trajectory_points,joint_path_length,"
              "end_effector_position_error,end_effector_orientation_error,ik_seeds_tested,collision_free_ik_count,"
              "ik_failure_pairs\n";
  }

  void appendStageCsv(const std::string& mode, const Candidate& candidate, const StageResult& result) const
  {
    std::ofstream output(output_csv_, std::ios::app);
    output << csvEscape(timestampNow()) << ',' << csvEscape(planning_attempt_id_) << ',' << scene_config_.planning_time
           << ',' << scene_config_.planning_attempts << ',' << csvEscape(mode) << ',' << csvEscape(candidate.id) << ','
           << std::setprecision(15) << candidate.lift << ',' << candidate.yaw << ',' << candidate.pitch << ','
           << csvEscape(result.stage) << ',' << (result.joint_limit_valid ? 1 : 0) << ','
           << (result.ik_success ? 1 : 0) << ',' << (result.collision_free ? 1 : 0) << ','
           << (result.planning_success ? 1 : 0) << ',' << csvEscape(result.failure_reason) << ','
           << result.planning_time_ms << ',' << result.trajectory_points << ',' << result.joint_path_length << ','
           << result.position_error << ',' << result.orientation_error << ',' << result.ik_seeds_tested << ','
           << result.collision_free_ik_count << ',' << csvEscape(pairString(result.ik_failure_pairs)) << '\n';
  }

  void initializeBoundaryCsvs() const
  {
    {
      std::ofstream out(boundary_csv_, std::ios::trunc);
      out << "target_x,target_y,target_z,search_phase,geometric_feasible,infeasible_reason,geometric_pairs,"
             "lift_only_success,lift_only_structural_failure,lift_only_first_failure_stage,proposed_success,"
             "proposed_first_failure_stage,proposed_candidates_tested,selected_lift,selected_yaw,selected_pitch,"
             "torso_recovery_success\n";
    }
    {
      std::ofstream out(comparison_csv_, std::ios::trunc);
      out << "target_x,target_y,target_z,mode,lift,yaw,pitch,candidate_id,ik_seeds_tested,collision_free_ik_count,"
             "planning_attempts,planning_time_limit,planning_success,structural_stage_failure,first_failure_stage,"
             "colliding_link_pairs\n";
    }
    {
      std::ofstream out(ik_audit_csv_, std::ios::trunc);
      out << "timestamp,target_x,target_y,target_z,mode,lift,yaw,pitch,stage,ik_seed_id,ik_rng_seed,ik_success,"
             "joint_limit_valid,self_collision,environment_collision,collision_free,colliding_link_pairs\n";
    }
    {
      std::ofstream out(geometric_csv_, std::ios::trunc);
      out << "target_x,target_y,target_z,geometric_feasible,infeasible_reason,stage,ik_seeds_tested,"
             "colliding_link_pairs\n";
    }
  }

  void initializeRepeatCsvs() const
  {
    std::ofstream out(repeat_trials_csv_, std::ios::trunc);
    if (!out)
      throw std::runtime_error("Cannot create repeat trials CSV: " + repeat_trials_csv_);
    out << "timestamp,target_id,target_x,target_y,target_z,mode,time_budget,repeat_id,planning_attempts,stage,"
           "planning_success,planning_time_ms,first_failure_stage,trajectory_points,joint_path_length,"
           "minimum_environment_clearance,minimum_self_clearance,selected_lift,selected_yaw,selected_pitch,"
           "attached_object_box_collision,failure_reason,colliding_link_pairs\n";
  }

  void writeBoundarySummary(bool found, const std::string& phase, const ModeEvaluation& baseline,
                            const ModeEvaluation& proposed) const
  {
    std::ofstream out(summary_path_, std::ios::trunc);
    out << "# Lift-only versus Lift-Yaw-Pitch boundary search\n\n";
    out << "Generated: " << timestampNow() << "\n\n";
    out << "- Previous `TORSO_LOCKED` results are preserved as **LEGACY_FIXED_LIFT** because Lift was fixed at q=0.\n";
    const auto& lift_bounds = robot_model_->getVariableBounds("lift_joint");
    out << "- Runtime URDF Lift limit: `" << lift_bounds.min_position_ << " .. " << lift_bounds.max_position_
        << " m`; q=0 is top and positive motion is downward.\n";
    out << "- Local Lift search candidates:";
    for (const double lift : liftCandidates())
      out << ' ' << lift;
    out << " m.\n";
    out << "- IK multistart: `" << scene_config_.ik_multistart_count << "` explicit seeds per stage; RNG base seed `"
        << scene_config_.ik_rng_seed << "`.\n";
    out << "- Planning time/attempts are identical in both modes: `" << scene_config_.planning_time << " s / "
        << scene_config_.planning_attempts << "`.\n";
    out << "- Search phase: `" << phase << "`.\n";
    out << "- Recovery found: **" << (found ? "YES" : "NO") << "**.\n";
    if (found)
    {
      out << "- Target XYZ: `" << scene_config_.target_position[0] << ' ' << scene_config_.target_position[1] << ' '
          << scene_config_.target_position[2] << "` m.\n";
      out << "- LIFT_ONLY structural failure: `" << (baseline.structural_failure ? "true" : "false")
          << "`, first failure stage `" << baseline.first_failure_stage << "`.\n";
      out << "- LIFT_YAW_PITCH success: `" << (proposed.success ? "true" : "false") << "`.\n";
      out << "- Selected Lift/Yaw/Pitch: `" << proposed.best.candidate.lift << " m / "
          << proposed.best.candidate.yaw_deg << " deg / " << proposed.best.candidate.pitch_deg << " deg`.\n";
    }
    out << "\nNo trajectory was executed. No controller, ros2_control, hardware interface, serial, CAN, USB, or real robot "
           "node was used. Geometry and experiment values remain provisional.\n";
  }

  void appendExcludedCandidate(const Candidate& candidate) const
  {
    StageResult result;
    result.stage = "CANDIDATE_FILTER";
    result.failure_reason = "OUTSIDE_EXACT_URDF_LIMIT_NO_CLAMP";
    appendStageCsv("TORSO_CANDIDATE_SEARCH", candidate, result);
  }

  void appendTargetHistory(bool locked_success, bool candidate_success, const CandidateResult& best,
                           std::size_t executed, bool desired_case) const
  {
    const bool exists = std::filesystem::exists(target_history_csv_);
    std::ofstream output(target_history_csv_, std::ios::app);
    if (!exists || std::filesystem::file_size(target_history_csv_) == 0)
      output << "timestamp,target_x,target_y,target_z,locked_success,candidate_success,selected_candidate,"
                "executed_candidates,desired_case\n";
    output << csvEscape(timestampNow()) << ',' << scene_config_.target_position[0] << ','
           << scene_config_.target_position[1] << ',' << scene_config_.target_position[2] << ','
           << (locked_success ? 1 : 0) << ',' << (candidate_success ? 1 : 0) << ','
           << csvEscape(candidate_success ? best.candidate.id : "") << ',' << executed << ','
           << (desired_case ? 1 : 0) << '\n';
  }

  void writeSummary(const CandidateResult& locked, const CandidateResult& best, bool candidate_success,
                    std::size_t executed, std::size_t excluded, bool desired_case) const
  {
    std::ofstream out(summary_path_, std::ios::trunc);
    out << "# Single-case extraction summary\n\n";
    out << "Generated: " << timestampNow() << "\n\n";
    out << "All scene, object, pose, and task-distance values are **PROVISIONAL_DEVELOPMENT_VALUE** and are not "
           "final paper data.\n\n";
    out << "## Scene\n\n";
    out << "- Frame: `" << scene_config_.frame_id << "`\n";
    out << "- Box center [m]: `" << scene_config_.box_center[0] << ' ' << scene_config_.box_center[1] << ' '
        << scene_config_.box_center[2] << "`\n";
    out << "- Interior width/depth/height [m]: `" << scene_config_.box_width << ' ' << scene_config_.box_depth
        << ' ' << scene_config_.box_height << "`\n";
    out << "- Wall/floor thickness [m]: `" << scene_config_.wall_thickness << ' '
        << scene_config_.floor_thickness << "`\n";
    out << "- Target position [m]: `" << scene_config_.target_position[0] << ' '
        << scene_config_.target_position[1] << ' ' << scene_config_.target_position[2] << "`\n";
    out << "- Target size [m]: `" << scene_config_.target_size[0] << ' ' << scene_config_.target_size[1] << ' '
        << scene_config_.target_size[2] << "`\n";
    out << "- TCP to grasp center [m]: `" << scene_config_.tcp_to_grasp_center << "`\n";
    out << "- Selected pre-grasp clearance [m]: `" << resolved_pregrasp_clearance_ << "`\n";
    out << "- Insertion/lift [m]: `" << scene_config_.insertion_offset << ' '
        << scene_config_.lift_distance << "`\n";
    out << "- Selected extraction clearance [m]: `" << resolved_extraction_clearance_ << "`\n";
    out << "- Box front plane X [m]: `" << boxFrontX() << "`\n";
    const Eigen::Vector3d extracted_center = extractedObjectCenter(resolved_extraction_clearance_);
    out << "- Final extracted object center [m]: `" << extracted_center.x() << ' ' << extracted_center.y() << ' '
        << extracted_center.z() << "`\n";
    out << "- Legacy fixed extraction_distance [m]: `" << scene_config_.legacy_extraction_distance
        << "` (**DEPRECATED_NOT_USED**)\n";
    out << "- Planning attempt ID/time limit/number of attempts: `" << planning_attempt_id_ << " / "
        << scene_config_.planning_time << " s / " << scene_config_.planning_attempts << "`\n";
    out << "- Simulation planning aperture q [m]: `" << scene_config_.left_finger << "`\n\n";
    out << "## Result\n\n";
    out << "- TORSO_LOCKED: **" << (locked.success ? "SUCCESS" : "FAILURE") << "**\n";
    out << "- TORSO_LOCKED first failure stage: `"
        << (locked.first_failure_stage.empty() ? "none" : locked.first_failure_stage) << "`\n";
    out << "- TORSO_CANDIDATE_SEARCH: **" << (candidate_success ? "SUCCESS" : "FAILURE") << "**\n";
    out << "- Successful candidate: `" << (candidate_success ? best.candidate.id : "none") << "`\n";
    if (candidate_success)
      out << "- Successful Yaw/Pitch: `" << best.candidate.yaw_deg << " deg / " << best.candidate.pitch_deg
          << " deg`\n";
    out << "- Executed candidates: " << executed << "\n";
    out << "- Excluded out-of-limit candidates: " << excluded << " (no clamping)\n";
    out << "- Desired locked-failure / torso-recovery case: **" << (desired_case ? "YES" : "NO") << "**\n\n";
    out << "## Safety and limitations\n\n";
    out << "No trajectory was executed. No ros2_control, controller, hardware interface, or real robot node was used. "
           "The target was attached only as a PlanningScene collision object; object dynamics, physical contact, grasp "
           "closure, and real grasp success were not validated. Both gripper independent joints remained at the "
           "simulation planning aperture 0.044 m; this is "
           "not asserted to be a hardware-validated fully-open position. The target remained a world collision object "
           "through GRASP_POSE and was attached atomically for LIFT and EXTRACTION.\n";
  }

  void publishResult(const CandidateResult& result)
  {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = robot_model_->getName();
    display.trajectory_start = result.start_state;
    display.trajectory = result.trajectories;
    display_publisher_->publish(display);
    publishState(result.final_state);
  }

  void publishStateOnce(const moveit::core::RobotState& state) const
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = node_->now();
    message.name = whole_body_group_->getVariableNames();
    message.position.reserve(message.name.size());
    for (const auto& name : message.name)
      message.position.push_back(state.getVariablePosition(name));
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      buffered_joint_state_ = message;
    }
    joint_state_publisher_->publish(message);
    rclcpp::sleep_for(std::chrono::milliseconds(60));
  }

  void publishBufferedState() const
  {
    sensor_msgs::msg::JointState message;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (buffered_joint_state_.name.empty())
        return;
      message = buffered_joint_state_;
    }
    message.header.stamp = node_->now();
    joint_state_publisher_->publish(message);
  }

  void publishState(const moveit::core::RobotState& state)
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = node_->now();
    message.name = whole_body_group_->getVariableNames();
    message.position.reserve(message.name.size());
    for (const auto& name : message.name)
      message.position.push_back(state.getVariablePosition(name));
    for (int i = 0; i < 3; ++i)
    {
      joint_state_publisher_->publish(message);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  rclcpp::Node::SharedPtr node_;
  SceneConfig scene_config_;
  std::string candidate_config_path_;
  std::string output_csv_;
  std::string summary_path_;
  std::string target_history_csv_;
  std::string extraction_clearance_csv_;
  std::string scene_translation_csv_;
  std::string planning_attempt_id_;
  std::string boundary_csv_;
  std::string comparison_csv_;
  std::string ik_audit_csv_;
  std::string geometric_csv_;
  std::string comparison_input_csv_;
  std::string repeat_trials_csv_;
  std::string budget_sensitivity_csv_;
  std::string analysis_path_;
  std::map<std::string, std::vector<TrialMetric>> trial_metrics_;
  bool hold_for_rviz_{ false };
  const std::string left_tcp_link_{ "openarm_left_hand_tcp" };
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* whole_body_group_{ nullptr };
  const moveit::core::JointModelGroup* left_arm_group_{ nullptr };
  planning_scene::PlanningScenePtr local_scene_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_interface_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<moveit_msgs::msg::CollisionObject> scene_objects_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  mutable std::mutex joint_state_mutex_;
  mutable sensor_msgs::msg::JointState buffered_joint_state_;
  double resolved_pregrasp_clearance_{ 0.0 };
  double resolved_extraction_clearance_{ 0.0 };
  Eigen::Isometry3d attached_target_in_tcp_{ Eigen::Isometry3d::Identity() };
  std::unique_ptr<CandidateResult> selected_result_;
  std::vector<double> selected_target_;
  ObjectPhase object_phase_{ ObjectPhase::WORLD_STRICT };
  const std::string target_object_id_{ "target_object" };
  const std::array<std::string, 2> left_finger_links_{ "openarm_left_left_finger",
                                                       "openarm_left_right_finger" };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("differential_repeat_trials", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  int exit_code = 1;
  try
  {
    auto experiment = std::make_shared<DifferentialRepeatTrials>(node);
    const bool desired_case = experiment->run();
    if (experiment->holdForRviz() && desired_case)
    {
      RCLCPP_INFO(node->get_logger(), "RViz hold active. No trajectory execution is available. Stop with Ctrl-C.");
      while (rclcpp::ok())
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    exit_code = desired_case ? 0 : 2;
  }
  catch (const std::exception& error)
  {
    std::cerr << "single_case_extraction fatal: " << error.what() << std::endl;
  }
  rclcpp::shutdown();
  executor.cancel();
  if (spin_thread.joinable())
    spin_thread.join();
  return exit_code;
}
