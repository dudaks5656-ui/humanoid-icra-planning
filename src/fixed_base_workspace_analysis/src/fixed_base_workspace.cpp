#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

namespace fixed_base_workspace
{
using Clock = std::chrono::steady_clock;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

enum class Configuration { LIFT_ONLY, LIFT_YAW, LIFT_PITCH, LIFT_YAW_PITCH };

struct TorsoCandidate
{
  double lift{};
  double yaw{};
  double pitch{};
};

struct Point
{
  std::size_t id{};
  int i{};
  int j{};
  int k{};
  Eigen::Vector3d xyz{ Eigen::Vector3d::Zero() };
};

struct JacobianMetrics
{
  double manipulability{ kNaN };
  double minimum_singular_value{ kNaN };
  double condition_number{ kNaN };
};

struct Result
{
  Point point;
  Configuration configuration{ Configuration::LIFT_ONLY };
  bool success{ false };
  std::string failure_reason{ "INTERNAL_ERROR" };
  int seeds_tested{};
  int valid_count{};
  int selected_seed{ -1 };
  double lift{ kNaN };
  double yaw{ kNaN };
  double pitch{ kNaN };
  double joint_margin{ kNaN };
  double active_revolute_margin{ kNaN };
  double self_clearance{ kNaN };
  double manipulability{ kNaN };
  double min_singular_value{ kNaN };
  double condition_number{ kNaN };
  double orientation_error{ kNaN };
  double torso_displacement{ kInf };
  std::string collision_pairs;
  double runtime_ms{};
};

struct Bounds3
{
  Eigen::Vector3d minimum{ Eigen::Vector3d::Constant(kInf) };
  Eigen::Vector3d maximum{ Eigen::Vector3d::Constant(-kInf) };
};

std::string configName(Configuration config)
{
  if (config == Configuration::LIFT_ONLY) return "LIFT_ONLY";
  if (config == Configuration::LIFT_YAW) return "LIFT_YAW";
  if (config == Configuration::LIFT_PITCH) return "LIFT_PITCH";
  return "LIFT_YAW_PITCH";
}

std::string csvEscape(const std::string& value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos)
    return value;
  std::string out = "\"";
  for (const char c : value)
    out += c == '\"' ? "\"\"" : std::string(1, c);
  return out + "\"";
}

std::string number(double value)
{
  if (!std::isfinite(value))
    return "";
  std::ostringstream out;
  out << std::setprecision(15) << value;
  return out.str();
}

std::string isoTimestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&time, &utc);
  std::ostringstream out;
  out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
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

double median(std::vector<double> values)
{
  values.erase(std::remove_if(values.begin(), values.end(), [](double x) { return !std::isfinite(x); }), values.end());
  if (values.empty())
    return kNaN;
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  return values.size() % 2 ? values[middle] : 0.5 * (values[middle - 1] + values[middle]);
}

double mean(const std::vector<double>& values)
{
  double sum = 0.0;
  std::size_t count = 0;
  for (const double value : values)
    if (std::isfinite(value))
    {
      sum += value;
      ++count;
    }
  return count ? sum / static_cast<double>(count) : kNaN;
}

double minimum(const std::vector<double>& values)
{
  double result = kInf;
  for (const double value : values)
    if (std::isfinite(value))
      result = std::min(result, value);
  return std::isfinite(result) ? result : kNaN;
}

class Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    loadParameters();
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    model_ = loader_->getModel();
    if (!model_)
      throw std::runtime_error("RobotModel/SRDF could not be loaded");
    arm_group_ = requiredGroup(arm_group_name_);
    full_group_ = requiredGroup(full_group_name_);
    if (!arm_group_->getSolverInstance())
      throw std::runtime_error("left_arm IK solver is unavailable");
    base_link_ = requiredLink(base_frame_);
    tcp_link_ = requiredLink(tcp_frame_);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    validateModelContract();
  }

  void run()
  {
    start_time_ = Clock::now();
    timestamp_ = isoTimestamp();
    std::filesystem::create_directories(output_dir_);
    initializeCollisionCsv();

    lift_values_ = interiorValues("lift_joint", lift_candidate_count_, false);
    yaw_values_ = interiorValues("waist_yaw_joint", yaw_candidate_count_, true);
    pitch_values_ = interiorValues("waist_pitch_joint", pitch_candidate_count_, true);
    lift_only_candidates_ = buildLiftOnlyCandidates();
    torso_candidates_ = buildTorsoCandidates();
    bounds_ = automaticBoundingBox();
    points_ = buildGrid();
    preflight();

    results_.reserve(points_.size() * 2);
    evaluateConfiguration(Configuration::LIFT_ONLY, lift_only_candidates_);
    evaluateConfiguration(Configuration::LIFT_YAW_PITCH, torso_candidates_);
    writeAllOutputs();
  }

protected:
  template <typename T>
  T parameter(const std::string& name)
  {
    if (!node_->has_parameter(name))
      node_->declare_parameter<T>(name);
    return node_->get_parameter(name).get_value<T>();
  }

  void loadParameters()
  {
    output_dir_ = parameter<std::string>("output_dir");
    base_frame_ = parameter<std::string>("base_frame");
    tcp_frame_ = parameter<std::string>("tcp_frame");
    arm_group_name_ = parameter<std::string>("arm_group");
    full_group_name_ = parameter<std::string>("full_group");
    grid_x_ = parameter<int>("grid_x");
    grid_y_ = parameter<int>("grid_y");
    grid_z_ = parameter<int>("grid_z");
    max_ik_seeds_ = parameter<int>("max_ik_seeds");
    ik_timeout_s_ = parameter<double>("ik_timeout_s");
    orientation_tolerance_ = parameter<double>("orientation_tolerance_rad");
    exact_bound_epsilon_ = parameter<double>("exact_bound_epsilon");
    target_q_ = Eigen::Quaterniond(parameter<double>("target_qw"), parameter<double>("target_qx"),
                                   parameter<double>("target_qy"), parameter<double>("target_qz"));
    target_q_.normalize();
    orientation_source_ = parameter<std::string>("orientation_source");
    lift_candidate_count_ = parameter<int>("lift_candidate_count");
    yaw_candidate_count_ = parameter<int>("yaw_candidate_count");
    pitch_candidate_count_ = parameter<int>("pitch_candidate_count");
    max_grid_points_ = parameter<int>("max_grid_points");
    max_ik_seeds_hard_ = parameter<int>("max_ik_seeds_hard");
    max_torso_candidates_ = parameter<int>("max_torso_candidates");
    max_configuration_evaluations_ = parameter<int>("max_configuration_evaluations");
    max_total_ik_attempts_ = parameter<int>("max_total_ik_attempts");
    max_wall_time_s_ = parameter<double>("max_experiment_wall_time_s");
    progress_every_ = parameter<int>("progress_every");
  }

  const moveit::core::JointModelGroup* requiredGroup(const std::string& name) const
  {
    const auto* group = model_->getJointModelGroup(name);
    if (!group)
      throw std::runtime_error("Required group not found: " + name);
    return group;
  }

  const moveit::core::LinkModel* requiredLink(const std::string& name) const
  {
    const auto* link = model_->getLinkModel(name);
    if (!link)
      throw std::runtime_error("Required link not found: " + name);
    return link;
  }

  void validateModelContract() const
  {
    const std::vector<std::string> torso{ "lift_joint", "waist_yaw_joint", "waist_pitch_joint" };
    for (const auto& name : torso)
      if (!model_->hasJointModel(name))
        throw std::runtime_error("Required torso joint missing: " + name);
    if (arm_group_->getVariableCount() != 7 || full_group_->getVariableCount() != 10)
      throw std::runtime_error("Expected 7-variable left_arm and 10-variable left_arm_with_torso groups");
    if (model_->getModelFrame().empty())
      throw std::runtime_error("Robot model frame is empty");
  }

  std::vector<double> interiorValues(const std::string& joint, int count, bool include_zero) const
  {
    if (count <= 0)
      throw std::runtime_error("Candidate count must be positive for " + joint);
    const auto& bound = model_->getVariableBounds(joint);
    if (!bound.position_bounded_)
      throw std::runtime_error("Candidate joint must have finite position bounds: " + joint);
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(count) + 1);
    for (int i = 0; i < count; ++i)
      values.push_back(bound.min_position_ + (static_cast<double>(i) + 0.5) *
                                             (bound.max_position_ - bound.min_position_) / count);
    if (include_zero && bound.min_position_ < 0.0 && bound.max_position_ > 0.0)
      values.push_back(0.0);
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end(), [](double a, double b) {
      return std::abs(a - b) < 1e-12;
    }), values.end());
    return values;
  }

  std::vector<TorsoCandidate> buildLiftOnlyCandidates() const
  {
    std::vector<TorsoCandidate> candidates;
    for (const double lift : lift_values_)
      candidates.push_back({ lift, 0.0, 0.0 });
    return candidates;
  }

  std::vector<TorsoCandidate> buildTorsoCandidates() const
  {
    std::vector<TorsoCandidate> candidates;
    // Include every lift-only torso posture first so configuration B is a true superset search.
    for (const double lift : lift_values_)
      candidates.push_back({ lift, 0.0, 0.0 });
    std::vector<TorsoCandidate> remaining;
    for (const double lift : lift_values_)
      for (const double yaw : yaw_values_)
        for (const double pitch : pitch_values_)
          if (std::abs(yaw) > 1e-12 || std::abs(pitch) > 1e-12)
            remaining.push_back({ lift, yaw, pitch });
    // Deterministic interleaving spreads the limited budget over the bounded torso lattice.
    const std::size_t stride = remaining.empty() ? 1 : remaining.size() / 2 + 1;
    std::vector<bool> used(remaining.size(), false);
    std::size_t index = 0;
    while (!remaining.empty() && candidates.size() < static_cast<std::size_t>(max_torso_candidates_))
    {
      if (!used[index])
      {
        candidates.push_back(remaining[index]);
        used[index] = true;
      }
      if (std::all_of(used.begin(), used.end(), [](bool value) { return value; }))
        break;
      index = (index + stride) % remaining.size();
      if (used[index])
        index = (index + 1) % remaining.size();
    }
    return candidates;
  }

  moveit::core::RobotState nominalState() const
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    // Set only the two independent finger variables; RobotState updates their mimic joints.
    for (const std::string finger : { "openarm_left_finger_joint1", "openarm_right_finger_joint1" })
    {
      const auto& bound = model_->getVariableBounds(finger);
      if (bound.position_bounded_)
        state.setVariablePosition(finger, 0.5 * (bound.min_position_ + bound.max_position_));
    }
    state.update();
    return state;
  }

  Bounds3 automaticBoundingBox()
  {
    moveit::core::RobotState state = nominalState();
    Bounds3 shoulder_bounds;
    const auto* shoulder_link = requiredLink("openarm_left_link0");
    for (const auto& candidate : torso_candidates_)
    {
      setTorso(state, candidate);
      state.update();
      const Eigen::Isometry3d base_in_model = state.getGlobalLinkTransform(base_link_);
      const Eigen::Vector3d shoulder_in_base = base_in_model.inverse() *
        state.getGlobalLinkTransform(shoulder_link).translation();
      shoulder_bounds.minimum = shoulder_bounds.minimum.cwiseMin(shoulder_in_base);
      shoulder_bounds.maximum = shoulder_bounds.maximum.cwiseMax(shoulder_in_base);
    }

    state = nominalState();
    state.update();
    const std::vector<const moveit::core::LinkModel*> links = arm_group_->getLinkModels();
    Eigen::Vector3d previous = state.getGlobalLinkTransform(shoulder_link).translation();
    arm_reach_radius_ = 0.0;
    for (const auto* link : links)
    {
      const Eigen::Vector3d current = state.getGlobalLinkTransform(link).translation();
      if ((current - previous).norm() > 1e-12)
        arm_reach_radius_ += (current - previous).norm();
      previous = current;
    }
    const Eigen::Vector3d tcp = state.getGlobalLinkTransform(tcp_link_).translation();
    arm_reach_radius_ += (tcp - previous).norm();
    if (!(arm_reach_radius_ > 0.0 && std::isfinite(arm_reach_radius_)))
      throw std::runtime_error("Failed to derive a finite arm reach radius from RobotModel geometry");

    Bounds3 result;
    result.minimum = shoulder_bounds.minimum - Eigen::Vector3d::Constant(arm_reach_radius_);
    result.maximum = shoulder_bounds.maximum + Eigen::Vector3d::Constant(arm_reach_radius_);
    return result;
  }

  std::vector<Point> buildGrid()
  {
    const Eigen::Vector3d size = bounds_.maximum - bounds_.minimum;
    voxel_ = Eigen::Vector3d(size.x() / grid_x_, size.y() / grid_y_, size.z() / grid_z_);
    std::vector<Point> points;
    points.reserve(static_cast<std::size_t>(grid_x_) * grid_y_ * grid_z_);
    std::size_t id = 0;
    for (int i = 0; i < grid_x_; ++i)
      for (int j = 0; j < grid_y_; ++j)
        for (int k = 0; k < grid_z_; ++k)
        {
          Point point;
          point.id = id++;
          point.i = i;
          point.j = j;
          point.k = k;
          point.xyz = bounds_.minimum + Eigen::Vector3d((i + 0.5) * voxel_.x(), (j + 0.5) * voxel_.y(),
                                                        (k + 0.5) * voxel_.z());
          points.push_back(point);
        }
    return points;
  }

  void preflight() const
  {
    if (grid_x_ <= 0 || grid_y_ <= 0 || grid_z_ <= 0)
      throw std::runtime_error("Grid dimensions must be positive");
    if (max_ik_seeds_ <= 0 || max_ik_seeds_ > max_ik_seeds_hard_)
      throw std::runtime_error("max_ik_seeds violates its hard cap");
    if (points_.size() > static_cast<std::size_t>(max_grid_points_))
      throw std::runtime_error("Physical grid point count exceeds max_grid_points");
    const std::size_t config_evaluations = points_.size() * 2;
    if (config_evaluations > static_cast<std::size_t>(max_configuration_evaluations_))
      throw std::runtime_error("Configuration evaluation count exceeds hard cap");
    const std::size_t max_attempts = config_evaluations * static_cast<std::size_t>(max_ik_seeds_);
    if (max_attempts > static_cast<std::size_t>(max_total_ik_attempts_))
      throw std::runtime_error("Maximum IK attempts exceed hard cap");
    if (torso_candidates_.size() > static_cast<std::size_t>(max_torso_candidates_))
      throw std::runtime_error("Torso candidate count exceeds hard cap");

    RCLCPP_INFO(node_->get_logger(),
      "FIXED_BASE_WORKSPACE PREFLIGHT grid=%dx%dx%d physical_points=%zu configurations=2 "
      "configuration_evaluations=%zu max_ik_attempts=%zu bbox=[%.4f %.4f %.4f]-[%.4f %.4f %.4f]",
      grid_x_, grid_y_, grid_z_, points_.size(), config_evaluations, max_attempts,
      bounds_.minimum.x(), bounds_.minimum.y(), bounds_.minimum.z(),
      bounds_.maximum.x(), bounds_.maximum.y(), bounds_.maximum.z());
  }

  void setTorso(moveit::core::RobotState& state, const TorsoCandidate& candidate) const
  {
    state.setVariablePosition("lift_joint", candidate.lift);
    state.setVariablePosition("waist_yaw_joint", candidate.yaw);
    state.setVariablePosition("waist_pitch_joint", candidate.pitch);
  }

  void seedArm(moveit::core::RobotState& state, std::size_t seed_index, std::size_t point_id) const
  {
    static const int primes[] = { 2, 3, 5, 7, 11, 13, 17 };
    const auto& variables = arm_group_->getVariableNames();
    for (std::size_t joint = 0; joint < variables.size(); ++joint)
    {
      const auto& bound = model_->getVariableBounds(variables[joint]);
      const double u = halton(1 + seed_index + point_id * static_cast<std::size_t>(max_ik_seeds_), primes[joint]);
      const double inset = std::max(exact_bound_epsilon_ * 10.0,
                                    (bound.max_position_ - bound.min_position_) * 1e-6);
      state.setVariablePosition(variables[joint], bound.min_position_ + inset + u *
        (bound.max_position_ - bound.min_position_ - 2.0 * inset));
    }
  }

  geometry_msgs::msg::Pose targetPoseInModel(const Point& point, moveit::core::RobotState& reference) const
  {
    reference.update();
    Eigen::Isometry3d target_in_base = Eigen::Isometry3d::Identity();
    target_in_base.linear() = target_q_.toRotationMatrix();
    target_in_base.translation() = point.xyz;
    const Eigen::Isometry3d target_in_model = reference.getGlobalLinkTransform(base_link_) * target_in_base;
    const Eigen::Quaterniond q(target_in_model.rotation());
    geometry_msgs::msg::Pose pose;
    pose.position.x = target_in_model.translation().x();
    pose.position.y = target_in_model.translation().y();
    pose.position.z = target_in_model.translation().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  double jointMargin(const moveit::core::RobotState& state, const std::vector<std::string>& names) const
  {
    double margin = kInf;
    for (const auto& name : names)
    {
      const auto& bound = model_->getVariableBounds(name);
      if (bound.position_bounded_)
      {
        const double q = state.getVariablePosition(name);
        margin = std::min(margin, std::min(q - bound.min_position_, bound.max_position_ - q));
      }
    }
    return margin;
  }

  double activeRevoluteMargin(const moveit::core::RobotState& state,
                              const std::vector<std::string>& names) const
  {
    double margin = kInf;
    for (const auto& name : names)
    {
      const auto* joint = model_->getJointOfVariable(name);
      if (!joint || joint->getType() != moveit::core::JointModel::REVOLUTE)
        continue;
      const auto& bound = model_->getVariableBounds(name);
      if (bound.position_bounded_)
      {
        const double q = state.getVariablePosition(name);
        margin = std::min(margin, std::min(q - bound.min_position_, bound.max_position_ - q));
      }
    }
    return margin;
  }

  std::string collisionPairs(const collision_detection::CollisionResult& collision) const
  {
    std::set<std::pair<std::string, std::string>> pairs;
    for (const auto& entry : collision.contacts)
      pairs.insert(entry.first);
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

  double orientationError(const moveit::core::RobotState& state) const
  {
    const Eigen::Quaterniond actual(state.getGlobalLinkTransform(tcp_link_).rotation());
    moveit::core::RobotState reference = nominalState();
    const Eigen::Quaterniond base_rotation(reference.getGlobalLinkTransform(base_link_).rotation());
    const Eigen::Quaterniond expected = base_rotation * target_q_;
    const double dot = std::clamp(std::abs(actual.normalized().dot(expected.normalized())), 0.0, 1.0);
    return 2.0 * std::acos(dot);
  }

  JacobianMetrics jacobianMetrics(moveit::core::RobotState state, Configuration config) const
  {
    state.update();
    Eigen::MatrixXd full;
    if (!state.getJacobian(full_group_, tcp_link_, Eigen::Vector3d::Zero(), full, false))
      return {};
    Eigen::MatrixXd active;
    if (config == Configuration::LIFT_YAW_PITCH)
      active = full;
    else
    {
      const auto& full_names = full_group_->getVariableNames();
      const auto active_names = activeNames(config);
      const std::set<std::string> wanted(active_names.begin(), active_names.end());
      std::vector<Eigen::Index> columns;
      for (std::size_t i = 0; i < full_names.size(); ++i)
        if (wanted.count(full_names[i]))
          columns.push_back(static_cast<Eigen::Index>(i));
      active.resize(full.rows(), static_cast<Eigen::Index>(columns.size()));
      for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(columns.size()); ++i)
        active.col(i) = full.col(columns[static_cast<std::size_t>(i)]);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(active, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = svd.singularValues();
    JacobianMetrics result;
    result.manipulability = 1.0;
    for (Eigen::Index i = 0; i < singular.size(); ++i)
      result.manipulability *= singular[i];
    result.minimum_singular_value = singular.size() ? singular.minCoeff() : kNaN;
    const double maximum = singular.size() ? singular.maxCoeff() : kNaN;
    result.condition_number = result.minimum_singular_value > 1e-15 ?
      maximum / result.minimum_singular_value : kInf;
    return result;
  }

  std::vector<std::string> activeNames(Configuration config) const
  {
    if (config == Configuration::LIFT_YAW_PITCH)
      return full_group_->getVariableNames();
    std::vector<std::string> names{ "lift_joint" };
    if (config == Configuration::LIFT_YAW) names.push_back("waist_yaw_joint");
    if (config == Configuration::LIFT_PITCH) names.push_back("waist_pitch_joint");
    const auto& arm = arm_group_->getVariableNames();
    names.insert(names.end(), arm.begin(), arm.end());
    return names;
  }

  bool better(const Result& candidate, const Result& incumbent) const
  {
    if (!incumbent.success)
      return true;
    const auto greater = [](double a, double b) { return a > b + 1e-12; };
    if (greater(candidate.joint_margin, incumbent.joint_margin)) return true;
    if (greater(incumbent.joint_margin, candidate.joint_margin)) return false;
    if (greater(candidate.self_clearance, incumbent.self_clearance)) return true;
    if (greater(incumbent.self_clearance, candidate.self_clearance)) return false;
    if (greater(candidate.manipulability, incumbent.manipulability)) return true;
    if (greater(incumbent.manipulability, candidate.manipulability)) return false;
    if (candidate.torso_displacement + 1e-12 < incumbent.torso_displacement) return true;
    if (incumbent.torso_displacement + 1e-12 < candidate.torso_displacement) return false;
    return candidate.selected_seed < incumbent.selected_seed;
  }

  Result evaluatePoint(const Point& point, Configuration config,
                       const std::vector<TorsoCandidate>& torso_candidates)
  {
    const auto begin = Clock::now();
    Result best;
    best.point = point;
    best.configuration = config;
    std::map<std::string, int> failures;
    std::set<std::string> all_collision_pairs;
    const auto names = activeNames(config);

    for (int seed = 0; seed < max_ik_seeds_; ++seed)
    {
      enforceWallTime();
      moveit::core::RobotState state = nominalState();
      const TorsoCandidate& torso = torso_candidates[static_cast<std::size_t>(seed) % torso_candidates.size()];
      setTorso(state, torso);
      seedArm(state, static_cast<std::size_t>(seed), point.id);
      state.update();
      const geometry_msgs::msg::Pose target = targetPoseInModel(point, state);
      ++best.seeds_tested;
      if (!state.setFromIK(arm_group_, target, tcp_frame_, ik_timeout_s_))
      {
        ++failures["NO_IK"];
        continue;
      }
      // Reassert torso values: only left_arm variables are legal IK outputs.
      setTorso(state, torso);
      state.update();
      if (!state.satisfiesBounds())
      {
        ++failures["JOINT_LIMIT_VIOLATION"];
        continue;
      }
      const double joint_margin = jointMargin(state, names);
      if (!(joint_margin > exact_bound_epsilon_))
      {
        ++failures["ACTIVE_JOINT_AT_BOUND"];
        continue;
      }
      const double angle_error = orientationError(state);
      if (!(angle_error <= orientation_tolerance_))
      {
        ++failures["ORIENTATION_ERROR"];
        continue;
      }

      collision_detection::CollisionRequest request;
      request.contacts = true;
      request.max_contacts = 1000;
      request.max_contacts_per_pair = 50;
      collision_detection::CollisionResult collision;
      scene_->checkSelfCollision(request, collision, state);
      const std::string pairs = collisionPairs(collision);
      if (collision.collision)
      {
        ++failures["SELF_COLLISION"];
        if (!pairs.empty()) all_collision_pairs.insert(pairs);
        appendCollision(point, config, seed, torso, state, pairs);
        continue;
      }

      Result valid;
      valid.point = point;
      valid.configuration = config;
      valid.success = true;
      valid.failure_reason = "REACHABLE";
      valid.selected_seed = seed;
      valid.lift = torso.lift;
      valid.yaw = torso.yaw;
      valid.pitch = torso.pitch;
      valid.joint_margin = joint_margin;
      valid.active_revolute_margin = activeRevoluteMargin(state, names);
      valid.self_clearance = scene_->getCollisionEnv()->distanceSelf(state, scene_->getAllowedCollisionMatrix());
      const JacobianMetrics jacobian = jacobianMetrics(state, config);
      valid.manipulability = jacobian.manipulability;
      valid.min_singular_value = jacobian.minimum_singular_value;
      valid.condition_number = jacobian.condition_number;
      valid.orientation_error = angle_error;
      const auto& yaw_bound = model_->getVariableBounds("waist_yaw_joint");
      const auto& pitch_bound = model_->getVariableBounds("waist_pitch_joint");
      valid.torso_displacement = std::abs(torso.yaw) / (yaw_bound.max_position_ - yaw_bound.min_position_) +
        std::abs(torso.pitch) / (pitch_bound.max_position_ - pitch_bound.min_position_);
      ++best.valid_count;
      if (better(valid, best))
      {
        const int tested = best.seeds_tested;
        const int valid_count = best.valid_count;
        best = valid;
        best.seeds_tested = tested;
        best.valid_count = valid_count;
      }
    }

    if (!best.success)
    {
      const std::vector<std::string> priority{ "SELF_COLLISION", "ACTIVE_JOINT_AT_BOUND",
        "JOINT_LIMIT_VIOLATION", "ORIENTATION_ERROR", "NO_IK", "TIMEOUT", "INTERNAL_ERROR" };
      int largest = -1;
      for (const auto& reason : priority)
        if (failures[reason] > largest)
        {
          largest = failures[reason];
          best.failure_reason = reason;
        }
      std::ostringstream pairs;
      bool first = true;
      for (const auto& value : all_collision_pairs)
      {
        if (!first) pairs << ';';
        pairs << value;
        first = false;
      }
      best.collision_pairs = pairs.str();
    }
    best.runtime_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    return best;
  }

  void evaluateConfiguration(Configuration config, const std::vector<TorsoCandidate>& torso_candidates)
  {
    RCLCPP_INFO(node_->get_logger(), "FIXED_BASE_WORKSPACE config=%s begin points=%zu seeds=%d torso_candidates=%zu",
                configName(config).c_str(), points_.size(), max_ik_seeds_, torso_candidates.size());
    for (std::size_t index = 0; index < points_.size(); ++index)
    {
      Result result = evaluatePoint(points_[index], config, torso_candidates);
      results_.push_back(result);
      if (index % static_cast<std::size_t>(std::max(1, progress_every_)) == 0 || index + 1 == points_.size())
      {
        RCLCPP_INFO(node_->get_logger(),
          "FIXED_BASE_WORKSPACE config=%s point=%zu/%zu xyz=(%.4f,%.4f,%.4f) result=%s reason=%s lift=%s margin=%s",
          configName(config).c_str(), index + 1, points_.size(), result.point.xyz.x(), result.point.xyz.y(),
          result.point.xyz.z(), result.success ? "REACHABLE" : "FAIL", result.failure_reason.c_str(),
          number(result.lift).c_str(), number(result.joint_margin).c_str());
      }
    }
  }

  void enforceWallTime() const
  {
    const double elapsed = std::chrono::duration<double>(Clock::now() - start_time_).count();
    if (elapsed > max_wall_time_s_)
      throw std::runtime_error("MAX_EXPERIMENT_WALL_TIME exceeded; partial outputs are intentionally not finalized");
  }

  void initializeCollisionCsv()
  {
    collision_csv_path_ = output_dir_ + "/fixed_base_workspace_collisions.csv";
    collision_csv_.open(collision_csv_path_, std::ios::trunc);
    if (!collision_csv_)
      throw std::runtime_error("Cannot create collision audit CSV");
    collision_csv_ << "timestamp,point_id,configuration,seed_index,tcp_x,tcp_y,tcp_z,lift,yaw,pitch,";
    for (const auto& name : arm_group_->getVariableNames())
      collision_csv_ << name << ',';
    collision_csv_ << "collision_pairs\n";
  }

  void appendCollision(const Point& point, Configuration config, int seed, const TorsoCandidate& torso,
                       const moveit::core::RobotState& state, const std::string& pairs)
  {
    collision_csv_ << timestamp_ << ',' << point.id << ',' << configName(config) << ',' << seed << ','
                   << number(point.xyz.x()) << ',' << number(point.xyz.y()) << ',' << number(point.xyz.z()) << ','
                   << number(torso.lift) << ',' << number(torso.yaw) << ',' << number(torso.pitch) << ',';
    for (const auto& name : arm_group_->getVariableNames())
      collision_csv_ << number(state.getVariablePosition(name)) << ',';
    collision_csv_ << csvEscape(pairs) << '\n';
  }

  const Result& resultFor(std::size_t point_id, Configuration config) const
  {
    const std::size_t offset = config == Configuration::LIFT_ONLY ? 0 : points_.size();
    return results_.at(offset + point_id);
  }

  void writePointsCsv() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_points.csv", std::ios::trunc);
    out << "timestamp,point_id,grid_i,grid_j,grid_k,configuration,tcp_x,tcp_y,tcp_z,success,"
           "failure_reason,ik_seeds_tested,valid_ik_count,selected_lift,selected_yaw,selected_pitch,"
           "min_joint_limit_margin,min_active_revolute_margin,self_collision_clearance,manipulability,"
           "min_jacobian_singular_value,jacobian_condition_number,orientation_error,collision_pairs,runtime_ms\n";
    for (const auto& result : results_)
      out << timestamp_ << ',' << result.point.id << ',' << result.point.i << ',' << result.point.j << ','
          << result.point.k << ',' << configName(result.configuration) << ',' << number(result.point.xyz.x()) << ','
          << number(result.point.xyz.y()) << ',' << number(result.point.xyz.z()) << ',' << (result.success ? 1 : 0)
          << ',' << result.failure_reason << ',' << result.seeds_tested << ',' << result.valid_count << ','
          << number(result.lift) << ',' << number(result.yaw) << ',' << number(result.pitch) << ','
          << number(result.joint_margin) << ',' << number(result.active_revolute_margin) << ','
          << number(result.self_clearance) << ',' << number(result.manipulability) << ','
          << number(result.min_singular_value) << ',' << number(result.condition_number) << ','
          << number(result.orientation_error) << ',' << csvEscape(result.collision_pairs) << ','
          << number(result.runtime_ms) << '\n';
  }

  std::string classification(const Result& lift, const Result& torso) const
  {
    if (lift.success && torso.success) return "COMMON_REACHABLE";
    if (!lift.success && torso.success) return "TORSO_EXPANDED";
    if (lift.success && !torso.success) return "LIFT_ONLY_ONLY";
    return "UNREACHABLE_BOTH";
  }

  void writeComparisonCsv() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_comparison.csv", std::ios::trunc);
    out << "point_id,tcp_x,tcp_y,tcp_z,lift_only_success,lift_yaw_pitch_success,classification,"
           "lift_only_failure_reason,lift_yaw_pitch_failure_reason,lift_only_joint_margin,"
           "lift_yaw_pitch_joint_margin,lift_only_self_clearance,lift_yaw_pitch_self_clearance,"
           "lift_only_manipulability,lift_yaw_pitch_manipulability\n";
    for (const auto& point : points_)
    {
      const auto& lift = resultFor(point.id, Configuration::LIFT_ONLY);
      const auto& torso = resultFor(point.id, Configuration::LIFT_YAW_PITCH);
      out << point.id << ',' << number(point.xyz.x()) << ',' << number(point.xyz.y()) << ',' << number(point.xyz.z())
          << ',' << (lift.success ? 1 : 0) << ',' << (torso.success ? 1 : 0) << ','
          << classification(lift, torso) << ',' << lift.failure_reason << ',' << torso.failure_reason << ','
          << number(lift.joint_margin) << ',' << number(torso.joint_margin) << ','
          << number(lift.self_clearance) << ',' << number(torso.self_clearance) << ','
          << number(lift.manipulability) << ',' << number(torso.manipulability) << '\n';
    }
  }

  std::vector<const Result*> successful(Configuration config) const
  {
    std::vector<const Result*> values;
    for (const auto& point : points_)
    {
      const auto& result = resultFor(point.id, config);
      if (result.success) values.push_back(&result);
    }
    return values;
  }

  void writeSummaryCsv() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_summary.csv", std::ios::trunc);
    out << "configuration,total_points,reachable_points,reachable_rate,estimated_workspace_volume,"
           "x_min_reachable,x_max_reachable,y_min_reachable,y_max_reachable,z_min_reachable,z_max_reachable,"
           "x_span,y_span,z_span,maximum_forward_reach,maximum_backward_reach,maximum_left_reach,"
           "maximum_right_reach,mean_joint_margin,median_joint_margin,min_joint_margin,mean_self_clearance,"
           "min_self_clearance,mean_manipulability,median_manipulability,mean_runtime_ms\n";
    const double voxel_volume = voxel_.x() * voxel_.y() * voxel_.z();
    for (const Configuration config : { Configuration::LIFT_ONLY, Configuration::LIFT_YAW_PITCH })
    {
      const auto reached = successful(config);
      Bounds3 extents;
      std::vector<double> margins, clearances, manipulabilities, runtimes;
      for (const auto* result : reached)
      {
        extents.minimum = extents.minimum.cwiseMin(result->point.xyz);
        extents.maximum = extents.maximum.cwiseMax(result->point.xyz);
        margins.push_back(result->joint_margin);
        clearances.push_back(result->self_clearance);
        manipulabilities.push_back(result->manipulability);
      }
      const std::size_t offset = config == Configuration::LIFT_ONLY ? 0 : points_.size();
      for (std::size_t i = 0; i < points_.size(); ++i) runtimes.push_back(results_[offset + i].runtime_ms);
      const bool any = !reached.empty();
      Eigen::Vector3d span = Eigen::Vector3d::Constant(kNaN);
      if (any) span = extents.maximum - extents.minimum;
      out << configName(config) << ',' << points_.size() << ',' << reached.size() << ','
          << number(static_cast<double>(reached.size()) / points_.size()) << ','
          << number(reached.size() * voxel_volume) << ','
          << number(any ? extents.minimum.x() : kNaN) << ',' << number(any ? extents.maximum.x() : kNaN) << ','
          << number(any ? extents.minimum.y() : kNaN) << ',' << number(any ? extents.maximum.y() : kNaN) << ','
          << number(any ? extents.minimum.z() : kNaN) << ',' << number(any ? extents.maximum.z() : kNaN) << ','
          << number(span.x()) << ',' << number(span.y()) << ',' << number(span.z()) << ','
          << number(any ? extents.maximum.x() : kNaN) << ',' << number(any ? extents.minimum.x() : kNaN) << ','
          << number(any ? extents.maximum.y() : kNaN) << ',' << number(any ? extents.minimum.y() : kNaN) << ','
          << number(mean(margins)) << ',' << number(median(margins)) << ',' << number(minimum(margins)) << ','
          << number(mean(clearances)) << ',' << number(minimum(clearances)) << ','
          << number(mean(manipulabilities)) << ',' << number(median(manipulabilities)) << ','
          << number(mean(runtimes)) << '\n';
    }
  }

  std::string distribution(const std::vector<double>& values) const
  {
    std::map<std::string, int> counts;
    for (const double value : values) ++counts[number(value)];
    std::ostringstream out;
    bool first = true;
    for (const auto& entry : counts)
    {
      if (!first) out << ';';
      out << entry.first << ':' << entry.second;
      first = false;
    }
    return out.str();
  }

  void writeExpansionCsv() const
  {
    std::size_t common = 0, expanded = 0, lift_only_only = 0, neither = 0;
    std::vector<double> yaw, pitch, lift;
    for (const auto& point : points_)
    {
      const auto& a = resultFor(point.id, Configuration::LIFT_ONLY);
      const auto& b = resultFor(point.id, Configuration::LIFT_YAW_PITCH);
      const std::string label = classification(a, b);
      if (label == "COMMON_REACHABLE") ++common;
      else if (label == "TORSO_EXPANDED")
      {
        ++expanded;
        yaw.push_back(b.yaw); pitch.push_back(b.pitch); lift.push_back(b.lift);
      }
      else if (label == "LIFT_ONLY_ONLY") ++lift_only_only;
      else ++neither;
    }
    const double voxel_volume = voxel_.x() * voxel_.y() * voxel_.z();
    const double volume_a = successful(Configuration::LIFT_ONLY).size() * voxel_volume;
    const double volume_b = successful(Configuration::LIFT_YAW_PITCH).size() * voxel_volume;
    const double delta = volume_b - volume_a;
    const double percentage = volume_a > 0.0 ? 100.0 * delta / volume_a : kNaN;
    std::ofstream out(output_dir_ + "/fixed_base_workspace_torso_expansion.csv", std::ios::trunc);
    out << "common_reachable_count,torso_expanded_count,lift_only_only_count,unreachable_both_count,"
           "lift_only_volume,lift_yaw_pitch_volume,delta_volume,percentage_volume_increase,"
           "selected_yaw_distribution,selected_pitch_distribution,selected_lift_distribution\n";
    out << common << ',' << expanded << ',' << lift_only_only << ',' << neither << ',' << number(volume_a) << ','
        << number(volume_b) << ',' << number(delta) << ',' << number(percentage) << ','
        << csvEscape(distribution(yaw)) << ',' << csvEscape(distribution(pitch)) << ','
        << csvEscape(distribution(lift)) << '\n';
  }

  std::map<std::string, int> failureCounts(Configuration config) const
  {
    std::map<std::string, int> counts;
    for (const auto& point : points_)
      ++counts[resultFor(point.id, config).failure_reason];
    return counts;
  }

  std::string dominantFailure(Configuration config) const
  {
    const auto counts = failureCounts(config);
    std::pair<std::string, int> best{ "NONE", -1 };
    for (const auto& entry : counts)
      if (entry.first != "REACHABLE" && entry.second > best.second) best = entry;
    return best.first + " (" + std::to_string(std::max(0, best.second)) + ")";
  }

  std::string valuesString(const std::vector<double>& values) const
  {
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); ++i)
    {
      if (i) out << ", ";
      out << number(values[i]);
    }
    return out.str();
  }

  void writeAudit() const
  {
    const auto a = successful(Configuration::LIFT_ONLY);
    const auto b = successful(Configuration::LIFT_YAW_PITCH);
    const double voxel_volume = voxel_.prod();
    const double volume_a = a.size() * voxel_volume;
    const double volume_b = b.size() * voxel_volume;
    std::size_t common = 0, expanded = 0, lift_only_only = 0, neither = 0;
    std::vector<double> expanded_yaw, expanded_pitch, expanded_lift;
    for (const auto& point : points_)
    {
      const auto& ra = resultFor(point.id, Configuration::LIFT_ONLY);
      const auto& rb = resultFor(point.id, Configuration::LIFT_YAW_PITCH);
      const auto label = classification(ra, rb);
      if (label == "COMMON_REACHABLE") ++common;
      else if (label == "TORSO_EXPANDED")
      {
        ++expanded; expanded_yaw.push_back(rb.yaw); expanded_pitch.push_back(rb.pitch); expanded_lift.push_back(rb.lift);
      }
      else if (label == "LIFT_ONLY_ONLY") ++lift_only_only;
      else ++neither;
    }
    auto extents = [](const std::vector<const Result*>& reached) {
      Bounds3 bounds;
      for (const auto* result : reached)
      {
        bounds.minimum = bounds.minimum.cwiseMin(result->point.xyz);
        bounds.maximum = bounds.maximum.cwiseMax(result->point.xyz);
      }
      return bounds;
    };
    const Bounds3 ea = extents(a), eb = extents(b);
    Eigen::Vector3d span_a = Eigen::Vector3d::Constant(kNaN);
    Eigen::Vector3d span_b = Eigen::Vector3d::Constant(kNaN);
    if (!a.empty()) span_a = ea.maximum - ea.minimum;
    if (!b.empty()) span_b = eb.maximum - eb.minimum;
    const Eigen::Vector3d span_delta = span_b - span_a;
    int largest_axis = 0;
    if (span_delta.y() > span_delta.x()) largest_axis = 1;
    if (span_delta.z() > span_delta[largest_axis]) largest_axis = 2;
    const char* axis_names[] = { "X (forward/backward)", "Y (left/right)", "Z (vertical)" };
    std::vector<double> margins_a, margins_b, clearance_a, clearance_b, manip_a, manip_b;
    for (const auto* r : a) { margins_a.push_back(r->joint_margin); clearance_a.push_back(r->self_clearance); manip_a.push_back(r->manipulability); }
    for (const auto* r : b) { margins_b.push_back(r->joint_margin); clearance_b.push_back(r->self_clearance); manip_b.push_back(r->manipulability); }

    std::ofstream out(output_dir_ + "/fixed_base_workspace_audit.md", std::ios::trunc);
    out << "# Fixed-base manipulation workspace audit\n\n"
        << "Generated: " << timestamp_ << "\n\n"
        << "## Scope and model contract\n\n"
        << "- Experiment: IK-based, self-collision-aware, fixed-base workspace characterization.\n"
        << "- AMR/base fixed: **yes**; no base variables are sampled or modified.\n"
        << "- Robot model frame: `" << model_->getModelFrame() << "`\n"
        << "- Base frame and coordinate convention: `" << base_frame_ << "`; +X forward, +Y left, +Z up.\n"
        << "- TCP frame: `" << tcp_frame_ << "`\n"
        << "- TCP orientation source: `" << orientation_source_ << "`; quaternion xyzw = ["
        << target_q_.x() << ", " << target_q_.y() << ", " << target_q_.z() << ", " << target_q_.w() << "]\n"
        << "- Orientation tolerance: " << orientation_tolerance_ << " rad.\n"
        << "- IK method: bounded discrete torso candidates plus the existing `left_arm` KDL IK solver.\n"
        << "- LIFT_ONLY fixes waist yaw/pitch to exactly 0 for every seed.\n"
        << "- LIFT_YAW_PITCH samples the same lift-only postures first, then a deterministic bounded torso lattice.\n"
        << "- Environment objects: none. Existing SRDF ACM is used unchanged.\n"
        << "- Controller: none; ros2_control: none; trajectory execution: none; hardware execution: none.\n"
        << "- Joint/model/collision geometry modifications: none.\n\n"
        << "## Sampling\n\n"
        << "- Automatically derived conservative bounding box in `" << base_frame_ << "`: ["
        << number(bounds_.minimum.x()) << ", " << number(bounds_.minimum.y()) << ", " << number(bounds_.minimum.z())
        << "] to [" << number(bounds_.maximum.x()) << ", " << number(bounds_.maximum.y()) << ", "
        << number(bounds_.maximum.z()) << "] m.\n"
        << "- RobotModel-derived arm reach radius used for the box: " << number(arm_reach_radius_) << " m.\n"
        << "- Grid resolution: " << grid_x_ << " x " << grid_y_ << " x " << grid_z_ << "\n"
        << "- Voxel size: [" << number(voxel_.x()) << ", " << number(voxel_.y()) << ", " << number(voxel_.z())
        << "] m; voxel volume: " << number(voxel_volume) << " m^3.\n"
        << "- Physical points: " << points_.size() << "; configuration evaluations: " << points_.size() * 2
        << "; configured maximum IK seeds per evaluation: " << max_ik_seeds_ << ".\n"
        << "- Lift candidates (m): " << valuesString(lift_values_) << "\n"
        << "- Yaw candidates (rad): " << valuesString(yaw_values_) << "\n"
        << "- Pitch candidates (rad): " << valuesString(pitch_values_) << "\n"
        << "- Exact-bound equality epsilon: " << exact_bound_epsilon_ << " (numerical equality test only; not a feasibility threshold).\n\n"
        << "## Results\n\n"
        << "| Metric | LIFT_ONLY | LIFT_YAW_PITCH |\n|---|---:|---:|\n"
        << "| Reachable points | " << a.size() << " | " << b.size() << " |\n"
        << "| Estimated workspace volume (m^3) | " << number(volume_a) << " | " << number(volume_b) << " |\n"
        << "| X reachable min/max (m) | " << number(ea.minimum.x()) << " / " << number(ea.maximum.x()) << " | "
        << number(eb.minimum.x()) << " / " << number(eb.maximum.x()) << " |\n"
        << "| Y reachable min/max (m) | " << number(ea.minimum.y()) << " / " << number(ea.maximum.y()) << " | "
        << number(eb.minimum.y()) << " / " << number(eb.maximum.y()) << " |\n"
        << "| Z reachable min/max (m) | " << number(ea.minimum.z()) << " / " << number(ea.maximum.z()) << " | "
        << number(eb.minimum.z()) << " / " << number(eb.maximum.z()) << " |\n"
        << "| X/Y/Z span (m) | " << number(span_a.x()) << " / " << number(span_a.y()) << " / " << number(span_a.z())
        << " | " << number(span_b.x()) << " / " << number(span_b.y()) << " / " << number(span_b.z()) << " |\n"
        << "| Minimum joint margin | " << number(minimum(margins_a)) << " | " << number(minimum(margins_b)) << " |\n"
        << "| Minimum self-clearance (m) | " << number(minimum(clearance_a)) << " | " << number(minimum(clearance_b)) << " |\n"
        << "| Mean manipulability | " << number(mean(manip_a)) << " | " << number(mean(manip_b)) << " |\n"
        << "| Dominant failure reason | " << dominantFailure(Configuration::LIFT_ONLY) << " | "
        << dominantFailure(Configuration::LIFT_YAW_PITCH) << " |\n\n"
        << "- Common reachable: " << common << "\n"
        << "- Torso-expanded: " << expanded << "\n"
        << "- Lift-only-only: " << lift_only_only << " (reported without suppression)\n"
        << "- Unreachable both: " << neither << "\n"
        << "- Volume delta: " << number(volume_b - volume_a) << " m^3\n"
        << "- Percentage volume increase: " << number(volume_a > 0.0 ? 100.0 * (volume_b - volume_a) / volume_a : kNaN) << " %\n"
        << "- Largest sampled span increase direction: " << axis_names[largest_axis] << " (delta "
        << number(span_delta[largest_axis]) << " m).\n"
        << "- Torso-expanded selected yaw distribution: " << distribution(expanded_yaw) << "\n"
        << "- Torso-expanded selected pitch distribution: " << distribution(expanded_pitch) << "\n"
        << "- Torso-expanded selected lift distribution: " << distribution(expanded_lift) << "\n\n"
        << "Jacobian columns mix one prismatic lift coordinate with revolute coordinates; manipulability values are best used comparatively under this identical convention. "
           "Boundary points can be identified from successful rows with the smallest joint margin/self-clearance and failed rows classified as self-collision or bounds failures.\n\n"
        << "## Next-stage guidance\n\n"
        << "Use `TORSO_EXPANDED` points to place later boxes where waist DOF matter, `COMMON_REACHABLE` interior points for robust baseline tasks, "
           "and low-margin/low-clearance reachable neighbors for recovery scenarios. No box placement or task planning was performed in this run.\n";
  }

  void writeMetadata() const
  {
    moveit::core::RobotState state = nominalState();
    const Eigen::Isometry3d base_in_model = state.getGlobalLinkTransform(base_link_);
    std::ofstream out(output_dir_ + "/fixed_base_workspace_metadata.csv", std::ios::trunc);
    out << "key,value\n"
        << "timestamp," << timestamp_ << '\n'
        << "model_frame," << model_->getModelFrame() << '\n'
        << "base_frame," << base_frame_ << '\n'
        << "tcp_frame," << tcp_frame_ << '\n'
        << "grid_x," << grid_x_ << '\n'
        << "grid_y," << grid_y_ << '\n'
        << "grid_z," << grid_z_ << '\n'
        << "physical_points," << points_.size() << '\n'
        << "configuration_evaluations," << points_.size() * 2 << '\n'
        << "voxel_dx," << number(voxel_.x()) << '\n'
        << "voxel_dy," << number(voxel_.y()) << '\n'
        << "voxel_dz," << number(voxel_.z()) << '\n'
        << "voxel_volume," << number(voxel_.prod()) << '\n'
        << "bbox_x_min," << number(bounds_.minimum.x()) << '\n'
        << "bbox_x_max," << number(bounds_.maximum.x()) << '\n'
        << "bbox_y_min," << number(bounds_.minimum.y()) << '\n'
        << "bbox_y_max," << number(bounds_.maximum.y()) << '\n'
        << "bbox_z_min," << number(bounds_.minimum.z()) << '\n'
        << "bbox_z_max," << number(bounds_.maximum.z()) << '\n'
        << "arm_reach_radius," << number(arm_reach_radius_) << '\n'
        << "orientation_qx," << number(target_q_.x()) << '\n'
        << "orientation_qy," << number(target_q_.y()) << '\n'
        << "orientation_qz," << number(target_q_.z()) << '\n'
        << "orientation_qw," << number(target_q_.w()) << '\n'
        << "orientation_tolerance_rad," << number(orientation_tolerance_) << '\n'
        << "base_in_model_tx," << number(base_in_model.translation().x()) << '\n'
        << "base_in_model_ty," << number(base_in_model.translation().y()) << '\n'
        << "base_in_model_tz," << number(base_in_model.translation().z()) << '\n'
        << "ik_method,DISCRETE_TORSO_PLUS_LEFT_ARM_IK\n"
        << "environment_objects,0\n"
        << "trajectory_execution,false\n"
        << "controllers,false\n"
        << "ros2_control,false\n"
        << "hardware,false\n";
  }

  void writeAllOutputs()
  {
    collision_csv_.flush();
    collision_csv_.close();
    writePointsCsv();
    writeComparisonCsv();
    writeSummaryCsv();
    writeExpansionCsv();
    writeMetadata();
    writeAudit();
  }

  rclcpp::Node::SharedPtr node_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* arm_group_{ nullptr };
  const moveit::core::JointModelGroup* full_group_{ nullptr };
  const moveit::core::LinkModel* base_link_{ nullptr };
  const moveit::core::LinkModel* tcp_link_{ nullptr };
  planning_scene::PlanningScenePtr scene_;

  std::string output_dir_, base_frame_, tcp_frame_, arm_group_name_, full_group_name_;
  std::string orientation_source_, timestamp_, collision_csv_path_;
  int grid_x_{}, grid_y_{}, grid_z_{}, max_ik_seeds_{}, lift_candidate_count_{};
  int yaw_candidate_count_{}, pitch_candidate_count_{}, max_grid_points_{}, max_ik_seeds_hard_{};
  int max_torso_candidates_{}, max_configuration_evaluations_{}, max_total_ik_attempts_{}, progress_every_{};
  double ik_timeout_s_{}, orientation_tolerance_{}, exact_bound_epsilon_{}, max_wall_time_s_{};
  double arm_reach_radius_{};
  Eigen::Quaterniond target_q_{ Eigen::Quaterniond::Identity() };
  Bounds3 bounds_;
  Eigen::Vector3d voxel_{ Eigen::Vector3d::Zero() };
  std::vector<double> lift_values_, yaw_values_, pitch_values_;
  std::vector<TorsoCandidate> lift_only_candidates_, torso_candidates_;
  std::vector<Point> points_;
  std::vector<Result> results_;
  std::ofstream collision_csv_;
  Clock::time_point start_time_;
};
}  // namespace fixed_base_workspace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fixed_base_workspace");
  try
  {
    fixed_base_workspace::Runner runner(node);
    runner.run();
    RCLCPP_INFO(node->get_logger(), "FIXED_BASE_WORKSPACE batch outputs written successfully");
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "FIXED_BASE_WORKSPACE failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
