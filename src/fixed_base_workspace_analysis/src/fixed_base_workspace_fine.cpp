#include <array>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <random>
#include <unordered_map>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define main preserved_fixed_base_workspace_main
#include "fixed_base_workspace.cpp"
#undef main

namespace fixed_base_workspace_fine
{
using fixed_base_workspace::Bounds3;
using fixed_base_workspace::Clock;
using fixed_base_workspace::Configuration;
using fixed_base_workspace::JacobianMetrics;
using fixed_base_workspace::Point;
using fixed_base_workspace::Result;
using fixed_base_workspace::TorsoCandidate;
using fixed_base_workspace::configName;
using fixed_base_workspace::csvEscape;
using fixed_base_workspace::isoTimestamp;
using fixed_base_workspace::kInf;
using fixed_base_workspace::kNaN;
using fixed_base_workspace::mean;
using fixed_base_workspace::median;
using fixed_base_workspace::minimum;
using fixed_base_workspace::number;

struct CoarseRecord
{
  std::size_t point_id{};
  Configuration configuration{ Configuration::LIFT_ONLY };
  Eigen::Vector3d xyz{ Eigen::Vector3d::Zero() };
  bool success{};
  double lift{ kNaN };
  double yaw{ kNaN };
  double pitch{ kNaN };
  std::string failure;
};

struct CoarseComparison
{
  std::size_t point_id{};
  Eigen::Vector3d xyz{ Eigen::Vector3d::Zero() };
  bool lift_success{};
  bool torso_success{};
  std::string classification;
};

struct FineResult
{
  Result metrics;
  moveit::core::RobotState state;
  bool neighbor_seed_used{};
  bool neighbor_seed_success{};
  bool special_validation{};
  std::string seed_strategy;

  explicit FineResult(const moveit::core::RobotModelConstPtr& model) : state(model) {}
};

struct HighZRecord
{
  Point point;
  bool primary_a{};
  bool primary_b{};
  bool special_b_run{};
  bool final_b{};
  std::string primary_b_reason;
  std::string final_b_reason;
  int special_seeds{};
};

std::vector<std::string> parseCsvLine(const std::string& line)
{
  std::vector<std::string> fields;
  std::string current;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i)
  {
    const char c = line[i];
    if (c == '"')
    {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"')
      {
        current += '"';
        ++i;
      }
      else quoted = !quoted;
    }
    else if (c == ',' && !quoted)
    {
      fields.push_back(current);
      current.clear();
    }
    else current += c;
  }
  fields.push_back(current);
  return fields;
}

std::unordered_map<std::string, std::size_t> headerMap(const std::vector<std::string>& header)
{
  std::unordered_map<std::string, std::size_t> result;
  for (std::size_t i = 0; i < header.size(); ++i) result[header[i]] = i;
  return result;
}

double optionalDouble(const std::string& value)
{
  return value.empty() ? kNaN : std::stod(value);
}

class Runner : public fixed_base_workspace::Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node) : fixed_base_workspace::Runner(node)
  {
    loadFineParameters();
  }

  void runFine()
  {
    start_time_ = Clock::now();
    timestamp_ = isoTimestamp();
    loadCoarseEvidence();
    validateTargetedRegion();
    buildFineGrid();
    buildCandidateSets();
    preflightFine();
    initializeFineCollisionCsv();

    RCLCPP_INFO(node_->get_logger(), "FIXED_BASE_WORKSPACE_FINE config=LIFT_ONLY begin");
    results_a_ = evaluateGrid(Configuration::LIFT_ONLY);
    RCLCPP_INFO(node_->get_logger(), "FIXED_BASE_WORKSPACE_FINE config=LIFT_YAW_PITCH begin");
    results_b_ = evaluateGrid(Configuration::LIFT_YAW_PITCH);
    revalidateHighZDiscrepancies();
    validateCoarseAnomalyNeighborhood();
    writeOutputs();
    fine_collision_csv_.close();
    runPostprocessor();
  }

protected:
  void loadFineParameters()
  {
    target_bounds_.minimum = Eigen::Vector3d(parameter<double>("target_x_min"), parameter<double>("target_y_min"),
                                              parameter<double>("target_z_min"));
    target_bounds_.maximum = Eigen::Vector3d(parameter<double>("target_x_max"), parameter<double>("target_y_max"),
                                              parameter<double>("target_z_max"));
    special_ik_seeds_ = parameter<int>("special_validation_ik_seeds");
    max_special_ik_seeds_hard_ = parameter<int>("max_special_ik_seeds_hard");
    required_valid_solutions_ = parameter<int>("required_valid_solutions");
    special_required_valid_solutions_ = parameter<int>("special_required_valid_solutions");
    random_seed_ = parameter<int>("random_seed");
    high_z_min_ = parameter<double>("high_z_min");
    coarse_points_csv_ = parameter<std::string>("coarse_points_csv");
    coarse_comparison_csv_ = parameter<std::string>("coarse_comparison_csv");
    coarse_manifest_ = parameter<std::string>("coarse_manifest");
    fine_config_path_ = parameter<std::string>("fine_config_path");
    postprocess_executable_ = parameter<std::string>("postprocess_executable");
  }

  void loadCoarseEvidence()
  {
    std::ifstream points(coarse_points_csv_);
    if (!points) throw std::runtime_error("Cannot read immutable coarse points CSV");
    std::string line;
    std::getline(points, line);
    const auto point_header = headerMap(parseCsvLine(line));
    while (std::getline(points, line))
    {
      const auto row = parseCsvLine(line);
      CoarseRecord record;
      record.point_id = std::stoull(row.at(point_header.at("point_id")));
      record.configuration = row.at(point_header.at("configuration")) == "LIFT_ONLY" ?
        Configuration::LIFT_ONLY : Configuration::LIFT_YAW_PITCH;
      record.xyz = Eigen::Vector3d(std::stod(row.at(point_header.at("tcp_x"))),
                                   std::stod(row.at(point_header.at("tcp_y"))),
                                   std::stod(row.at(point_header.at("tcp_z"))));
      record.success = row.at(point_header.at("success")) == "1";
      record.failure = row.at(point_header.at("failure_reason"));
      record.lift = optionalDouble(row.at(point_header.at("selected_lift")));
      record.yaw = optionalDouble(row.at(point_header.at("selected_yaw")));
      record.pitch = optionalDouble(row.at(point_header.at("selected_pitch")));
      coarse_records_.push_back(record);
    }

    std::ifstream comparison(coarse_comparison_csv_);
    if (!comparison) throw std::runtime_error("Cannot read immutable coarse comparison CSV");
    std::getline(comparison, line);
    const auto comparison_header = headerMap(parseCsvLine(line));
    while (std::getline(comparison, line))
    {
      const auto row = parseCsvLine(line);
      CoarseComparison record;
      record.point_id = std::stoull(row.at(comparison_header.at("point_id")));
      record.xyz = Eigen::Vector3d(std::stod(row.at(comparison_header.at("tcp_x"))),
                                   std::stod(row.at(comparison_header.at("tcp_y"))),
                                   std::stod(row.at(comparison_header.at("tcp_z"))));
      record.lift_success = row.at(comparison_header.at("lift_only_success")) == "1";
      record.torso_success = row.at(comparison_header.at("lift_yaw_pitch_success")) == "1";
      record.classification = row.at(comparison_header.at("classification"));
      coarse_comparisons_.push_back(record);
      if (record.classification == "LIFT_ONLY_ONLY") coarse_anomaly_ = record;
    }
    if (coarse_records_.size() != 686 || coarse_comparisons_.size() != 343)
      throw std::runtime_error("Unexpected coarse evidence row count");
    if (coarse_anomaly_.classification.empty())
      throw std::runtime_error("Coarse LIFT_ONLY_ONLY anomaly was not found");
  }

  bool inside(const Eigen::Vector3d& point) const
  {
    return (point.array() >= target_bounds_.minimum.array()).all() &&
           (point.array() <= target_bounds_.maximum.array()).all();
  }

  void validateTargetedRegion()
  {
    int forward_expanded_inside = 0;
    double coarse_lift_max_z = -kInf;
    double coarse_torso_max_x = -kInf;
    for (const auto& row : coarse_comparisons_)
    {
      if (row.classification == "TORSO_EXPANDED" && row.xyz.x() >= 0.4 && inside(row.xyz))
        ++forward_expanded_inside;
      if (row.lift_success) coarse_lift_max_z = std::max(coarse_lift_max_z, row.xyz.z());
      if (row.torso_success) coarse_torso_max_x = std::max(coarse_torso_max_x, row.xyz.x());
    }
    if (forward_expanded_inside < 6)
      throw std::runtime_error("Targeted region does not contain the six forward coarse torso-expanded points");
    if (!inside(coarse_anomaly_.xyz))
      throw std::runtime_error("Targeted region does not contain the coarse LIFT_ONLY_ONLY anomaly");
    if (target_bounds_.maximum.z() < coarse_lift_max_z || target_bounds_.maximum.x() < coarse_torso_max_x)
      throw std::runtime_error("Targeted region truncates a required coarse forward/high-Z boundary");
  }

  void buildFineGrid()
  {
    voxel_ = Eigen::Vector3d((target_bounds_.maximum.x() - target_bounds_.minimum.x()) / grid_x_,
                             (target_bounds_.maximum.y() - target_bounds_.minimum.y()) / grid_y_,
                             (target_bounds_.maximum.z() - target_bounds_.minimum.z()) / grid_z_);
    points_.clear();
    std::size_t id = 0;
    for (int i = 0; i < grid_x_; ++i)
      for (int j = 0; j < grid_y_; ++j)
        for (int k = 0; k < grid_z_; ++k)
        {
          Point point;
          point.id = id++;
          point.i = i; point.j = j; point.k = k;
          point.xyz = target_bounds_.minimum + Eigen::Vector3d((i + 0.5) * voxel_.x(),
            (j + 0.5) * voxel_.y(), (k + 0.5) * voxel_.z());
          points_.push_back(point);
        }
  }

  double inward(const std::string& name, double value) const
  {
    const auto& bound = model_->getVariableBounds(name);
    const double inset = std::max(1e-6, exact_bound_epsilon_ * 10.0);
    return std::clamp(value, bound.min_position_ + inset, bound.max_position_ - inset);
  }

  void buildCandidateSets()
  {
    lift_values_fine_ = { 0.25, 0.35, 0.30, 0.20, 0.40, 0.05, 0.15, 0.45, 0.55, 0.65 };
    for (double& value : lift_values_fine_) value = inward("lift_joint", value);
    lift_values_fine_.erase(std::unique(lift_values_fine_.begin(), lift_values_fine_.end()), lift_values_fine_.end());
    for (const double lift : lift_values_fine_) candidates_a_.push_back({ lift, 0.0, 0.0 });

    constexpr double degree = M_PI / 180.0;
    const std::vector<double> yaw_raw{ 0, -5 * degree, 5 * degree, -10 * degree, 10 * degree,
                                      -2.5 * degree, 2.5 * degree, -7.5 * degree, 7.5 * degree };
    const std::vector<double> pitch_raw{ 0, -2.5 * degree, 2.5 * degree, -5 * degree, 5 * degree,
                                        -7.5 * degree, 7.5 * degree, -10 * degree, 10 * degree };
    std::vector<double> yaw, pitch;
    for (const double value : yaw_raw) yaw.push_back(inward("waist_yaw_joint", value));
    for (const double value : pitch_raw) pitch.push_back(inward("waist_pitch_joint", value));
    auto unique = [](std::vector<double>& values) {
      std::sort(values.begin(), values.end());
      values.erase(std::unique(values.begin(), values.end(), [](double a, double b) {
        return std::abs(a - b) < 1e-12;
      }), values.end());
    };
    unique(yaw); unique(pitch);
    for (const double lift : lift_values_fine_)
    {
      for (const double y : yaw) candidates_c1_.push_back({ lift, y, 0.0 });
      for (const double p : pitch) candidates_c2_.push_back({ lift, 0.0, p });
    }
    const auto single_displacement_sort = [](const TorsoCandidate& a, const TorsoCandidate& b) {
      return std::abs(a.yaw) + std::abs(a.pitch) < std::abs(b.yaw) + std::abs(b.pitch);
    };
    std::stable_sort(candidates_c1_.begin(), candidates_c1_.end(), single_displacement_sort);
    std::stable_sort(candidates_c2_.begin(), candidates_c2_.end(), single_displacement_sort);
    if (candidates_c1_.size() > static_cast<std::size_t>(max_torso_candidates_))
      candidates_c1_.resize(static_cast<std::size_t>(max_torso_candidates_));
    if (candidates_c2_.size() > static_cast<std::size_t>(max_torso_candidates_))
      candidates_c2_.resize(static_cast<std::size_t>(max_torso_candidates_));
    for (std::size_t lift_index = 0; lift_index < lift_values_fine_.size(); ++lift_index)
      for (const double y : yaw)
        for (const double p : pitch)
          candidates_b_.push_back({ lift_values_fine_[lift_index], y, p });
    const auto& yaw_bound = model_->getVariableBounds("waist_yaw_joint");
    const auto& pitch_bound = model_->getVariableBounds("waist_pitch_joint");
    std::stable_sort(candidates_b_.begin(), candidates_b_.end(), [&](const auto& a, const auto& b) {
      const double da = std::abs(a.yaw) / (yaw_bound.max_position_ - yaw_bound.min_position_) +
                        std::abs(a.pitch) / (pitch_bound.max_position_ - pitch_bound.min_position_);
      const double db = std::abs(b.yaw) / (yaw_bound.max_position_ - yaw_bound.min_position_) +
                        std::abs(b.pitch) / (pitch_bound.max_position_ - pitch_bound.min_position_);
      return da < db;
    });
    if (candidates_b_.size() > static_cast<std::size_t>(max_torso_candidates_))
      candidates_b_.resize(static_cast<std::size_t>(max_torso_candidates_));
  }

  void preflightFine()
  {
    if (points_.size() > static_cast<std::size_t>(max_grid_points_))
      throw std::runtime_error("Fine physical point count exceeds hard cap");
    if (max_ik_seeds_ > max_ik_seeds_hard_ || special_ik_seeds_ > max_special_ik_seeds_hard_)
      throw std::runtime_error("Fine IK seed setting exceeds hard cap");
    const std::size_t main_evaluations = points_.size() * 2;
    const std::size_t anomaly_evaluations = 27 * 2;
    const std::size_t high_z_points = std::count_if(points_.begin(), points_.end(), [&](const Point& point) {
      return point.xyz.z() >= high_z_min_;
    });
    if (main_evaluations + anomaly_evaluations + high_z_points >
        static_cast<std::size_t>(max_configuration_evaluations_))
      throw std::runtime_error("Fine configuration evaluations exceed hard cap");
    const std::size_t declared_attempts = main_evaluations * static_cast<std::size_t>(max_ik_seeds_) +
      (anomaly_evaluations + high_z_points) * static_cast<std::size_t>(special_ik_seeds_);
    if (declared_attempts > static_cast<std::size_t>(max_total_ik_attempts_))
      throw std::runtime_error("Fine declared IK attempts exceed hard cap");
    RCLCPP_INFO(node_->get_logger(),
      "FIXED_BASE_WORKSPACE_FINE PREFLIGHT grid=%dx%dx%d physical_points=%zu main_evaluations=%zu "
      "main_max_ik_attempts=%zu anomaly_evaluations=%zu declared_max_attempts=%zu torso_candidates=%zu "
      "bbox=[%.3f %.3f %.3f]-[%.3f %.3f %.3f]",
      grid_x_, grid_y_, grid_z_, points_.size(), main_evaluations,
      main_evaluations * static_cast<std::size_t>(max_ik_seeds_), anomaly_evaluations, declared_attempts,
      candidates_b_.size(), target_bounds_.minimum.x(), target_bounds_.minimum.y(), target_bounds_.minimum.z(),
      target_bounds_.maximum.x(), target_bounds_.maximum.y(), target_bounds_.maximum.z());
  }

  void initializeFineCollisionCsv()
  {
    fine_collision_csv_.open(output_dir_ + "/fixed_base_workspace_fine_collisions.csv", std::ios::trunc);
    if (!fine_collision_csv_) throw std::runtime_error("Cannot create fine collision CSV");
    fine_collision_csv_ << "timestamp,point_id,configuration,seed_index,tcp_x,tcp_y,tcp_z,lift,yaw,pitch,";
    for (const auto& name : arm_group_->getVariableNames()) fine_collision_csv_ << name << ',';
    fine_collision_csv_ << "collision_pairs,special_validation\n";
  }

  void randomArmSeed(moveit::core::RobotState& state, std::size_t point_id, int attempt) const
  {
    std::mt19937_64 engine(static_cast<std::uint64_t>(random_seed_) + point_id * 1000003ULL +
                           static_cast<std::uint64_t>(attempt) * 9176ULL);
    for (const auto& name : arm_group_->getVariableNames())
    {
      const auto& bound = model_->getVariableBounds(name);
      const double inset = std::max(exact_bound_epsilon_ * 10.0,
                                    (bound.max_position_ - bound.min_position_) * 1e-6);
      std::uniform_real_distribution<double> distribution(bound.min_position_ + inset, bound.max_position_ - inset);
      state.setVariablePosition(name, distribution(engine));
    }
  }

  const CoarseRecord* nearestCoarseSuccess(const Point& point, Configuration config) const
  {
    const CoarseRecord* best = nullptr;
    double distance = kInf;
    for (const auto& row : coarse_records_)
    {
      if (row.configuration != config || !row.success) continue;
      const double candidate = (row.xyz - point.xyz).squaredNorm();
      if (candidate < distance) { distance = candidate; best = &row; }
    }
    return best;
  }

  std::vector<TorsoCandidate> pointCandidates(const Point& point, Configuration config) const
  {
    std::vector<TorsoCandidate> result;
    if (const auto* coarse = nearestCoarseSuccess(point, config))
      result.push_back({ coarse->lift, config == Configuration::LIFT_ONLY ? 0.0 : coarse->yaw,
                        config == Configuration::LIFT_ONLY ? 0.0 : coarse->pitch });
    const std::vector<TorsoCandidate>* source = &candidates_b_;
    if (config == Configuration::LIFT_ONLY) source = &candidates_a_;
    else if (config == Configuration::LIFT_YAW) source = &candidates_c1_;
    else if (config == Configuration::LIFT_PITCH) source = &candidates_c2_;
    for (const auto& candidate : *source)
    {
      const bool duplicate = std::any_of(result.begin(), result.end(), [&](const auto& existing) {
        return std::abs(existing.lift - candidate.lift) < 1e-12 &&
               std::abs(existing.yaw - candidate.yaw) < 1e-12 &&
               std::abs(existing.pitch - candidate.pitch) < 1e-12;
      });
      if (!duplicate) result.push_back(candidate);
    }
    if (result.size() > static_cast<std::size_t>(max_torso_candidates_))
      result.resize(static_cast<std::size_t>(max_torso_candidates_));
    return result;
  }

  void appendFineCollision(const Point& point, Configuration config, int seed, const TorsoCandidate& torso,
                           const moveit::core::RobotState& state, const std::string& pairs, bool special)
  {
    fine_collision_csv_ << timestamp_ << ',' << point.id << ',' << configName(config) << ',' << seed << ','
      << number(point.xyz.x()) << ',' << number(point.xyz.y()) << ',' << number(point.xyz.z()) << ','
      << number(torso.lift) << ',' << number(torso.yaw) << ',' << number(torso.pitch) << ',';
    for (const auto& name : arm_group_->getVariableNames())
      fine_collision_csv_ << number(state.getVariablePosition(name)) << ',';
    fine_collision_csv_ << csvEscape(pairs) << ',' << (special ? 1 : 0) << '\n';
  }

  FineResult evaluateFinePoint(const Point& point, Configuration config, int budget, int required_valid,
                               const std::vector<const moveit::core::RobotState*>& neighbor_states,
                               bool special)
  {
    const auto begin = Clock::now();
    FineResult best(model_);
    best.metrics.point = point;
    best.metrics.configuration = config;
    best.special_validation = special;
    std::map<std::string, int> failures;
    std::set<std::string> all_pairs;
    const auto names = activeNames(config);
    const auto candidates = pointCandidates(point, config);
    bool any_neighbor_success = false;

    for (int attempt = 0; attempt < budget; ++attempt)
    {
      enforceWallTime();
      moveit::core::RobotState state(model_);
      TorsoCandidate torso;
      bool neighbor_attempt = static_cast<std::size_t>(attempt) < neighbor_states.size();
      std::string strategy;
      if (neighbor_attempt)
      {
        state = *neighbor_states[static_cast<std::size_t>(attempt)];
        torso = { state.getVariablePosition("lift_joint"), state.getVariablePosition("waist_yaw_joint"),
                   state.getVariablePosition("waist_pitch_joint") };
        strategy = "NEIGHBOR_STATE";
        best.neighbor_seed_used = true;
      }
      else
      {
        state = nominalState();
        const int general = attempt - static_cast<int>(neighbor_states.size());
        torso = candidates[static_cast<std::size_t>(general) % candidates.size()];
        setTorso(state, torso);
        const int strategy_index = general % 3;
        if (strategy_index == 0)
          strategy = "DEFAULT_PLUS_COARSE_TORSO";
        else if (strategy_index == 1)
        {
          seedArm(state, static_cast<std::size_t>(general), point.id);
          strategy = "HALTON_BOUNDED";
        }
        else
        {
          randomArmSeed(state, point.id, general);
          strategy = "FIXED_RANDOM_BOUNDED";
        }
      }
      if (config == Configuration::LIFT_ONLY) { torso.yaw = 0.0; torso.pitch = 0.0; }
      else if (config == Configuration::LIFT_YAW) torso.pitch = 0.0;
      else if (config == Configuration::LIFT_PITCH) torso.yaw = 0.0;
      setTorso(state, torso);
      state.update();
      const auto target = targetPoseInModel(point, state);
      ++best.metrics.seeds_tested;
      if (!state.setFromIK(arm_group_, target, tcp_frame_, ik_timeout_s_))
      {
        ++failures["NO_IK"];
        continue;
      }
      setTorso(state, torso);
      state.update();
      if (!state.satisfiesBounds()) { ++failures["JOINT_LIMIT_VIOLATION"]; continue; }
      const double margin = jointMargin(state, names);
      if (!(margin > exact_bound_epsilon_)) { ++failures["ACTIVE_JOINT_AT_BOUND"]; continue; }
      const double angle = orientationError(state);
      if (!(angle <= orientation_tolerance_)) { ++failures["ORIENTATION_ERROR"]; continue; }

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
        if (!pairs.empty()) all_pairs.insert(pairs);
        appendFineCollision(point, config, attempt, torso, state, pairs, special);
        continue;
      }

      Result valid;
      valid.point = point;
      valid.configuration = config;
      valid.success = true;
      valid.failure_reason = "REACHABLE";
      valid.selected_seed = attempt;
      valid.lift = torso.lift; valid.yaw = torso.yaw; valid.pitch = torso.pitch;
      valid.joint_margin = margin;
      valid.active_revolute_margin = activeRevoluteMargin(state, names);
      valid.self_clearance = scene_->getCollisionEnv()->distanceSelf(state, scene_->getAllowedCollisionMatrix());
      const JacobianMetrics jacobian = jacobianMetrics(state, config);
      valid.manipulability = jacobian.manipulability;
      valid.min_singular_value = jacobian.minimum_singular_value;
      valid.condition_number = jacobian.condition_number;
      valid.orientation_error = angle;
      const auto& yaw_bound = model_->getVariableBounds("waist_yaw_joint");
      const auto& pitch_bound = model_->getVariableBounds("waist_pitch_joint");
      valid.torso_displacement = std::abs(torso.yaw) / (yaw_bound.max_position_ - yaw_bound.min_position_) +
        std::abs(torso.pitch) / (pitch_bound.max_position_ - pitch_bound.min_position_);
      ++best.metrics.valid_count;
      if (neighbor_attempt) any_neighbor_success = true;
      if (better(valid, best.metrics))
      {
        const int tested = best.metrics.seeds_tested;
        const int valid_count = best.metrics.valid_count;
        best.metrics = valid;
        best.metrics.seeds_tested = tested;
        best.metrics.valid_count = valid_count;
        best.state = state;
        best.seed_strategy = strategy;
      }
      if (best.metrics.valid_count >= required_valid) break;
    }
    best.neighbor_seed_success = any_neighbor_success;
    if (!best.metrics.success)
    {
      const std::vector<std::string> priority{ "SELF_COLLISION", "ACTIVE_JOINT_AT_BOUND",
        "JOINT_LIMIT_VIOLATION", "ORIENTATION_ERROR", "NO_IK", "TIMEOUT", "INTERNAL_ERROR" };
      int largest = -1;
      for (const auto& reason : priority)
        if (failures[reason] > largest) { largest = failures[reason]; best.metrics.failure_reason = reason; }
      std::ostringstream pairs;
      bool first = true;
      for (const auto& item : all_pairs) { if (!first) pairs << ';'; pairs << item; first = false; }
      best.metrics.collision_pairs = pairs.str();
      best.seed_strategy = "EXHAUSTED_MIXED_SEARCH";
    }
    best.metrics.runtime_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    return best;
  }

  std::vector<const moveit::core::RobotState*> neighborStates(const Point& point,
                                                               const std::vector<FineResult>& completed) const
  {
    std::vector<const moveit::core::RobotState*> states;
    const auto add = [&](int i, int j, int k) {
      if (i < 0 || j < 0 || k < 0) return;
      const std::size_t id = static_cast<std::size_t>((i * grid_y_ + j) * grid_z_ + k);
      if (id < completed.size() && completed[id].metrics.success) states.push_back(&completed[id].state);
    };
    add(point.i, point.j, point.k - 1);
    add(point.i, point.j - 1, point.k);
    add(point.i - 1, point.j, point.k);
    return states;
  }

  std::vector<FineResult> evaluateGrid(Configuration config)
  {
    std::vector<FineResult> results;
    results.reserve(points_.size());
    for (std::size_t index = 0; index < points_.size(); ++index)
    {
      const auto neighbors = neighborStates(points_[index], results);
      results.push_back(evaluateFinePoint(points_[index], config, max_ik_seeds_, required_valid_solutions_,
                                          neighbors, false));
      if (index % static_cast<std::size_t>(std::max(1, progress_every_)) == 0 || index + 1 == points_.size())
      {
        const auto& result = results.back().metrics;
        RCLCPP_INFO(node_->get_logger(),
          "FIXED_BASE_WORKSPACE_FINE config=%s point=%zu/%zu xyz=(%.4f,%.4f,%.4f) result=%s "
          "reason=%s seeds=%d valid=%d neighbor=%d",
          configName(config).c_str(), index + 1, points_.size(), result.point.xyz.x(), result.point.xyz.y(),
          result.point.xyz.z(), result.success ? "REACHABLE" : "FAIL", result.failure_reason.c_str(),
          result.seeds_tested, result.valid_count, results.back().neighbor_seed_success ? 1 : 0);
      }
    }
    return results;
  }

  void revalidateHighZDiscrepancies()
  {
    for (std::size_t id = 0; id < points_.size(); ++id)
    {
      if (points_[id].xyz.z() < high_z_min_) continue;
      HighZRecord record;
      record.point = points_[id];
      record.primary_a = results_a_[id].metrics.success;
      record.primary_b = results_b_[id].metrics.success;
      record.primary_b_reason = results_b_[id].metrics.failure_reason;
      if (record.primary_a && !record.primary_b)
      {
        FineResult repeated = evaluateFinePoint(points_[id], Configuration::LIFT_YAW_PITCH,
          special_ik_seeds_, special_required_valid_solutions_, {}, true);
        record.special_b_run = true;
        record.special_seeds = repeated.metrics.seeds_tested;
        results_b_[id] = repeated;
      }
      record.final_b = results_b_[id].metrics.success;
      record.final_b_reason = results_b_[id].metrics.failure_reason;
      if (record.primary_a || record.primary_b || record.special_b_run) high_z_records_.push_back(record);
    }
  }

  void validateCoarseAnomalyNeighborhood()
  {
    std::size_t special_id = 100000;
    for (int di = -1; di <= 1; ++di)
      for (int dj = -1; dj <= 1; ++dj)
        for (int dk = -1; dk <= 1; ++dk)
        {
          Point point;
          point.id = special_id++;
          point.i = di; point.j = dj; point.k = dk;
          point.xyz = coarse_anomaly_.xyz + Eigen::Vector3d(di * voxel_.x(), dj * voxel_.y(), dk * voxel_.z());
          point.xyz = point.xyz.cwiseMax(target_bounds_.minimum).cwiseMin(target_bounds_.maximum);
          FineResult a = evaluateFinePoint(point, Configuration::LIFT_ONLY, special_ik_seeds_,
                                           special_required_valid_solutions_, {}, true);
          FineResult b = evaluateFinePoint(point, Configuration::LIFT_YAW_PITCH, special_ik_seeds_,
                                           special_required_valid_solutions_, {}, true);
          special_points_.push_back(point);
          special_a_.push_back(a);
          special_b_.push_back(b);
          if (di == 0 && dj == 0 && dk == 0)
          {
            if (a.metrics.success && b.metrics.success) anomaly_verdict_ = "SAMPLING_ARTIFACT";
            else if (a.metrics.success && !b.metrics.success) anomaly_verdict_ = "CONFIRMED_LIFT_ONLY_ONLY";
            else anomaly_verdict_ = "UNRESOLVED";
          }
    }
  }

  std::string classify(const FineResult& a, const FineResult& b) const
  {
    if (a.metrics.success && b.metrics.success) return "COMMON_REACHABLE";
    if (!a.metrics.success && b.metrics.success) return "TORSO_EXPANDED";
    if (a.metrics.success && !b.metrics.success) return "LIFT_ONLY_ONLY";
    return "UNREACHABLE_BOTH";
  }

  int sameClassNeighbors(std::size_t id, const std::string& classification) const
  {
    const Point& point = points_[id];
    int count = 0;
    for (const auto& delta : std::vector<std::array<int, 3>>{
      { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 } })
    {
      const int i = point.i + delta[0], j = point.j + delta[1], k = point.k + delta[2];
      if (i < 0 || i >= grid_x_ || j < 0 || j >= grid_y_ || k < 0 || k >= grid_z_) continue;
      const std::size_t neighbor = static_cast<std::size_t>((i * grid_y_ + j) * grid_z_ + k);
      if (classify(results_a_[neighbor], results_b_[neighbor]) == classification) ++count;
    }
    return count;
  }

  std::string confidence(std::size_t id) const
  {
    const auto& a = results_a_[id].metrics;
    const auto& b = results_b_[id].metrics;
    const std::string label = classify(results_a_[id], results_b_[id]);
    const int neighbors = sameClassNeighbors(id, label);
    bool repeated = false;
    if (label == "COMMON_REACHABLE") repeated = a.valid_count >= 2 && b.valid_count >= 2;
    else if (label == "TORSO_EXPANDED") repeated = b.valid_count >= 2 && a.seeds_tested >= max_ik_seeds_;
    else if (label == "LIFT_ONLY_ONLY") repeated = a.valid_count >= 2 && b.seeds_tested >= max_ik_seeds_;
    else repeated = a.seeds_tested >= max_ik_seeds_ && b.seeds_tested >= max_ik_seeds_;
    if (repeated && neighbors >= 2) return "HIGH_CONFIDENCE";
    if (repeated || a.success || b.success) return "MEDIUM_CONFIDENCE";
    return "LOW_CONFIDENCE";
  }

  void writeFinePoints() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_points.csv", std::ios::trunc);
    out << "timestamp,point_id,grid_i,grid_j,grid_k,configuration,tcp_x,tcp_y,tcp_z,success,failure_reason,"
      "ik_seeds_tested,valid_ik_count,neighbor_seed_used,neighbor_seed_success,selected_lift,selected_yaw,"
      "selected_pitch,min_joint_limit_margin,min_active_revolute_margin,self_collision_clearance,"
      "manipulability,min_jacobian_singular_value,jacobian_condition_number,orientation_error,collision_pairs,"
      "runtime_ms,seed_strategy,confidence\n";
    for (std::size_t id = 0; id < points_.size(); ++id)
      for (const FineResult* fine : { &results_a_[id], &results_b_[id] })
      {
        const Result& r = fine->metrics;
        out << timestamp_ << ',' << r.point.id << ',' << r.point.i << ',' << r.point.j << ',' << r.point.k << ','
          << configName(r.configuration) << ',' << number(r.point.xyz.x()) << ',' << number(r.point.xyz.y()) << ','
          << number(r.point.xyz.z()) << ',' << (r.success ? 1 : 0) << ',' << r.failure_reason << ','
          << r.seeds_tested << ',' << r.valid_count << ',' << (fine->neighbor_seed_used ? 1 : 0) << ','
          << (fine->neighbor_seed_success ? 1 : 0) << ',' << number(r.lift) << ',' << number(r.yaw) << ','
          << number(r.pitch) << ',' << number(r.joint_margin) << ',' << number(r.active_revolute_margin) << ','
          << number(r.self_clearance) << ',' << number(r.manipulability) << ',' << number(r.min_singular_value) << ','
          << number(r.condition_number) << ',' << number(r.orientation_error) << ',' << csvEscape(r.collision_pairs)
          << ',' << number(r.runtime_ms) << ',' << fine->seed_strategy << ',' << confidence(id) << '\n';
      }
  }

  void writeComparison() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_comparison.csv", std::ios::trunc);
    out << "point_id,tcp_x,tcp_y,tcp_z,lift_only_success,lift_yaw_pitch_success,classification,confidence,"
      "lift_only_failure_reason,lift_yaw_pitch_failure_reason,lift_only_joint_margin,lift_yaw_pitch_joint_margin,"
      "lift_only_self_clearance,lift_yaw_pitch_self_clearance,lift_only_manipulability,"
      "lift_yaw_pitch_manipulability\n";
    for (std::size_t id = 0; id < points_.size(); ++id)
    {
      const auto& a = results_a_[id].metrics; const auto& b = results_b_[id].metrics;
      out << id << ',' << number(points_[id].xyz.x()) << ',' << number(points_[id].xyz.y()) << ','
        << number(points_[id].xyz.z()) << ',' << (a.success ? 1 : 0) << ',' << (b.success ? 1 : 0) << ','
        << classify(results_a_[id], results_b_[id]) << ',' << confidence(id) << ',' << a.failure_reason << ','
        << b.failure_reason << ',' << number(a.joint_margin) << ',' << number(b.joint_margin) << ','
        << number(a.self_clearance) << ',' << number(b.self_clearance) << ',' << number(a.manipulability) << ','
        << number(b.manipulability) << '\n';
    }
  }

  void writeBoundary() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_boundary.csv", std::ios::trunc);
    out << "grid_j,grid_k,y,z,lift_only_status,lift_only_max_x,lift_yaw_pitch_status,"
      "lift_yaw_pitch_max_x,delta_x\n";
    for (int j = 0; j < grid_y_; ++j)
      for (int k = 0; k < grid_z_; ++k)
      {
        double max_a = -kInf, max_b = -kInf;
        for (int i = 0; i < grid_x_; ++i)
        {
          const std::size_t id = static_cast<std::size_t>((i * grid_y_ + j) * grid_z_ + k);
          if (results_a_[id].metrics.success) max_a = std::max(max_a, points_[id].xyz.x());
          if (results_b_[id].metrics.success) max_b = std::max(max_b, points_[id].xyz.x());
        }
        const std::size_t reference = static_cast<std::size_t>(j * grid_z_ + k);
        out << j << ',' << k << ',' << number(points_[reference].xyz.y()) << ','
          << number(points_[reference].xyz.z()) << ',' << (std::isfinite(max_a) ? "REACHABLE" : "NO_REACHABLE_POINT")
          << ',' << number(max_a) << ',' << (std::isfinite(max_b) ? "REACHABLE" : "NO_REACHABLE_POINT") << ','
          << number(max_b) << ',' << number(std::isfinite(max_a) && std::isfinite(max_b) ? max_b - max_a : kNaN) << '\n';
      }
  }

  void writeExpanded() const
  {
    std::vector<Eigen::Vector3d> common;
    for (std::size_t id = 0; id < points_.size(); ++id)
      if (classify(results_a_[id], results_b_[id]) == "COMMON_REACHABLE") common.push_back(points_[id].xyz);
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_torso_expanded.csv", std::ios::trunc);
    out << "point_id,x,y,z,selected_lift,selected_yaw,selected_pitch,joint_margin,self_clearance,manipulability,"
      "distance_to_nearest_common_point,confidence\n";
    for (std::size_t id = 0; id < points_.size(); ++id)
    {
      if (classify(results_a_[id], results_b_[id]) != "TORSO_EXPANDED") continue;
      double distance = kInf;
      for (const auto& point : common) distance = std::min(distance, (point - points_[id].xyz).norm());
      const auto& b = results_b_[id].metrics;
      out << id << ',' << number(points_[id].xyz.x()) << ',' << number(points_[id].xyz.y()) << ','
        << number(points_[id].xyz.z()) << ',' << number(b.lift) << ',' << number(b.yaw) << ',' << number(b.pitch)
        << ',' << number(b.joint_margin) << ',' << number(b.self_clearance) << ',' << number(b.manipulability)
        << ',' << number(distance) << ',' << confidence(id) << '\n';
    }
  }

  void writeSpecialValidation() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_anomaly_validation.csv", std::ios::trunc);
    out << "special_point_id,offset_i,offset_j,offset_k,x,y,z,is_exact_coarse_anomaly,lift_only_success,"
      "lift_yaw_pitch_success,classification,lift_only_seeds,lift_yaw_pitch_seeds,lift_only_failure_reason,"
      "lift_yaw_pitch_failure_reason,exact_anomaly_verdict\n";
    for (std::size_t i = 0; i < special_points_.size(); ++i)
    {
      const auto& point = special_points_[i];
      const bool exact = point.i == 0 && point.j == 0 && point.k == 0;
      out << point.id << ',' << point.i << ',' << point.j << ',' << point.k << ',' << number(point.xyz.x()) << ','
        << number(point.xyz.y()) << ',' << number(point.xyz.z()) << ',' << (exact ? 1 : 0) << ','
        << (special_a_[i].metrics.success ? 1 : 0) << ',' << (special_b_[i].metrics.success ? 1 : 0) << ','
        << classify(special_a_[i], special_b_[i]) << ',' << special_a_[i].metrics.seeds_tested << ','
        << special_b_[i].metrics.seeds_tested << ',' << special_a_[i].metrics.failure_reason << ','
        << special_b_[i].metrics.failure_reason << ',' << (exact ? anomaly_verdict_ : "NEIGHBORHOOD") << '\n';
    }
  }

  void writeHighZ() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_high_z_validation.csv", std::ios::trunc);
    out << "point_id,x,y,z,lift_only_primary_success,torso_primary_success,torso_special_validation_run,"
      "torso_final_success,torso_primary_failure_reason,torso_final_failure_reason,special_ik_seeds_tested\n";
    for (const auto& row : high_z_records_)
      out << row.point.id << ',' << number(row.point.xyz.x()) << ',' << number(row.point.xyz.y()) << ','
        << number(row.point.xyz.z()) << ',' << (row.primary_a ? 1 : 0) << ',' << (row.primary_b ? 1 : 0) << ','
        << (row.special_b_run ? 1 : 0) << ',' << (row.final_b ? 1 : 0) << ',' << row.primary_b_reason << ','
        << row.final_b_reason << ',' << row.special_seeds << '\n';
  }

  std::size_t nearestFine(const Eigen::Vector3d& xyz) const
  {
    std::size_t best = 0;
    double distance = kInf;
    for (std::size_t id = 0; id < points_.size(); ++id)
    {
      const double candidate = (points_[id].xyz - xyz).squaredNorm();
      if (candidate < distance) { distance = candidate; best = id; }
    }
    return best;
  }

  void writeCoarseConsistency() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_coarse_consistency.csv", std::ios::trunc);
    out << "coarse_point_id,configuration,coarse_x,coarse_y,coarse_z,fine_point_id,fine_x,fine_y,fine_z,"
      "nearest_distance,coarse_success,fine_success,consistency_classification,fine_failure_reason,"
      "coarse_failure_reason\n";
    for (const auto& coarse : coarse_records_)
    {
      if (!inside(coarse.xyz)) continue;
      const std::size_t id = nearestFine(coarse.xyz);
      const auto& fine = coarse.configuration == Configuration::LIFT_ONLY ? results_a_[id] : results_b_[id];
      std::string label;
      if (coarse.success && fine.metrics.success) label = "COARSE_PASS_FINE_PASS";
      else if (!coarse.success && fine.metrics.success) label = "COARSE_FAIL_FINE_PASS";
      else if (coarse.success && !fine.metrics.success) label = "COARSE_PASS_FINE_FAIL";
      else label = "COARSE_FAIL_FINE_FAIL";
      out << coarse.point_id << ',' << configName(coarse.configuration) << ',' << number(coarse.xyz.x()) << ','
        << number(coarse.xyz.y()) << ',' << number(coarse.xyz.z()) << ',' << id << ',' << number(points_[id].xyz.x())
        << ',' << number(points_[id].xyz.y()) << ',' << number(points_[id].xyz.z()) << ','
        << number((points_[id].xyz - coarse.xyz).norm()) << ',' << (coarse.success ? 1 : 0) << ','
        << (fine.metrics.success ? 1 : 0) << ',' << label << ',' << fine.metrics.failure_reason << ','
        << coarse.failure << '\n';
    }
  }

  void writeSummary() const
  {
    std::size_t count_a = 0, count_b = 0, common = 0, expanded = 0, only = 0, neither = 0;
    double max_a = -kInf, max_b = -kInf;
    std::vector<double> margin_a, margin_b, clearance_a, clearance_b, manip_a, manip_b;
    for (std::size_t id = 0; id < points_.size(); ++id)
    {
      const auto& a = results_a_[id].metrics; const auto& b = results_b_[id].metrics;
      if (a.success) { ++count_a; max_a = std::max(max_a, points_[id].xyz.x()); margin_a.push_back(a.joint_margin);
        clearance_a.push_back(a.self_clearance); manip_a.push_back(a.manipulability); }
      if (b.success) { ++count_b; max_b = std::max(max_b, points_[id].xyz.x()); margin_b.push_back(b.joint_margin);
        clearance_b.push_back(b.self_clearance); manip_b.push_back(b.manipulability); }
      const std::string label = classify(results_a_[id], results_b_[id]);
      if (label == "COMMON_REACHABLE") ++common;
      else if (label == "TORSO_EXPANDED") ++expanded;
      else if (label == "LIFT_ONLY_ONLY") ++only;
      else ++neither;
    }
    const double volume = voxel_.prod();
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_summary.csv", std::ios::trunc);
    out << "configuration,total_points,reachable_points,reachable_rate,targeted_reachable_volume,max_forward_x,"
      "mean_joint_margin,min_joint_margin,mean_self_clearance,min_self_clearance,mean_manipulability,"
      "median_manipulability\n";
    out << "LIFT_ONLY," << points_.size() << ',' << count_a << ',' << number(static_cast<double>(count_a) / points_.size())
      << ',' << number(count_a * volume) << ',' << number(max_a) << ',' << number(mean(margin_a)) << ','
      << number(minimum(margin_a)) << ',' << number(mean(clearance_a)) << ',' << number(minimum(clearance_a)) << ','
      << number(mean(manip_a)) << ',' << number(median(manip_a)) << '\n';
    out << "LIFT_YAW_PITCH," << points_.size() << ',' << count_b << ',' << number(static_cast<double>(count_b) / points_.size())
      << ',' << number(count_b * volume) << ',' << number(max_b) << ',' << number(mean(margin_b)) << ','
      << number(minimum(margin_b)) << ',' << number(mean(clearance_b)) << ',' << number(minimum(clearance_b)) << ','
      << number(mean(manip_b)) << ',' << number(median(manip_b)) << '\n';
    std::ofstream expansion(output_dir_ + "/fixed_base_workspace_fine_expansion_summary.csv", std::ios::trunc);
    expansion << "common_reachable_count,torso_expanded_count,lift_only_only_count,unreachable_both_count,"
      "voxel_volume,targeted_lift_only_volume,targeted_torso_volume,targeted_delta_volume,torso_expanded_volume,"
      "lift_only_max_forward_x,torso_max_forward_x,delta_forward_x,coarse_anomaly_verdict\n";
    expansion << common << ',' << expanded << ',' << only << ',' << neither << ',' << number(volume) << ','
      << number(count_a * volume) << ',' << number(count_b * volume) << ','
      << number((static_cast<double>(count_b) - static_cast<double>(count_a)) * volume)
      << ',' << number(expanded * volume) << ',' << number(max_a) << ',' << number(max_b) << ','
      << number(max_b - max_a) << ',' << anomaly_verdict_ << '\n';
  }

  void writeMetadata() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_fine_metadata.csv", std::ios::trunc);
    out << "key,value\n"
      << "timestamp," << timestamp_ << '\n'
      << "model_frame," << model_->getModelFrame() << '\n'
      << "base_frame," << base_frame_ << '\n'
      << "tcp_frame," << tcp_frame_ << '\n'
      << "bbox_x_min," << number(target_bounds_.minimum.x()) << '\n'
      << "bbox_x_max," << number(target_bounds_.maximum.x()) << '\n'
      << "bbox_y_min," << number(target_bounds_.minimum.y()) << '\n'
      << "bbox_y_max," << number(target_bounds_.maximum.y()) << '\n'
      << "bbox_z_min," << number(target_bounds_.minimum.z()) << '\n'
      << "bbox_z_max," << number(target_bounds_.maximum.z()) << '\n'
      << "grid_x," << grid_x_ << '\n' << "grid_y," << grid_y_ << '\n' << "grid_z," << grid_z_ << '\n'
      << "voxel_dx," << number(voxel_.x()) << '\n' << "voxel_dy," << number(voxel_.y()) << '\n'
      << "voxel_dz," << number(voxel_.z()) << '\n' << "voxel_volume," << number(voxel_.prod()) << '\n'
      << "physical_points," << points_.size() << '\n' << "configuration_evaluations," << points_.size() * 2 << '\n'
      << "max_ik_seeds," << max_ik_seeds_ << '\n' << "special_max_ik_seeds," << special_ik_seeds_ << '\n'
      << "random_seed," << random_seed_ << '\n' << "torso_candidate_count," << candidates_b_.size() << '\n'
      << "orientation_qx," << number(target_q_.x()) << '\n' << "orientation_qy," << number(target_q_.y()) << '\n'
      << "orientation_qz," << number(target_q_.z()) << '\n' << "orientation_qw," << number(target_q_.w()) << '\n'
      << "orientation_tolerance_rad," << number(orientation_tolerance_) << '\n'
      << "seed_strategy,NEIGHBOR_THEN_COARSE_TORSO_THEN_DEFAULT_HALTON_FIXED_RANDOM\n"
      << "coarse_manifest_precheck,PASS\n"
      << "environment_objects,0\ntrajectory_execution,false\ncontrollers,false\nros2_control,false\nhardware,false\n";
  }

  void writeOutputs()
  {
    fine_collision_csv_.flush();
    writeFinePoints(); writeComparison(); writeBoundary(); writeExpanded(); writeSpecialValidation(); writeHighZ();
    writeCoarseConsistency(); writeSummary(); writeMetadata();
  }

  void runPostprocessor() const
  {
    const pid_t child = fork();
    if (child < 0) throw std::runtime_error("fork failed for fine postprocessor");
    if (child == 0)
    {
      execl(postprocess_executable_.c_str(), postprocess_executable_.c_str(), "--output-dir", output_dir_.c_str(),
            "--config", fine_config_path_.c_str(), "--coarse-manifest", coarse_manifest_.c_str(),
            static_cast<char*>(nullptr));
      _exit(127);
    }
    int status = 0;
    if (waitpid(child, &status, 0) < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
      throw std::runtime_error("Fine postprocessor failed");
  }

  Bounds3 target_bounds_;
  int special_ik_seeds_{}, max_special_ik_seeds_hard_{}, required_valid_solutions_{};
  int special_required_valid_solutions_{}, random_seed_{};
  double high_z_min_{};
  std::string coarse_points_csv_, coarse_comparison_csv_, coarse_manifest_, fine_config_path_;
  std::string postprocess_executable_, anomaly_verdict_{ "UNRESOLVED" };
  std::vector<CoarseRecord> coarse_records_;
  std::vector<CoarseComparison> coarse_comparisons_;
  CoarseComparison coarse_anomaly_;
  std::vector<double> lift_values_fine_;
  std::vector<TorsoCandidate> candidates_a_, candidates_b_, candidates_c1_, candidates_c2_;
  std::vector<FineResult> results_a_, results_b_, special_a_, special_b_;
  std::vector<Point> special_points_;
  std::vector<HighZRecord> high_z_records_;
  std::ofstream fine_collision_csv_;
};
}  // namespace fixed_base_workspace_fine

#ifndef FIXED_BASE_WORKSPACE_FINE_NO_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fixed_base_workspace_fine");
  try
  {
    fixed_base_workspace_fine::Runner runner(node);
    runner.runFine();
    RCLCPP_INFO(node->get_logger(), "FIXED_BASE_WORKSPACE_FINE completed with postprocess and manifest verification");
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "FIXED_BASE_WORKSPACE_FINE failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
