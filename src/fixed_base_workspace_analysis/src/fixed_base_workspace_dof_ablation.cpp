#include <memory>

#define FIXED_BASE_WORKSPACE_FINE_NO_MAIN
#include "fixed_base_workspace_fine.cpp"

namespace fixed_base_workspace_dof_ablation
{
using fixed_base_workspace::Configuration;
using fixed_base_workspace::Point;
using fixed_base_workspace::Result;
using fixed_base_workspace::Clock;
using fixed_base_workspace::configName;
using fixed_base_workspace::csvEscape;
using fixed_base_workspace::isoTimestamp;
using fixed_base_workspace::kNaN;
using fixed_base_workspace::number;
using fixed_base_workspace_fine::FineResult;
using fixed_base_workspace_fine::headerMap;
using fixed_base_workspace_fine::optionalDouble;
using fixed_base_workspace_fine::parseCsvLine;

struct ExistingResult
{
  bool present{};
  bool success{};
  std::string failure_reason;
  int seeds_tested{};
  int valid_count{};
  double lift{ kNaN }, yaw{ kNaN }, pitch{ kNaN };
  double joint_margin{ kNaN }, active_revolute_margin{ kNaN }, self_clearance{ kNaN };
  double manipulability{ kNaN }, min_singular{ kNaN }, condition{ kNaN }, orientation_error{ kNaN };
  std::string collision_pairs;
  double runtime_ms{};
};

struct ConsistencyRecord
{
  std::size_t point_id{};
  std::string pattern;
  std::string target_configuration;
  bool lower_success{};
  bool before_success{};
  bool after_success{};
  int seeds_tested{};
  bool lower_state_seed_used{};
  std::string before_reason;
  std::string after_reason;
  std::string verdict;
};

class Runner : public fixed_base_workspace_fine::Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node) : fixed_base_workspace_fine::Runner(node)
  {
    fine_points_csv_ = parameter<std::string>("fine_points_csv");
    fine_metadata_csv_ = parameter<std::string>("fine_metadata_csv");
    fine_manifest_path_ = parameter<std::string>("fine_manifest");
    ablation_config_path_ = parameter<std::string>("ablation_config_path");
    postprocess_ablation_ = parameter<std::string>("postprocess_executable");
    urdf_hash_ = parameter<std::string>("urdf_xacro_sha256");
    srdf_hash_ = parameter<std::string>("srdf_sha256");
    limits_hash_ = parameter<std::string>("joint_limits_sha256");
    kinematics_hash_ = parameter<std::string>("kinematics_sha256");
    max_new_evaluations_ = parameter<int>("max_new_configuration_evaluations");
    max_special_evaluations_ = parameter<int>("max_special_revalidation_evaluations");
  }

  void runAblation()
  {
    start_time_ = Clock::now();
    timestamp_ = isoTimestamp();
    loadExistingFineEvidence();
    validateReuseContract();
    buildCandidateSets();
    preflightAblation();
    initializeAblationCollisionCsv();

    RCLCPP_INFO(node_->get_logger(), "FIXED_BASE_WORKSPACE_DOF_ABLATION config=LIFT_YAW begin");
    results_c1_ = evaluateGrid(Configuration::LIFT_YAW);
    RCLCPP_INFO(node_->get_logger(), "FIXED_BASE_WORKSPACE_DOF_ABLATION config=LIFT_PITCH begin");
    results_c2_ = evaluateGrid(Configuration::LIFT_PITCH);
    verifyLockedJoints();
    revalidateNestedConsistency();
    writeNewPoints();
    writeConsistency();
    writeC3Overrides();
    writeAblationMetadata();
    fine_collision_csv_.close();
    runAblationPostprocessor();
  }

protected:
  ExistingResult parseExisting(const std::vector<std::string>& row,
                               const std::unordered_map<std::string, std::size_t>& header) const
  {
    ExistingResult result;
    result.present = true;
    result.success = row.at(header.at("success")) == "1";
    result.failure_reason = row.at(header.at("failure_reason"));
    result.seeds_tested = std::stoi(row.at(header.at("ik_seeds_tested")));
    result.valid_count = std::stoi(row.at(header.at("valid_ik_count")));
    result.lift = optionalDouble(row.at(header.at("selected_lift")));
    result.yaw = optionalDouble(row.at(header.at("selected_yaw")));
    result.pitch = optionalDouble(row.at(header.at("selected_pitch")));
    result.joint_margin = optionalDouble(row.at(header.at("min_joint_limit_margin")));
    result.active_revolute_margin = optionalDouble(row.at(header.at("min_active_revolute_margin")));
    result.self_clearance = optionalDouble(row.at(header.at("self_collision_clearance")));
    result.manipulability = optionalDouble(row.at(header.at("manipulability")));
    result.min_singular = optionalDouble(row.at(header.at("min_jacobian_singular_value")));
    result.condition = optionalDouble(row.at(header.at("jacobian_condition_number")));
    result.orientation_error = optionalDouble(row.at(header.at("orientation_error")));
    result.collision_pairs = row.at(header.at("collision_pairs"));
    result.runtime_ms = std::stod(row.at(header.at("runtime_ms")));
    return result;
  }

  void loadExistingFineEvidence()
  {
    std::ifstream metadata_file(fine_metadata_csv_);
    if (!metadata_file) throw std::runtime_error("Cannot read fine metadata for C0/C3 reuse");
    std::string line;
    std::getline(metadata_file, line);
    while (std::getline(metadata_file, line))
    {
      const auto row = parseCsvLine(line);
      if (row.size() >= 2) fine_metadata_[row[0]] = row[1];
    }

    const std::size_t expected = static_cast<std::size_t>(grid_x_) * grid_y_ * grid_z_;
    points_.assign(expected, Point{});
    c0_.assign(expected, ExistingResult{});
    c3_.assign(expected, ExistingResult{});
    std::vector<bool> point_seen(expected, false);
    std::ifstream input(fine_points_csv_);
    if (!input) throw std::runtime_error("Cannot read fine points for C0/C3 reuse");
    std::getline(input, line);
    const auto header = headerMap(parseCsvLine(line));
    while (std::getline(input, line))
    {
      const auto row = parseCsvLine(line);
      const std::size_t id = std::stoull(row.at(header.at("point_id")));
      if (id >= expected) throw std::runtime_error("Fine point_id outside expected 1440-point set");
      Point point;
      point.id = id;
      point.i = std::stoi(row.at(header.at("grid_i")));
      point.j = std::stoi(row.at(header.at("grid_j")));
      point.k = std::stoi(row.at(header.at("grid_k")));
      point.xyz = Eigen::Vector3d(std::stod(row.at(header.at("tcp_x"))),
                                  std::stod(row.at(header.at("tcp_y"))),
                                  std::stod(row.at(header.at("tcp_z"))));
      if (point_seen[id] && (points_[id].xyz - point.xyz).norm() > 1e-12)
        throw std::runtime_error("C0/C3 coordinate mismatch in fine points CSV");
      points_[id] = point;
      point_seen[id] = true;
      const std::string configuration = row.at(header.at("configuration"));
      if (configuration == "LIFT_ONLY") c0_[id] = parseExisting(row, header);
      else if (configuration == "LIFT_YAW_PITCH") c3_[id] = parseExisting(row, header);
      else throw std::runtime_error("Unexpected configuration in reusable fine points CSV");
    }
    for (std::size_t id = 0; id < expected; ++id)
      if (!point_seen[id] || !c0_[id].present || !c3_[id].present)
        throw std::runtime_error("Fine reuse evidence lacks exactly one C0 and C3 row per point");
  }

  void validateReuseContract()
  {
    const auto require = [&](const std::string& key, const std::string& expected) {
      if (!fine_metadata_.count(key) || fine_metadata_.at(key) != expected)
        throw std::runtime_error("Fine metadata reuse mismatch: " + key);
    };
    require("physical_points", "1440");
    require("grid_x", std::to_string(grid_x_));
    require("grid_y", std::to_string(grid_y_));
    require("grid_z", std::to_string(grid_z_));
    require("max_ik_seeds", std::to_string(max_ik_seeds_));
    require("random_seed", std::to_string(random_seed_));
    require("base_frame", base_frame_);
    require("tcp_frame", tcp_frame_);
    if (std::abs(std::stod(fine_metadata_.at("orientation_qx")) - target_q_.x()) > 1e-15 ||
        std::abs(std::stod(fine_metadata_.at("orientation_qy")) - target_q_.y()) > 1e-15 ||
        std::abs(std::stod(fine_metadata_.at("orientation_qz")) - target_q_.z()) > 1e-15 ||
        std::abs(std::stod(fine_metadata_.at("orientation_qw")) - target_q_.w()) > 1e-15 ||
        std::abs(std::stod(fine_metadata_.at("orientation_tolerance_rad")) - orientation_tolerance_) > 1e-15)
      throw std::runtime_error("Fine orientation contract mismatch");
    voxel_ = Eigen::Vector3d(std::stod(fine_metadata_.at("voxel_dx")),
                             std::stod(fine_metadata_.at("voxel_dy")),
                             std::stod(fine_metadata_.at("voxel_dz")));
  }

  void preflightAblation()
  {
    if (points_.size() != 1440 || points_.size() > static_cast<std::size_t>(max_grid_points_))
      throw std::runtime_error("Ablation must use exactly the existing 1440 physical points");
    const std::size_t evaluations = points_.size() * 2;
    if (evaluations != static_cast<std::size_t>(max_new_evaluations_))
      throw std::runtime_error("New C1/C2 evaluation count must equal hard cap 2880");
    if (max_ik_seeds_ != 150 || special_ik_seeds_ != 300)
      throw std::runtime_error("Ablation seed policy differs from required fine policy");
    RCLCPP_INFO(node_->get_logger(),
      "FIXED_BASE_WORKSPACE_DOF_ABLATION PREFLIGHT coarse_manifest=PASS fine_manifest=PASS "
      "robot_model_hash=%s srdf_hash=%s joint_limits_hash=%s kinematics_hash=%s points=%zu "
      "reuse_C0=YES reuse_C3=YES new_configurations=2 new_evaluations=%zu orientation_same=YES "
      "seed_policy_same=YES hardware=NO controller=NO ros2_control=NO",
      urdf_hash_.c_str(), srdf_hash_.c_str(), limits_hash_.c_str(), kinematics_hash_.c_str(),
      points_.size(), evaluations);
  }

  void initializeAblationCollisionCsv()
  {
    fine_collision_csv_.open(output_dir_ + "/fixed_base_workspace_dof_ablation_collisions.csv", std::ios::trunc);
    if (!fine_collision_csv_) throw std::runtime_error("Cannot create ablation collision CSV");
    fine_collision_csv_ << "timestamp,point_id,configuration,seed_index,tcp_x,tcp_y,tcp_z,lift,yaw,pitch,";
    for (const auto& name : arm_group_->getVariableNames()) fine_collision_csv_ << name << ',';
    fine_collision_csv_ << "collision_pairs,special_validation\n";
  }

  void verifyLockedJoints() const
  {
    for (std::size_t id = 0; id < points_.size(); ++id)
    {
      if (results_c1_[id].metrics.success && std::abs(results_c1_[id].metrics.pitch) > 1e-14)
        throw std::runtime_error("C1 pitch lock violation");
      if (results_c2_[id].metrics.success && std::abs(results_c2_[id].metrics.yaw) > 1e-14)
        throw std::runtime_error("C2 yaw lock violation");
    }
  }

  void addConsistency(std::size_t id, const std::string& pattern, Configuration target,
                      bool before, const std::string& before_reason, FineResult&& repeated,
                      bool lower_seed_used)
  {
    if (++special_evaluations_ > max_special_evaluations_)
      throw std::runtime_error("Nested consistency special-evaluation hard cap exceeded");
    ConsistencyRecord record;
    record.point_id = id;
    record.pattern = pattern;
    record.target_configuration = configName(target);
    record.lower_success = true;
    record.before_success = before;
    record.after_success = repeated.metrics.success;
    record.seeds_tested = repeated.metrics.seeds_tested;
    record.lower_state_seed_used = lower_seed_used;
    record.before_reason = before_reason;
    record.after_reason = repeated.metrics.failure_reason;
    record.verdict = repeated.metrics.success ? "SAMPLING_ARTIFACT" :
      (repeated.metrics.failure_reason == "INTERNAL_ERROR" || repeated.metrics.failure_reason == "TIMEOUT" ?
       "UNRESOLVED" : "UNRESOLVED");
    consistency_.push_back(record);
    if (target == Configuration::LIFT_YAW) results_c1_[id] = std::move(repeated);
    else if (target == Configuration::LIFT_PITCH) results_c2_[id] = std::move(repeated);
    else c3_override_[id] = std::make_unique<FineResult>(std::move(repeated));
  }

  void revalidateNestedConsistency()
  {
    c3_override_.resize(points_.size());
    for (std::size_t id = 0; id < points_.size(); ++id)
    {
      if (c0_[id].success && !results_c1_[id].metrics.success)
      {
        std::vector<const moveit::core::RobotState*> seeds;
        if (results_c2_[id].metrics.success) seeds.push_back(&results_c2_[id].state);
        FineResult repeated = evaluateFinePoint(points_[id], Configuration::LIFT_YAW, special_ik_seeds_,
          special_required_valid_solutions_, seeds, true);
        addConsistency(id, "C0_PASS_C1_FAIL", Configuration::LIFT_YAW, false,
                       results_c1_[id].metrics.failure_reason, std::move(repeated), !seeds.empty());
      }
      if (c0_[id].success && !results_c2_[id].metrics.success)
      {
        std::vector<const moveit::core::RobotState*> seeds;
        if (results_c1_[id].metrics.success) seeds.push_back(&results_c1_[id].state);
        FineResult repeated = evaluateFinePoint(points_[id], Configuration::LIFT_PITCH, special_ik_seeds_,
          special_required_valid_solutions_, seeds, true);
        addConsistency(id, "C0_PASS_C2_FAIL", Configuration::LIFT_PITCH, false,
                       results_c2_[id].metrics.failure_reason, std::move(repeated), !seeds.empty());
      }

      const bool c3_before = c3_[id].success;
      const bool lower_pass = results_c1_[id].metrics.success || results_c2_[id].metrics.success || c0_[id].success;
      if (lower_pass && !c3_before)
      {
        std::vector<const moveit::core::RobotState*> seeds;
        if (results_c1_[id].metrics.success) seeds.push_back(&results_c1_[id].state);
        if (results_c2_[id].metrics.success) seeds.push_back(&results_c2_[id].state);
        FineResult repeated = evaluateFinePoint(points_[id], Configuration::LIFT_YAW_PITCH, special_ik_seeds_,
          special_required_valid_solutions_, seeds, true);
        std::string pattern = c0_[id].success ? "C0_PASS_C3_FAIL" :
          (results_c1_[id].metrics.success && results_c2_[id].metrics.success ? "C1_C2_PASS_C3_FAIL" :
           results_c1_[id].metrics.success ? "C1_PASS_C3_FAIL" : "C2_PASS_C3_FAIL");
        addConsistency(id, pattern, Configuration::LIFT_YAW_PITCH, false, c3_[id].failure_reason,
                       std::move(repeated), !seeds.empty());
      }
    }
    RCLCPP_INFO(node_->get_logger(),
      "FIXED_BASE_WORKSPACE_DOF_ABLATION nested_consistency_candidates=%zu special_evaluations=%d",
      consistency_.size(), special_evaluations_);
  }

  void writeFineResultRow(std::ofstream& out, const FineResult& fine) const
  {
    const Result& r = fine.metrics;
    out << timestamp_ << ',' << r.point.id << ',' << number(r.point.xyz.x()) << ',' << number(r.point.xyz.y()) << ','
      << number(r.point.xyz.z()) << ',' << configName(r.configuration) << ',' << (r.success ? 1 : 0) << ','
      << r.failure_reason << ',' << r.seeds_tested << ',' << r.valid_count << ',' << number(r.lift) << ','
      << number(r.yaw) << ',' << number(r.pitch) << ',' << number(r.joint_margin) << ','
      << number(r.active_revolute_margin) << ',' << number(r.self_clearance) << ',' << number(r.manipulability) << ','
      << number(r.min_singular_value) << ',' << number(r.condition_number) << ',' << number(r.orientation_error) << ','
      << csvEscape(r.collision_pairs) << ',' << number(r.runtime_ms) << '\n';
  }

  void writeNewPoints() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_dof_ablation_new_points.csv", std::ios::trunc);
    out << "timestamp,point_id,tcp_x,tcp_y,tcp_z,configuration,success,failure_reason,ik_seeds_tested,valid_ik_count,"
      "selected_lift,selected_yaw,selected_pitch,min_joint_limit_margin,min_active_revolute_margin,"
      "self_collision_clearance,manipulability,min_jacobian_singular_value,jacobian_condition_number,"
      "orientation_error,collision_pairs,runtime_ms\n";
    for (std::size_t id = 0; id < points_.size(); ++id)
    {
      writeFineResultRow(out, results_c1_[id]);
      writeFineResultRow(out, results_c2_[id]);
    }
  }

  void writeConsistency() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_dof_ablation_consistency_check.csv", std::ios::trunc);
    out << "point_id,tcp_x,tcp_y,tcp_z,anomaly_pattern,target_configuration,lower_configuration_success,"
      "before_success,after_success,special_ik_seeds_tested,lower_success_state_seed_used,before_failure_reason,"
      "after_failure_reason,verdict\n";
    for (const auto& record : consistency_)
      out << record.point_id << ',' << number(points_[record.point_id].xyz.x()) << ','
        << number(points_[record.point_id].xyz.y()) << ',' << number(points_[record.point_id].xyz.z()) << ','
        << record.pattern << ',' << record.target_configuration << ',' << (record.lower_success ? 1 : 0) << ','
        << (record.before_success ? 1 : 0) << ',' << (record.after_success ? 1 : 0) << ',' << record.seeds_tested << ','
        << (record.lower_state_seed_used ? 1 : 0) << ',' << record.before_reason << ',' << record.after_reason << ','
        << record.verdict << '\n';
  }

  void writeC3Overrides() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_dof_ablation_c3_special_overrides.csv", std::ios::trunc);
    out << "timestamp,point_id,tcp_x,tcp_y,tcp_z,configuration,success,failure_reason,ik_seeds_tested,valid_ik_count,"
      "selected_lift,selected_yaw,selected_pitch,min_joint_limit_margin,min_active_revolute_margin,"
      "self_collision_clearance,manipulability,min_jacobian_singular_value,jacobian_condition_number,"
      "orientation_error,collision_pairs,runtime_ms\n";
    for (std::size_t id = 0; id < c3_override_.size(); ++id)
      if (c3_override_[id]) writeFineResultRow(out, *c3_override_[id]);
  }

  void writeAblationMetadata() const
  {
    std::ofstream out(output_dir_ + "/fixed_base_workspace_dof_ablation_metadata.csv", std::ios::trunc);
    out << "key,value\n"
      << "timestamp," << timestamp_ << '\n'
      << "physical_points," << points_.size() << '\n'
      << "new_configuration_evaluations," << points_.size() * 2 << '\n'
      << "c0_reused,true\nc3_reused,true\ncoarse_manifest_precheck,PASS\nfine_manifest_precheck,PASS\n"
      << "model_frame," << model_->getModelFrame() << '\n'
      << "base_frame," << base_frame_ << '\n' << "tcp_frame," << tcp_frame_ << '\n'
      << "voxel_dx," << number(voxel_.x()) << '\n' << "voxel_dy," << number(voxel_.y()) << '\n'
      << "voxel_dz," << number(voxel_.z()) << '\n' << "voxel_volume," << number(voxel_.prod()) << '\n'
      << "max_ik_seeds," << max_ik_seeds_ << '\n' << "max_special_ik_seeds," << special_ik_seeds_ << '\n'
      << "random_seed," << random_seed_ << '\n'
      << "orientation_qx," << number(target_q_.x()) << '\n' << "orientation_qy," << number(target_q_.y()) << '\n'
      << "orientation_qz," << number(target_q_.z()) << '\n' << "orientation_qw," << number(target_q_.w()) << '\n'
      << "orientation_tolerance_rad," << number(orientation_tolerance_) << '\n'
      << "urdf_xacro_sha256," << urdf_hash_ << '\n' << "srdf_sha256," << srdf_hash_ << '\n'
      << "joint_limits_sha256," << limits_hash_ << '\n' << "kinematics_sha256," << kinematics_hash_ << '\n'
      << "validity_implementation,INHERITED_FIXED_BASE_WORKSPACE_FINE_EVALUATOR\n"
      << "seed_policy,NEIGHBOR_THEN_DEFAULT_HALTON_FIXED_RANDOM_SEED_20260818\n"
      << "inactive_joint_lock,C1_PITCH_ZERO_C2_YAW_ZERO_REASSERTED_BEFORE_AND_AFTER_IK\n"
      << "environment_objects,0\ntrajectory_execution,false\namr_motion,false\ncontrollers,false\nros2_control,false\nhardware,false\n";
  }

  void runAblationPostprocessor() const
  {
    const pid_t child = fork();
    if (child < 0) throw std::runtime_error("fork failed for ablation postprocessor");
    if (child == 0)
    {
      execl(postprocess_ablation_.c_str(), postprocess_ablation_.c_str(), "--output-dir", output_dir_.c_str(),
            "--config", ablation_config_path_.c_str(), "--coarse-manifest", coarse_manifest_.c_str(),
            "--fine-manifest", fine_manifest_path_.c_str(), static_cast<char*>(nullptr));
      _exit(127);
    }
    int status = 0;
    if (waitpid(child, &status, 0) < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
      throw std::runtime_error("Ablation postprocessor failed");
  }

  std::string fine_points_csv_, fine_metadata_csv_, fine_manifest_path_, ablation_config_path_;
  std::string postprocess_ablation_, urdf_hash_, srdf_hash_, limits_hash_, kinematics_hash_;
  int max_new_evaluations_{}, max_special_evaluations_{}, special_evaluations_{};
  std::map<std::string, std::string> fine_metadata_;
  std::vector<ExistingResult> c0_, c3_;
  std::vector<FineResult> results_c1_, results_c2_;
  std::vector<std::unique_ptr<FineResult>> c3_override_;
  std::vector<ConsistencyRecord> consistency_;
};
}  // namespace fixed_base_workspace_dof_ablation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fixed_base_workspace_dof_ablation");
  try
  {
    fixed_base_workspace_dof_ablation::Runner runner(node);
    runner.runAblation();
    RCLCPP_INFO(node->get_logger(), "FIXED_BASE_WORKSPACE_DOF_ABLATION completed and sealed");
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "FIXED_BASE_WORKSPACE_DOF_ABLATION failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
