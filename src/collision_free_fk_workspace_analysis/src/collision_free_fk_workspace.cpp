#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

enum class Configuration { C0, C1, C2, C3 };
enum class Family { BASE, YAW, PITCH, COMBINED };

struct Counters
{
  std::size_t attempts{ 0 }, valid{ 0 }, collision{ 0 }, bounds{ 0 }, exact_bound{ 0 }, internal{ 0 };
};

struct ValidRecord
{
  Family family;
  std::size_t sample_id;
  std::size_t sample_cap;
  std::string state_key;
  std::vector<double> positions;
  Eigen::Vector3d tcp;
  double self_clearance;
};

std::string configName(Configuration config)
{
  switch (config)
  {
    case Configuration::C0: return "LIFT_ONLY";
    case Configuration::C1: return "LIFT_YAW";
    case Configuration::C2: return "LIFT_PITCH";
    case Configuration::C3: return "LIFT_YAW_PITCH";
  }
  throw std::runtime_error("Unknown configuration");
}

std::string familyName(Family family)
{
  switch (family)
  {
    case Family::BASE: return "BASE";
    case Family::YAW: return "YAW";
    case Family::PITCH: return "PITCH";
    case Family::COMBINED: return "COMBINED";
  }
  throw std::runtime_error("Unknown family");
}

Configuration sourceConfiguration(Family family)
{
  if (family == Family::BASE) return Configuration::C0;
  if (family == Family::YAW) return Configuration::C1;
  if (family == Family::PITCH) return Configuration::C2;
  return Configuration::C3;
}

double halton(std::uint64_t index, unsigned base)
{
  double fraction = 1.0;
  double value = 0.0;
  while (index > 0)
  {
    fraction /= static_cast<double>(base);
    value += fraction * static_cast<double>(index % base);
    index /= base;
  }
  return value;
}

std::string number(double value)
{
  if (!std::isfinite(value)) return "";
  std::ostringstream out;
  out << std::setprecision(15) << value;
  return out.str();
}

class Runner
{
public:
  explicit Runner(rclcpp::Node::SharedPtr node) : node_(std::move(node))
  {
    loadParameters();
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    model_ = loader_->getModel();
    if (!model_) throw std::runtime_error("RobotModel/SRDF load failed");
    arm_group_ = requiredGroup(arm_group_name_);
    full_group_ = requiredGroup(full_group_name_);
    base_link_ = requiredLink(base_frame_);
    tcp_link_ = requiredLink(tcp_frame_);
    right_tcp_link_ = requiredLink(right_tcp_frame_);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    buildContract();
  }

  void run()
  {
    preflight();
    const auto start = Clock::now();
    openOutputs();
    for (const Family family : { Family::BASE, Family::YAW, Family::PITCH, Family::COMBINED })
      generateFamily(family, start);
    writeNestedPools();
    writeMetadata(Clock::now() - start);
    RCLCPP_INFO(node_->get_logger(),
      "COLLISION_FREE_FK COMPLETE checks=%zu pools=[%zu,%zu,%zu,%zu] IK=NO planner=NO execution=NO",
      total_checks_, poolSize(Configuration::C0), poolSize(Configuration::C1),
      poolSize(Configuration::C2), poolSize(Configuration::C3));
  }

private:
  template <typename T> T parameter(const std::string& key)
  {
    if (!node_->has_parameter(key)) node_->declare_parameter<T>(key);
    return node_->get_parameter(key).get_value<T>();
  }

  void loadParameters()
  {
    output_dir_ = parameter<std::string>("output_dir");
    base_frame_ = parameter<std::string>("base_frame");
    tcp_frame_ = parameter<std::string>("tcp_frame");
    right_tcp_frame_ = parameter<std::string>("right_tcp_frame");
    arm_group_name_ = parameter<std::string>("arm_group");
    full_group_name_ = parameter<std::string>("full_group");
    base_samples_ = parameter<int>("base_samples");
    yaw_samples_ = parameter<int>("yaw_enrichment_samples");
    pitch_samples_ = parameter<int>("pitch_enrichment_samples");
    combined_samples_ = parameter<int>("combined_enrichment_samples");
    max_base_samples_ = parameter<int>("max_base_samples");
    max_total_checks_ = parameter<int>("max_total_collision_checks");
    random_seed_ = parameter<int>("random_seed");
    epsilon_ = parameter<double>("exact_bound_epsilon");
    max_wall_s_ = parameter<double>("max_wall_time_s");
    progress_every_ = parameter<int>("progress_every");
  }

  const moveit::core::JointModelGroup* requiredGroup(const std::string& name) const
  {
    const auto* group = model_->getJointModelGroup(name);
    if (!group) throw std::runtime_error("Missing MoveIt group: " + name);
    return group;
  }

  const moveit::core::LinkModel* requiredLink(const std::string& name) const
  {
    const auto* link = model_->getLinkModel(name);
    if (!link) throw std::runtime_error("Missing RobotModel link: " + name);
    return link;
  }

  void buildContract()
  {
    canonical_names_ = { "lift_joint", "waist_yaw_joint", "waist_pitch_joint" };
    const auto& arm = arm_group_->getVariableNames();
    canonical_names_.insert(canonical_names_.end(), arm.begin(), arm.end());
    if (arm.size() != 7 || canonical_names_.size() != 10 ||
        canonical_names_ != full_group_->getVariableNames())
      throw std::runtime_error("Expected lift/yaw/pitch/7-arm canonical variable order");
    primes_ = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29 };
    for (const auto& variable : canonical_names_)
    {
      const auto& bound = model_->getVariableBounds(variable);
      if (!bound.position_bounded_ || !(bound.max_position_ > bound.min_position_))
        throw std::runtime_error("Finite URDF/MoveIt joint bounds required: " + variable);
    }
  }

  std::size_t familyCap(Family family) const
  {
    if (family == Family::BASE) return static_cast<std::size_t>(base_samples_);
    if (family == Family::YAW) return static_cast<std::size_t>(yaw_samples_);
    if (family == Family::PITCH) return static_cast<std::size_t>(pitch_samples_);
    return static_cast<std::size_t>(combined_samples_);
  }

  void preflight() const
  {
    const auto state = nominalState();
    for (const auto* tcp : { tcp_link_, right_tcp_link_ })
    {
      const Eigen::Isometry3d local = state.getGlobalLinkTransform(tcp->getParentLinkModel()).inverse() *
        state.getGlobalLinkTransform(tcp);
      const Eigen::Vector3d expected(0.0, 0.0, 0.0345);
      if (!local.translation().isApprox(expected, 1.0e-9) ||
          !local.rotation().isApprox(Eigen::Matrix3d::Identity(), 1.0e-12))
        throw std::runtime_error("Corrected grasp TCP preflight failed for " + tcp->getName() +
          ": expected xyz=[0,0,0.0345], RPY=[0,0,0]");
    }
    const long total = static_cast<long>(base_samples_) + yaw_samples_ + pitch_samples_ + combined_samples_;
    if (base_samples_ < 10000 || base_samples_ > max_base_samples_ || max_base_samples_ > 20000 ||
        yaw_samples_ <= 0 || pitch_samples_ <= 0 || combined_samples_ <= 0 ||
        total > max_total_checks_ || max_total_checks_ > 40000)
      throw std::runtime_error("Sampling hard cap or base-sample contract violated");
    if (progress_every_ <= 0 || max_wall_s_ <= 0.0)
      throw std::runtime_error("Invalid progress/wall-time parameter");
    std::filesystem::create_directories(output_dir_);
    for (const auto* file : { "collision_free_fk_workspace_states.csv",
                              "collision_free_fk_workspace_sampling_metadata.csv" })
      if (std::filesystem::exists(std::filesystem::path(output_dir_) / file))
        throw std::runtime_error(std::string("Refusing to overwrite: ") + file);
    RCLCPP_INFO(node_->get_logger(),
      "COLLISION_FREE_FK PREFLIGHT hand=LEFT tcp=%s base=%d yaw=%d pitch=%d combined=%d checks=%ld "
      "method=HALTON seed=%d nested=STRUCTURAL environment=NONE",
      tcp_frame_.c_str(), base_samples_, yaw_samples_, pitch_samples_, combined_samples_, total, random_seed_);
  }

  moveit::core::RobotState nominalState() const
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    for (const std::string finger : { "openarm_left_finger_joint1", "openarm_right_finger_joint1" })
    {
      const auto& bound = model_->getVariableBounds(finger);
      if (bound.position_bounded_)
        state.setVariablePosition(finger, 0.5 * (bound.min_position_ + bound.max_position_));
    }
    state.update();
    return state;
  }

  bool yawEnabled(Family family) const { return family == Family::YAW || family == Family::COMBINED; }
  bool pitchEnabled(Family family) const { return family == Family::PITCH || family == Family::COMBINED; }
  bool yawEnabled(Configuration config) const { return config == Configuration::C1 || config == Configuration::C3; }
  bool pitchEnabled(Configuration config) const { return config == Configuration::C2 || config == Configuration::C3; }

  void setSample(moveit::core::RobotState& state, Family family, std::size_t sample) const
  {
    const std::uint64_t index = static_cast<std::uint64_t>(sample) + 1u +
      static_cast<std::uint64_t>(random_seed_ % 1000000);
    for (std::size_t dimension = 0; dimension < canonical_names_.size(); ++dimension)
    {
      const auto& variable = canonical_names_[dimension];
      const auto& bound = model_->getVariableBounds(variable);
      double value = bound.min_position_ + halton(index, primes_[dimension]) *
        (bound.max_position_ - bound.min_position_);
      if (variable == "waist_yaw_joint" && !yawEnabled(family)) value = 0.0;
      if (variable == "waist_pitch_joint" && !pitchEnabled(family)) value = 0.0;
      state.setVariablePosition(variable, value);
    }
    state.update();
  }

  std::vector<std::string> activeNames(Configuration config) const
  {
    std::vector<std::string> names{ "lift_joint" };
    if (yawEnabled(config)) names.push_back("waist_yaw_joint");
    if (pitchEnabled(config)) names.push_back("waist_pitch_joint");
    const auto& arm = arm_group_->getVariableNames();
    names.insert(names.end(), arm.begin(), arm.end());
    return names;
  }

  double jointMargin(const moveit::core::RobotState& state, Configuration config) const
  {
    double margin = std::numeric_limits<double>::infinity();
    for (const auto& variable : activeNames(config))
    {
      const auto& bound = model_->getVariableBounds(variable);
      const double value = state.getVariablePosition(variable);
      margin = std::min(margin, std::min(value - bound.min_position_, bound.max_position_ - value));
    }
    return margin;
  }

  double manipulability(const moveit::core::RobotState& state, Configuration config) const
  {
    Eigen::MatrixXd full_jacobian;
    if (!state.getJacobian(full_group_, tcp_link_, Eigen::Vector3d::Zero(), full_jacobian)) return NAN;
    const auto active = activeNames(config);
    Eigen::MatrixXd selected(full_jacobian.rows(), static_cast<Eigen::Index>(active.size()));
    const auto& full_names = full_group_->getVariableNames();
    for (std::size_t column = 0; column < active.size(); ++column)
    {
      const auto it = std::find(full_names.begin(), full_names.end(), active[column]);
      if (it == full_names.end()) return NAN;
      selected.col(static_cast<Eigen::Index>(column)) =
        full_jacobian.col(static_cast<Eigen::Index>(std::distance(full_names.begin(), it)));
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(selected, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double product = 1.0;
    for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) product *= svd.singularValues()(i);
    return product;
  }

  Eigen::Vector3d tcpInBase(const moveit::core::RobotState& state, const moveit::core::LinkModel* tcp) const
  {
    return state.getGlobalLinkTransform(base_link_).inverse() * state.getGlobalLinkTransform(tcp).translation();
  }

  std::vector<double> positions(const moveit::core::RobotState& state) const
  {
    std::vector<double> values;
    values.reserve(canonical_names_.size());
    for (const auto& name : canonical_names_) values.push_back(state.getVariablePosition(name));
    return values;
  }

  moveit::core::RobotState stateFrom(const std::vector<double>& values) const
  {
    auto state = nominalState();
    for (std::size_t i = 0; i < canonical_names_.size(); ++i)
      state.setVariablePosition(canonical_names_[i], values[i]);
    state.update();
    return state;
  }

  std::string joinedNames() const
  {
    std::ostringstream out;
    for (std::size_t i = 0; i < canonical_names_.size(); ++i)
    {
      if (i) out << ';';
      out << canonical_names_[i];
    }
    return out.str();
  }

  std::string joinedValues(const std::vector<double>& values) const
  {
    std::ostringstream out;
    out << std::setprecision(15);
    for (std::size_t i = 0; i < values.size(); ++i)
    {
      if (i) out << ';';
      out << values[i];
    }
    return out.str();
  }

  void openOutputs()
  {
    states_.open(std::filesystem::path(output_dir_) / "collision_free_fk_workspace_states.csv",
                 std::ios::out | std::ios::trunc);
    if (!states_) throw std::runtime_error("Cannot open state CSV");
    states_ << "configuration,state_id,inherited_or_new,source_configuration,origin_family,"
               "origin_sample_id,origin_sample_cap,valid_joint_limits,self_collision,failure_reason,"
               "lift,waist_yaw,waist_pitch,tcp_x,tcp_y,tcp_z,joint_margin,self_clearance,"
               "manipulability,joint_names,joint_values\n";
  }

  void writeAttemptFailure(Family family, std::size_t sample, const moveit::core::RobotState& state,
                           bool bounds_valid, bool collision, const std::string& reason, double margin)
  {
    const auto config = sourceConfiguration(family);
    const auto values = positions(state);
    states_ << configName(config) << ',' << familyName(family) << '_' << sample << ",NEW,"
            << configName(config) << ',' << familyName(family) << ',' << sample << ',' << familyCap(family)
            << ',' << (bounds_valid ? 1 : 0) << ',' << (collision ? 1 : 0) << ',' << reason << ','
            << number(state.getVariablePosition("lift_joint")) << ','
            << number(state.getVariablePosition("waist_yaw_joint")) << ','
            << number(state.getVariablePosition("waist_pitch_joint")) << ",,,," << number(margin)
            << ',' << (collision ? "0" : "") << ",," << joinedNames() << ',' << joinedValues(values) << '\n';
    ++output_rows_;
  }

  void generateFamily(Family family, const Clock::time_point& start)
  {
    auto& counter = counters_[family];
    const auto config = sourceConfiguration(family);
    const std::size_t cap = familyCap(family);
    for (std::size_t sample = 0; sample < cap; ++sample)
    {
      if (std::chrono::duration<double>(Clock::now() - start).count() > max_wall_s_)
        throw std::runtime_error("Workspace wall-time hard limit exceeded");
      if (++total_checks_ > static_cast<std::size_t>(max_total_checks_))
        throw std::runtime_error("Collision-check hard cap exceeded");
      auto state = nominalState();
      setSample(state, family, sample);
      ++counter.attempts;
      const double margin = jointMargin(state, config);
      if (!state.satisfiesBounds())
      {
        ++counter.bounds;
        writeAttemptFailure(family, sample, state, false, false, "JOINT_LIMIT_VIOLATION", margin);
        continue;
      }
      if (!(margin > epsilon_))
      {
        ++counter.exact_bound;
        writeAttemptFailure(family, sample, state, true, false, "ACTIVE_JOINT_AT_BOUND", margin);
        continue;
      }
      collision_detection::CollisionRequest request;
      request.contacts = false;
      collision_detection::CollisionResult collision;
      scene_->checkSelfCollision(request, collision, state);
      if (collision.collision)
      {
        ++counter.collision;
        writeAttemptFailure(family, sample, state, true, true, "SELF_COLLISION", margin);
        continue;
      }
      const Eigen::Vector3d tcp = tcpInBase(state, tcp_link_);
      if (!tcp.allFinite())
      {
        ++counter.internal;
        writeAttemptFailure(family, sample, state, true, false, "INTERNAL_ERROR", margin);
        continue;
      }
      const double clearance = scene_->getCollisionEnv()->distanceSelf(
        state, scene_->getAllowedCollisionMatrix());
      ++counter.valid;
      valid_[family].push_back(ValidRecord{ family, sample, cap, familyName(family) + '_' +
        std::to_string(sample), positions(state), tcp, clearance });
      if ((sample + 1) % static_cast<std::size_t>(progress_every_) == 0)
        RCLCPP_INFO(node_->get_logger(),
          "COLLISION_FREE_FK family=%s sample=%zu/%zu valid=%zu collision=%zu tcp=(%.3f,%.3f,%.3f)",
          familyName(family).c_str(), sample + 1, cap, counter.valid, counter.collision,
          tcp.x(), tcp.y(), tcp.z());
    }
  }

  std::vector<Family> families(Configuration config) const
  {
    if (config == Configuration::C0) return { Family::BASE };
    if (config == Configuration::C1) return { Family::BASE, Family::YAW };
    if (config == Configuration::C2) return { Family::BASE, Family::PITCH };
    return { Family::BASE, Family::YAW, Family::PITCH, Family::COMBINED };
  }

  std::size_t poolSize(Configuration config) const
  {
    std::size_t size = 0;
    for (const auto family : families(config)) size += valid_.at(family).size();
    return size;
  }

  void writeNestedPools()
  {
    for (const auto config : { Configuration::C0, Configuration::C1, Configuration::C2, Configuration::C3 })
    {
      for (const auto family : families(config))
      {
        const auto source = sourceConfiguration(family);
        const bool inherited = source != config;
        for (const auto& record : valid_.at(family))
        {
          auto state = stateFrom(record.positions);
          const double margin = jointMargin(state, config);
          const double measure = manipulability(state, config);
          states_ << configName(config) << ',' << record.state_key << ','
                  << (inherited ? "INHERITED" : "NEW") << ',' << configName(source) << ','
                  << familyName(family) << ',' << record.sample_id << ',' << record.sample_cap
                  << ",1,0,VALID," << number(state.getVariablePosition("lift_joint")) << ','
                  << number(state.getVariablePosition("waist_yaw_joint")) << ','
                  << number(state.getVariablePosition("waist_pitch_joint")) << ','
                  << number(record.tcp.x()) << ',' << number(record.tcp.y()) << ',' << number(record.tcp.z())
                  << ',' << number(margin) << ',' << number(record.self_clearance) << ',' << number(measure)
                  << ',' << joinedNames() << ',' << joinedValues(record.positions) << '\n';
          ++output_rows_;
        }
      }
    }
  }

  template <typename Duration> void writeMetadata(Duration elapsed)
  {
    std::ofstream out(std::filesystem::path(output_dir_) /
      "collision_free_fk_workspace_sampling_metadata.csv", std::ios::out | std::ios::trunc);
    if (!out) throw std::runtime_error("Cannot open metadata CSV");
    auto nominal = nominalState();
    const auto left_local = nominal.getGlobalLinkTransform(tcp_link_->getParentLinkModel()).inverse() *
      nominal.getGlobalLinkTransform(tcp_link_);
    const auto right_local = nominal.getGlobalLinkTransform(right_tcp_link_->getParentLinkModel()).inverse() *
      nominal.getGlobalLinkTransform(right_tcp_link_);
    out << "key,value\n"
        << "workspace_type,COLLISION_FREE_CONFIGURATION_FK_WORKSPACE\n"
        << "sampling_method,HALTON_LOW_DISCREPANCY_NESTED_STATE_POOL\n"
        << "random_seed," << random_seed_ << '\n'
        << "analysis_hand,LEFT\n"
        << "base_frame," << base_frame_ << '\n'
        << "tcp_frame," << tcp_frame_ << '\n'
        << "tcp_parent," << tcp_link_->getParentLinkModel()->getName() << '\n'
        << "tcp_xyz," << number(left_local.translation().x()) << ';'
        << number(left_local.translation().y()) << ';' << number(left_local.translation().z()) << '\n'
        << "tcp_rpy,0;0;0\n"
        << "right_tcp_frame," << right_tcp_frame_ << '\n'
        << "right_tcp_parent," << right_tcp_link_->getParentLinkModel()->getName() << '\n'
        << "right_tcp_xyz," << number(right_local.translation().x()) << ';'
        << number(right_local.translation().y()) << ';' << number(right_local.translation().z()) << '\n'
        << "right_tcp_rpy,0;0;0\n"
        << "base_samples," << base_samples_ << '\n'
        << "yaw_enrichment_samples," << yaw_samples_ << '\n'
        << "pitch_enrichment_samples," << pitch_samples_ << '\n'
        << "combined_enrichment_samples," << combined_samples_ << '\n'
        << "total_collision_checks," << total_checks_ << '\n'
        << "output_rows," << output_rows_ << '\n'
        << "wall_time_s," << number(std::chrono::duration<double>(elapsed).count()) << '\n'
        << "joint_names," << joinedNames() << '\n';
    for (const auto& variable : canonical_names_)
    {
      const auto& bound = model_->getVariableBounds(variable);
      out << "joint_limit_" << variable << ',' << number(bound.min_position_) << ';'
          << number(bound.max_position_) << '\n';
    }
    for (const auto family : { Family::BASE, Family::YAW, Family::PITCH, Family::COMBINED })
    {
      const auto& count = counters_.at(family);
      out << familyName(family) << "_attempts," << count.attempts << '\n'
          << familyName(family) << "_valid," << count.valid << '\n'
          << familyName(family) << "_self_collision_rejections," << count.collision << '\n'
          << familyName(family) << "_joint_limit_rejections," << count.bounds << '\n'
          << familyName(family) << "_exact_bound_rejections," << count.exact_bound << '\n'
          << familyName(family) << "_internal_rejections," << count.internal << '\n';
    }
    for (const auto config : { Configuration::C0, Configuration::C1, Configuration::C2, Configuration::C3 })
      out << configName(config) << "_nested_pool_states," << poolSize(config) << '\n';
    out << "environment_objects,false\nbox,false\nIK_used_for_workspace,false\n"
           "OMPL_used,false\ntrajectory_execution,false\ncontroller,false\nros2_control,false\n"
           "hardware,false\namr_motion,false\n";
  }

  rclcpp::Node::SharedPtr node_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  const moveit::core::JointModelGroup* arm_group_{ nullptr };
  const moveit::core::JointModelGroup* full_group_{ nullptr };
  const moveit::core::LinkModel* base_link_{ nullptr };
  const moveit::core::LinkModel* tcp_link_{ nullptr };
  const moveit::core::LinkModel* right_tcp_link_{ nullptr };
  std::string output_dir_, base_frame_, tcp_frame_, right_tcp_frame_, arm_group_name_, full_group_name_;
  int base_samples_{ 0 }, yaw_samples_{ 0 }, pitch_samples_{ 0 }, combined_samples_{ 0 };
  int max_base_samples_{ 0 }, max_total_checks_{ 0 }, random_seed_{ 0 }, progress_every_{ 0 };
  double epsilon_{ 0.0 }, max_wall_s_{ 0.0 };
  std::vector<std::string> canonical_names_;
  std::array<unsigned, 10> primes_{};
  std::map<Family, Counters> counters_;
  std::map<Family, std::vector<ValidRecord>> valid_;
  std::size_t total_checks_{ 0 }, output_rows_{ 0 };
  std::ofstream states_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("collision_free_fk_workspace");
  try
  {
    Runner(node).run();
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
