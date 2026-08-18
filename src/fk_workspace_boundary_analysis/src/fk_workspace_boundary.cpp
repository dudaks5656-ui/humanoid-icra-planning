#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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

struct Counters
{
  std::size_t total{ 0 };
  std::size_t valid{ 0 };
  std::size_t collision{ 0 };
  std::size_t bounds{ 0 };
  std::size_t exact_bound{ 0 };
  std::size_t internal{ 0 };
};

std::string name(Configuration config)
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
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    buildVariableContract();
  }

  void run()
  {
    preflight();
    const auto start = Clock::now();
    openOutputs();
    for (const auto config : { Configuration::C0, Configuration::C1, Configuration::C2, Configuration::C3 })
      sampleConfiguration(config, start);
    writeMetadata(Clock::now() - start);
    RCLCPP_INFO(node_->get_logger(),
      "FK_WORKSPACE_BOUNDARY COMPLETE total=%zu valid=[%zu,%zu,%zu,%zu] IK=NO execution=NO",
      total_states_, counters_[Configuration::C0].valid, counters_[Configuration::C1].valid,
      counters_[Configuration::C2].valid, counters_[Configuration::C3].valid);
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
    arm_group_name_ = parameter<std::string>("arm_group");
    full_group_name_ = parameter<std::string>("full_group");
    samples_per_configuration_ = parameter<int>("samples_per_configuration");
    max_states_per_configuration_ = parameter<int>("max_states_per_configuration");
    max_total_states_ = parameter<int>("max_total_states");
    random_seed_ = parameter<int>("random_seed");
    exact_bound_epsilon_ = parameter<double>("exact_bound_epsilon");
    max_wall_time_s_ = parameter<double>("max_wall_time_s");
    progress_every_ = parameter<int>("progress_every");
  }

  const moveit::core::JointModelGroup* requiredGroup(const std::string& group) const
  {
    const auto* value = model_->getJointModelGroup(group);
    if (!value) throw std::runtime_error("Missing MoveIt group: " + group);
    return value;
  }

  const moveit::core::LinkModel* requiredLink(const std::string& link) const
  {
    const auto* value = model_->getLinkModel(link);
    if (!value) throw std::runtime_error("Missing RobotModel link: " + link);
    return value;
  }

  void buildVariableContract()
  {
    canonical_names_ = { "lift_joint", "waist_yaw_joint", "waist_pitch_joint" };
    const auto& arm = arm_group_->getVariableNames();
    canonical_names_.insert(canonical_names_.end(), arm.begin(), arm.end());
    if (arm.size() != 7 || canonical_names_.size() != 10 || full_group_->getVariableCount() != 10)
      throw std::runtime_error("Expected 7 arm variables and 10 lift/yaw/pitch/arm variables");
    if (canonical_names_ != full_group_->getVariableNames())
      throw std::runtime_error("Canonical variables do not match left_arm_with_torso order");
    static constexpr std::array<unsigned, 10> primes{ 2, 3, 5, 7, 11, 13, 17, 19, 23, 29 };
    primes_ = primes;
    for (const auto& variable : canonical_names_)
    {
      const auto& bound = model_->getVariableBounds(variable);
      if (!bound.position_bounded_ || !(bound.max_position_ > bound.min_position_))
        throw std::runtime_error("FK sampling requires a finite actual joint limit: " + variable);
    }
  }

  std::vector<std::string> activeNames(Configuration config) const
  {
    std::vector<std::string> result{ "lift_joint" };
    if (config == Configuration::C1 || config == Configuration::C3) result.push_back("waist_yaw_joint");
    if (config == Configuration::C2 || config == Configuration::C3) result.push_back("waist_pitch_joint");
    const auto& arm = arm_group_->getVariableNames();
    result.insert(result.end(), arm.begin(), arm.end());
    return result;
  }

  void preflight() const
  {
    if (samples_per_configuration_ <= 0 || samples_per_configuration_ > max_states_per_configuration_ ||
        max_states_per_configuration_ > 10000)
      throw std::runtime_error("Per-configuration state count violates hard cap 10,000");
    const std::size_t expected = static_cast<std::size_t>(samples_per_configuration_) * 4;
    if (expected > static_cast<std::size_t>(max_total_states_) || max_total_states_ > 40000)
      throw std::runtime_error("Total state count violates hard cap 40,000");
    if (progress_every_ <= 0 || max_wall_time_s_ <= 0.0)
      throw std::runtime_error("Invalid progress/wall-time parameters");
    std::filesystem::create_directories(output_dir_);
    for (const auto& file : { "fk_workspace_boundary_states.csv", "fk_workspace_boundary_sampling_metadata.csv" })
      if (std::filesystem::exists(std::filesystem::path(output_dir_) / file))
        throw std::runtime_error("Refusing to overwrite FK workspace evidence: " + std::string(file));
    RCLCPP_INFO(node_->get_logger(),
      "FK_WORKSPACE_BOUNDARY PREFLIGHT configs=4 states_per_config=%d max_total=%d method=HALTON seed=%d "
      "base_fixed=YES IK=NO planner=NO controller=NO hardware=NO",
      samples_per_configuration_, max_total_states_, random_seed_);
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

  void setSample(moveit::core::RobotState& state, Configuration config, std::size_t sample_id) const
  {
    // The same Halton index and dimensions are shared across C0-C3.  Thus every
    // configuration sees identical arm/lift samples; only enabled waist variables differ.
    const std::uint64_t index = static_cast<std::uint64_t>(sample_id) + 1u +
      static_cast<std::uint64_t>(random_seed_ % 1000000);
    for (std::size_t dimension = 0; dimension < canonical_names_.size(); ++dimension)
    {
      const auto& variable = canonical_names_[dimension];
      const auto& bound = model_->getVariableBounds(variable);
      double value = bound.min_position_ + halton(index, primes_[dimension]) *
        (bound.max_position_ - bound.min_position_);
      if (variable == "waist_yaw_joint" && !(config == Configuration::C1 || config == Configuration::C3))
        value = 0.0;
      if (variable == "waist_pitch_joint" && !(config == Configuration::C2 || config == Configuration::C3))
        value = 0.0;
      state.setVariablePosition(variable, value);
    }
    state.update();
  }

  double jointMargin(const moveit::core::RobotState& state, const std::vector<std::string>& active) const
  {
    double margin = std::numeric_limits<double>::infinity();
    for (const auto& variable : active)
    {
      const auto& bound = model_->getVariableBounds(variable);
      const double value = state.getVariablePosition(variable);
      margin = std::min(margin, std::min(value - bound.min_position_, bound.max_position_ - value));
    }
    return margin;
  }

  void openOutputs()
  {
    states_.open(std::filesystem::path(output_dir_) / "fk_workspace_boundary_states.csv",
                 std::ios::out | std::ios::trunc);
    if (!states_) throw std::runtime_error("Cannot open FK state CSV");
    states_ << "configuration,sample_id,valid,failure_reason,tcp_x,tcp_y,tcp_z,lift,yaw,pitch,"
               "joint_margin,self_clearance,joint_names,joint_values\n";
  }

  std::string joinedValues(const moveit::core::RobotState& state) const
  {
    std::ostringstream out;
    out << std::setprecision(15);
    for (std::size_t i = 0; i < canonical_names_.size(); ++i)
    {
      if (i) out << ';';
      out << state.getVariablePosition(canonical_names_[i]);
    }
    return out.str();
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

  void writeRow(Configuration config, std::size_t sample_id, bool valid, const std::string& reason,
                const moveit::core::RobotState& state, double margin, double clearance,
                const Eigen::Vector3d* tcp)
  {
    states_ << name(config) << ',' << sample_id << ',' << (valid ? 1 : 0) << ',' << reason << ',';
    if (tcp) states_ << number(tcp->x()) << ',' << number(tcp->y()) << ',' << number(tcp->z());
    else states_ << ",,";
    states_ << ',' << number(state.getVariablePosition("lift_joint"))
            << ',' << number(state.getVariablePosition("waist_yaw_joint"))
            << ',' << number(state.getVariablePosition("waist_pitch_joint"))
            << ',' << number(margin) << ',' << number(clearance)
            << ',' << joinedNames() << ',' << joinedValues(state) << '\n';
  }

  void sampleConfiguration(Configuration config, const Clock::time_point& global_start)
  {
    const auto active = activeNames(config);
    auto& counter = counters_[config];
    for (int sample = 0; sample < samples_per_configuration_; ++sample)
    {
      const double elapsed = std::chrono::duration<double>(Clock::now() - global_start).count();
      if (elapsed > max_wall_time_s_) throw std::runtime_error("FK workspace wall-time hard limit exceeded");
      moveit::core::RobotState state = nominalState();
      setSample(state, config, static_cast<std::size_t>(sample));
      ++counter.total;
      ++total_states_;
      const double margin = jointMargin(state, active);
      if (!state.satisfiesBounds())
      {
        ++counter.bounds;
        writeRow(config, sample, false, "JOINT_LIMIT_VIOLATION", state, margin,
                 std::numeric_limits<double>::quiet_NaN(), nullptr);
        continue;
      }
      if (!(margin > exact_bound_epsilon_))
      {
        ++counter.exact_bound;
        writeRow(config, sample, false, "ACTIVE_JOINT_AT_BOUND", state, margin,
                 std::numeric_limits<double>::quiet_NaN(), nullptr);
        continue;
      }
      collision_detection::CollisionRequest request;
      request.contacts = false;
      collision_detection::CollisionResult collision;
      scene_->checkSelfCollision(request, collision, state);
      if (collision.collision)
      {
        ++counter.collision;
        writeRow(config, sample, false, "SELF_COLLISION", state, margin, 0.0, nullptr);
        continue;
      }
      state.update();
      const Eigen::Isometry3d base_in_model = state.getGlobalLinkTransform(base_link_);
      const Eigen::Vector3d tcp_in_base = base_in_model.inverse() *
        state.getGlobalLinkTransform(tcp_link_).translation();
      if (!tcp_in_base.allFinite())
      {
        ++counter.internal;
        writeRow(config, sample, false, "INTERNAL_ERROR", state, margin,
                 std::numeric_limits<double>::quiet_NaN(), nullptr);
        continue;
      }
      const double clearance = scene_->getCollisionEnv()->distanceSelf(
        state, scene_->getAllowedCollisionMatrix());
      ++counter.valid;
      writeRow(config, sample, true, "VALID", state, margin, clearance, &tcp_in_base);
      if ((sample + 1) % progress_every_ == 0)
        RCLCPP_INFO(node_->get_logger(),
          "FK_WORKSPACE_BOUNDARY config=%s sample=%d/%d valid=%zu collision=%zu tcp=(%.3f,%.3f,%.3f)",
          name(config).c_str(), sample + 1, samples_per_configuration_, counter.valid,
          counter.collision, tcp_in_base.x(), tcp_in_base.y(), tcp_in_base.z());
    }
  }

  template <typename Duration> void writeMetadata(Duration duration)
  {
    std::ofstream out(std::filesystem::path(output_dir_) / "fk_workspace_boundary_sampling_metadata.csv",
                      std::ios::out | std::ios::trunc);
    if (!out) throw std::runtime_error("Cannot open FK sampling metadata CSV");
    out << "key,value\n"
        << "sampling_method,HALTON_LOW_DISCREPANCY\n"
        << "sampling_contract,SHARED_ARM_LIFT_HALTON_DIMENSIONS_ACROSS_C0_C3\n"
        << "random_seed," << random_seed_ << '\n'
        << "base_frame," << base_frame_ << '\n'
        << "tcp_frame," << tcp_frame_ << '\n'
        << "samples_per_configuration," << samples_per_configuration_ << '\n'
        << "total_states," << total_states_ << '\n'
        << "exact_bound_epsilon," << number(exact_bound_epsilon_) << '\n'
        << "wall_time_s," << number(std::chrono::duration<double>(duration).count()) << '\n'
        << "joint_names," << joinedNames() << '\n';
    for (const auto& variable : canonical_names_)
    {
      const auto& bound = model_->getVariableBounds(variable);
      out << "joint_limit_" << variable << ',' << number(bound.min_position_) << ';'
          << number(bound.max_position_) << '\n';
    }
    for (const auto config : { Configuration::C0, Configuration::C1, Configuration::C2, Configuration::C3 })
    {
      const auto& c = counters_.at(config);
      out << name(config) << "_valid," << c.valid << '\n'
          << name(config) << "_self_collision_rejections," << c.collision << '\n'
          << name(config) << "_joint_limit_rejections," << c.bounds << '\n'
          << name(config) << "_exact_bound_rejections," << c.exact_bound << '\n'
          << name(config) << "_internal_rejections," << c.internal << '\n';
    }
    out << "ik_used,false\ntrajectory_execution,false\ncontroller,false\nros2_control,false\n"
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
  std::string output_dir_, base_frame_, tcp_frame_, arm_group_name_, full_group_name_;
  int samples_per_configuration_{ 0 }, max_states_per_configuration_{ 0 }, max_total_states_{ 0 };
  int random_seed_{ 0 }, progress_every_{ 0 };
  double exact_bound_epsilon_{ 0.0 }, max_wall_time_s_{ 0.0 };
  std::vector<std::string> canonical_names_;
  std::array<unsigned, 10> primes_{};
  std::map<Configuration, Counters> counters_;
  std::size_t total_states_{ 0 };
  std::ofstream states_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fk_workspace_boundary");
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
