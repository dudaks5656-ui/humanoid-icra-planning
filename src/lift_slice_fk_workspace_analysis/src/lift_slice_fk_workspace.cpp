#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>

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
  std::size_t attempts{0}, valid{0}, collision{0}, bounds{0}, exact_bound{0}, internal{0};
};

std::string name(Configuration c)
{
  if (c == Configuration::C0) return "LIFT_ONLY";
  if (c == Configuration::C1) return "LIFT_YAW";
  if (c == Configuration::C2) return "LIFT_PITCH";
  return "LIFT_YAW_PITCH";
}

double halton(std::uint64_t index, unsigned base)
{
  double fraction = 1.0, value = 0.0;
  while (index > 0) { fraction /= base; value += fraction * (index % base); index /= base; }
  return value;
}

std::string number(double v)
{
  if (!std::isfinite(v)) return "";
  std::ostringstream out; out << std::setprecision(15) << v; return out.str();
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
    reference_link_model_ = requiredLink(reference_link_);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    buildContract();
  }

  void run()
  {
    preflight();
    const auto start = Clock::now();
    openOutputs();
    const auto& lift_bound = model_->getVariableBounds("lift_joint");
    for (std::size_t slice = 0; slice < ratios_.size(); ++slice)
    {
      const double lift = lift_bound.min_position_ + ratios_[slice] *
        (lift_bound.max_position_ - lift_bound.min_position_);
      references_[slice] = referencePosition(lift);
      for (auto config : {Configuration::C0, Configuration::C1, Configuration::C2, Configuration::C3})
        sampleSlice(config, slice, lift, start);
    }
    writeMetadata(Clock::now() - start);
    RCLCPP_INFO(node_->get_logger(), "LIFT_SLICE_FK COMPLETE states=%zu IK=NO OMPL=NO execution=NO", total_);
  }

private:
  template<class T> T parameter(const std::string& key)
  {
    if (!node_->has_parameter(key)) node_->declare_parameter<T>(key);
    return node_->get_parameter(key).get_value<T>();
  }

  void loadParameters()
  {
    output_dir_ = parameter<std::string>("output_dir");
    base_frame_ = parameter<std::string>("base_frame"); tcp_frame_ = parameter<std::string>("tcp_frame");
    reference_link_ = parameter<std::string>("reference_link"); arm_group_name_ = parameter<std::string>("arm_group");
    full_group_name_ = parameter<std::string>("full_group");
    samples_ = parameter<int>("samples_per_slice_configuration");
    max_samples_ = parameter<int>("max_samples_per_slice_configuration");
    max_total_ = parameter<int>("max_total_states"); random_seed_ = parameter<int>("random_seed");
    epsilon_ = parameter<double>("exact_bound_epsilon"); max_wall_s_ = parameter<double>("max_wall_time_s");
    progress_every_ = parameter<int>("progress_every");
  }

  const moveit::core::JointModelGroup* requiredGroup(const std::string& n) const
  {
    const auto* g = model_->getJointModelGroup(n); if (!g) throw std::runtime_error("Missing group: " + n); return g;
  }
  const moveit::core::LinkModel* requiredLink(const std::string& n) const
  {
    const auto* l = model_->getLinkModel(n); if (!l) throw std::runtime_error("Missing link: " + n); return l;
  }

  void buildContract()
  {
    canonical_ = {"lift_joint", "waist_yaw_joint", "waist_pitch_joint"};
    const auto& arm = arm_group_->getVariableNames(); canonical_.insert(canonical_.end(), arm.begin(), arm.end());
    if (arm.size() != 7 || canonical_.size() != 10 || canonical_ != full_group_->getVariableNames())
      throw std::runtime_error("Expected canonical lift/yaw/pitch/7-arm variable contract");
    primes_ = {2,3,5,7,11,13,17,19,23};
    for (const auto& n : canonical_)
    {
      const auto& b = model_->getVariableBounds(n);
      if (!b.position_bounded_ || !(b.max_position_ > b.min_position_))
        throw std::runtime_error("Finite actual joint bounds required: " + n);
    }
  }

  void preflight() const
  {
    const long expected = static_cast<long>(samples_) * 4L * 5L;
    if (samples_ <= 0 || samples_ > max_samples_ || max_samples_ > 2000 || expected > max_total_ || max_total_ > 40000)
      throw std::runtime_error("40,000-state hard cap violated");
    std::filesystem::create_directories(output_dir_);
    for (const auto* f : {"lift_slice_fk_workspace_states.csv", "lift_slice_fk_workspace_sampling_metadata.csv"})
      if (std::filesystem::exists(std::filesystem::path(output_dir_) / f))
        throw std::runtime_error(std::string("Refusing overwrite: ") + f);
    RCLCPP_INFO(node_->get_logger(),
      "LIFT_SLICE_FK PREFLIGHT configs=4 slices=5 attempts_per_slice_config=%d total=%ld method=HALTON seed=%d base_fixed=YES",
      samples_, expected, random_seed_);
  }

  moveit::core::RobotState nominal() const
  {
    moveit::core::RobotState state(model_); state.setToDefaultValues();
    for (const std::string n : {"openarm_left_finger_joint1", "openarm_right_finger_joint1"})
    {
      const auto& b = model_->getVariableBounds(n);
      if (b.position_bounded_) state.setVariablePosition(n, 0.5 * (b.min_position_ + b.max_position_));
    }
    state.update(); return state;
  }

  bool yawEnabled(Configuration c) const { return c == Configuration::C1 || c == Configuration::C3; }
  bool pitchEnabled(Configuration c) const { return c == Configuration::C2 || c == Configuration::C3; }

  void setSample(moveit::core::RobotState& state, Configuration c, double lift, std::size_t sample) const
  {
    state.setVariablePosition("lift_joint", lift);
    const std::uint64_t index = sample + 1u + static_cast<std::uint64_t>(random_seed_ % 1000000);
    for (std::size_t d = 0; d < sample_variables_.size(); ++d)
    {
      const auto& n = sample_variables_[d]; const auto& b = model_->getVariableBounds(n);
      double value = b.min_position_ + halton(index, primes_[d]) * (b.max_position_ - b.min_position_);
      if (n == "waist_yaw_joint" && !yawEnabled(c)) value = 0.0;
      if (n == "waist_pitch_joint" && !pitchEnabled(c)) value = 0.0;
      state.setVariablePosition(n, value);
    }
    state.update();
  }

  std::vector<std::string> sampledActive(Configuration c) const
  {
    std::vector<std::string> result;
    if (yawEnabled(c)) result.push_back("waist_yaw_joint");
    if (pitchEnabled(c)) result.push_back("waist_pitch_joint");
    const auto& arm = arm_group_->getVariableNames(); result.insert(result.end(), arm.begin(), arm.end());
    return result;
  }

  double margin(const moveit::core::RobotState& state, const std::vector<std::string>& active) const
  {
    double result = std::numeric_limits<double>::infinity();
    for (const auto& n : active)
    {
      const auto& b = model_->getVariableBounds(n); const double q = state.getVariablePosition(n);
      result = std::min(result, std::min(q - b.min_position_, b.max_position_ - q));
    }
    return result;
  }

  Eigen::Vector3d baseRelative(const moveit::core::RobotState& state, const moveit::core::LinkModel* link) const
  {
    return state.getGlobalLinkTransform(base_link_).inverse() * state.getGlobalLinkTransform(link).translation();
  }

  Eigen::Vector3d referencePosition(double lift) const
  {
    auto state = nominal(); state.setVariablePosition("lift_joint", lift);
    state.setVariablePosition("waist_yaw_joint", 0.0); state.setVariablePosition("waist_pitch_joint", 0.0);
    state.update(); return baseRelative(state, reference_link_model_);
  }

  std::string joinedNames() const
  {
    std::ostringstream out; for (std::size_t i=0;i<canonical_.size();++i) { if(i) out<<';'; out<<canonical_[i]; } return out.str();
  }
  std::string joinedValues(const moveit::core::RobotState& state) const
  {
    std::ostringstream out; out<<std::setprecision(15);
    for (std::size_t i=0;i<canonical_.size();++i) { if(i) out<<';'; out<<state.getVariablePosition(canonical_[i]); }
    return out.str();
  }

  void openOutputs()
  {
    states_.open(std::filesystem::path(output_dir_) / "lift_slice_fk_workspace_states.csv", std::ios::out|std::ios::trunc);
    if (!states_) throw std::runtime_error("Cannot open state CSV");
    states_ << "configuration,lift_ratio,lift_value,sample_id,tcp_x,tcp_y,tcp_z,yaw,pitch,joint_margin,self_clearance,valid,failure_reason,joint_names,joint_values\n";
  }

  void writeRow(Configuration c, std::size_t slice, double lift, std::size_t sample, bool valid,
                const std::string& reason, const moveit::core::RobotState& state, double m, double clearance,
                const Eigen::Vector3d* tcp)
  {
    states_ << name(c)<<','<<number(ratios_[slice])<<','<<number(lift)<<','<<sample<<',';
    if (tcp) states_<<number(tcp->x())<<','<<number(tcp->y())<<','<<number(tcp->z()); else states_<<",,";
    states_ << ','<<number(state.getVariablePosition("waist_yaw_joint"))<<','
            <<number(state.getVariablePosition("waist_pitch_joint"))<<','<<number(m)<<','<<number(clearance)<<','
            <<(valid?1:0)<<','<<reason<<','<<joinedNames()<<','<<joinedValues(state)<<'\n';
  }

  void sampleSlice(Configuration c, std::size_t slice, double lift, const Clock::time_point& start)
  {
    const auto active = sampledActive(c); auto& count = counters_[{c,slice}];
    for (int sample=0; sample<samples_; ++sample)
    {
      if (std::chrono::duration<double>(Clock::now()-start).count() > max_wall_s_)
        throw std::runtime_error("Wall-time hard limit exceeded");
      auto state = nominal(); setSample(state,c,lift,sample); ++count.attempts; ++total_;
      const double m = margin(state,active);
      if (!state.satisfiesBounds()) { ++count.bounds; writeRow(c,slice,lift,sample,false,"JOINT_LIMIT_VIOLATION",state,m,NAN,nullptr); continue; }
      if (!(m > epsilon_)) { ++count.exact_bound; writeRow(c,slice,lift,sample,false,"ACTIVE_JOINT_AT_BOUND",state,m,NAN,nullptr); continue; }
      collision_detection::CollisionRequest request; request.contacts=false;
      collision_detection::CollisionResult collision; scene_->checkSelfCollision(request,collision,state);
      if (collision.collision) { ++count.collision; writeRow(c,slice,lift,sample,false,"SELF_COLLISION",state,m,0.0,nullptr); continue; }
      const Eigen::Vector3d tcp = baseRelative(state,tcp_link_);
      if (!tcp.allFinite()) { ++count.internal; writeRow(c,slice,lift,sample,false,"INTERNAL_ERROR",state,m,NAN,nullptr); continue; }
      const double clearance = scene_->getCollisionEnv()->distanceSelf(state,scene_->getAllowedCollisionMatrix());
      ++count.valid; writeRow(c,slice,lift,sample,true,"VALID",state,m,clearance,&tcp);
      if ((sample+1)%progress_every_==0)
        RCLCPP_INFO(node_->get_logger(),"LIFT_SLICE_FK config=%s lift=%.3f sample=%d/%d valid=%zu",name(c).c_str(),lift,sample+1,samples_,count.valid);
    }
  }

  template<class Duration> void writeMetadata(Duration elapsed)
  {
    std::ofstream out(std::filesystem::path(output_dir_) / "lift_slice_fk_workspace_sampling_metadata.csv",std::ios::out|std::ios::trunc);
    out<<"key,value\n"<<"sampling_method,HALTON_LOW_DISCREPANCY\n"<<"random_seed,"<<random_seed_<<'\n'
       <<"base_frame,"<<base_frame_<<'\n'<<"tcp_frame,"<<tcp_frame_<<'\n'<<"reference_link,"<<reference_link_<<'\n'
       <<"samples_per_slice_configuration,"<<samples_<<'\n'<<"total_states,"<<total_<<'\n'
       <<"lift_exact_bound_policy,FIXED_SLICE_CONTEXT_EXEMPT_ACTIVE_ARM_YAW_PITCH_STILL_CHECKED\n"
       <<"lift_axis_semantics,Q_MIN_TOPMOST_Q_INCREASES_ROS_NEGATIVE_Z\n"
       <<"wall_time_s,"<<number(std::chrono::duration<double>(elapsed).count())<<'\n'<<"joint_names,"<<joinedNames()<<'\n';
    for (const auto& n:canonical_) { const auto& b=model_->getVariableBounds(n); out<<"joint_limit_"<<n<<','<<number(b.min_position_)<<';'<<number(b.max_position_)<<'\n'; }
    const auto& lb=model_->getVariableBounds("lift_joint");
    for(std::size_t s=0;s<ratios_.size();++s) {
      const double lift=lb.min_position_+ratios_[s]*(lb.max_position_-lb.min_position_);
      out<<"lift_slice_"<<s<<','<<number(ratios_[s])<<';'<<number(lift)<<'\n';
      out<<"reference_xyz_"<<s<<','<<number(references_[s].x())<<';'<<number(references_[s].y())<<';'<<number(references_[s].z())<<'\n';
    }
    for(const auto& entry:counters_) {
      const auto& c=entry.second; out<<name(entry.first.first)<<"_slice_"<<entry.first.second<<"_valid,"<<c.valid<<'\n'
        <<name(entry.first.first)<<"_slice_"<<entry.first.second<<"_self_collision_rejections,"<<c.collision<<'\n';
    }
    out<<"ik_used,false\nompl_used,false\ntrajectory_execution,false\ncontroller,false\nros2_control,false\nhardware,false\namr_motion,false\n";
  }

  rclcpp::Node::SharedPtr node_; robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr model_; planning_scene::PlanningScenePtr scene_;
  const moveit::core::JointModelGroup *arm_group_{nullptr},*full_group_{nullptr};
  const moveit::core::LinkModel *base_link_{nullptr},*tcp_link_{nullptr},*reference_link_model_{nullptr};
  std::string output_dir_,base_frame_,tcp_frame_,reference_link_,arm_group_name_,full_group_name_;
  int samples_{0},max_samples_{0},max_total_{0},random_seed_{0},progress_every_{0};
  double epsilon_{0},max_wall_s_{0};
  std::vector<std::string> canonical_;
  const std::vector<std::string> sample_variables_{"waist_yaw_joint","waist_pitch_joint","openarm_left_joint1","openarm_left_joint2","openarm_left_joint3","openarm_left_joint4","openarm_left_joint5","openarm_left_joint6","openarm_left_joint7"};
  std::array<unsigned,9> primes_{}; const std::array<double,5> ratios_{0.0,0.25,0.5,0.75,1.0};
  std::array<Eigen::Vector3d,5> references_{};
  std::map<std::pair<Configuration,std::size_t>,Counters> counters_; std::size_t total_{0}; std::ofstream states_;
};
}

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv); auto node=std::make_shared<rclcpp::Node>("lift_slice_fk_workspace");
  try { Runner(node).run(); }
  catch(const std::exception& e) { RCLCPP_FATAL(node->get_logger(),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
