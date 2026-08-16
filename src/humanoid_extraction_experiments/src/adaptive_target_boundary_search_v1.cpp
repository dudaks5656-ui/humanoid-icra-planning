#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <geometric_shapes/body_operations.h>

// Reuse only the audited scene schema and utility functions.  This executable
// creates a local RobotModel and PlanningScene; it never starts move_group/OMPL.
#define main preserved_reference_trajectory_generator_main_for_adaptive_boundary_v1
#include "reference_trajectory_generator.cpp"
#undef main

namespace adaptive_target_boundary_search_v1
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

struct Ray
{
  std::string id;
  double dx{};
  double dy{};
};

struct Target
{
  std::string ray;
  double distance{};
  double x{};
  double y{};
  double z{};
  bool object_wall_overlap{ false };
  bool gripper_envelope_infeasible{ false };
  std::string physical_pair;
};

struct Metrics
{
  bool bounds{ false };
  bool self_collision{ false };
  bool environment_collision{ false };
  double joint3_margin{ kNan };
  double joint5_margin{ kNan };
  double active_margin{ kNan };
  double environment_clearance{ kNan };
  double self_clearance{ kNan };
  double object_clearance{ kNan };
  std::string all_pairs;
  std::string environment_pairs;
  std::string self_pairs;
};

struct Candidate
{
  std::shared_ptr<moveit::core::RobotState> state;
  Metrics metrics;
  double yaw{};
  double pitch{};
  int seed{};
};

struct Search
{
  int attempts{};
  int raw_ik{};
  int collision_free_ik{};
  int coarse_attempts{};
  int fine_attempts{};
  bool found{ false };
  bool collision_seen{ false };
  Metrics collision_example;
  Candidate best;
};

struct Result
{
  Target target;
  double lift{};
  std::string mode;
  bool evaluated{ false };
  bool success{ false };
  int attempts{};
  int raw_ik{};
  int collision_free_ik{};
  int coarse_attempts{};
  int fine_attempts{};
  double yaw{ kNan };
  double pitch{ kNan };
  std::vector<double> arm;
  double joint3_margin{ kNan };
  double joint5_margin{ kNan };
  double active_margin{ kNan };
  double environment_clearance{ kNan };
  double self_clearance{ kNan };
  double descent_distance{};
  double ascent_distance{};
  double object_clearance{ kNan };
  std::string failure_stage;
  std::string failure_label;
  std::string failure_cause;
  std::string collision_pairs;
  double computation_ms{};
};

struct PointComparison
{
  Target target;
  double lift{};
  std::string phase;
  Result locked;
  Result posture;
  std::string classification;
};

struct Boundary
{
  std::string ray;
  double lift{};
  bool has_last_success{ false };
  Target last_success;
  Result last_success_result;
  bool has_first_failure{ false };
  Target first_failure;
  Result first_failure_result;
  bool has_recovery{ false };
  Target first_recovery;
  Result recovery_result;
  bool has_both_failure{ false };
  Target both_failure;
  bool has_gripper_infeasible{ false };
  Target first_gripper_infeasible;
  double best_margin_improvement{ kNan };
};

struct Envelope
{
  Eigen::Vector3d minimum{ Eigen::Vector3d::Constant(kInf) };
  Eigen::Vector3d maximum{ Eigen::Vector3d::Constant(-kInf) };
};

class Pilot
{
public:
  explicit Pilot(const rclcpp::Node::SharedPtr& node)
    : node_(node), scene_config_(loadSceneConfig(node_->get_parameter("scene_config").as_string()))
  {
    const YAML::Node cfg = YAML::LoadFile(node_->get_parameter("pilot_config").as_string());
    for (const auto& value : cfg["lift_candidates_m"])
      lifts_.push_back(value.as<double>());
    coarse_m_ = cfg["boundary_search"]["coarse_step_m"].as<double>();
    medium_m_ = cfg["boundary_search"]["medium_step_m"].as<double>();
    fine_m_ = cfg["boundary_search"]["fine_step_m"].as<double>();
    post_failure_points_ = cfg["boundary_search"]["post_failure_points"].as<int>();
    locked_multistart_ = cfg["locked"]["arm_ik_multistart"].as<int>();
    posture_multistart_ = cfg["yaw_pitch"]["arm_ik_multistart_per_posture"].as<int>();
    yaw_min_deg_ = cfg["yaw_pitch"]["yaw_min_deg"].as<double>();
    yaw_max_deg_ = cfg["yaw_pitch"]["yaw_max_deg"].as<double>();
    pitch_min_deg_ = cfg["yaw_pitch"]["pitch_min_deg"].as<double>();
    pitch_max_deg_ = cfg["yaw_pitch"]["pitch_max_deg"].as<double>();
    coarse_deg_ = cfg["yaw_pitch"]["coarse_step_deg"].as<double>();
    fine_deg_ = cfg["yaw_pitch"]["fine_step_deg"].as<double>();
    refine_half_deg_ = cfg["yaw_pitch"]["refinement_half_width_deg"].as<double>();
    sample_spacing_ = cfg["lift_motion"]["maximum_sample_spacing_m"].as<double>();
    clearance_required_ = cfg["lift_motion"]["object_bottom_clearance_above_box_top_m"].as<double>();
    low_margin_ = cfg["classification"]["locked_low_margin_rad"].as<double>();
    margin_improvement_ = cfg["classification"]["margin_improvement_rad"].as<double>();
    envelope_samples_ = cfg["gripper_envelope"]["finger_motion_samples"].as<int>();

    sequence_csv_ = node_->get_parameter("ray_state_sequence_csv").as_string();
    summary_csv_ = node_->get_parameter("boundary_summary_csv").as_string();
    envelope_csv_ = node_->get_parameter("gripper_envelope_csv").as_string();
    result_yaml_ = node_->get_parameter("result_yaml").as_string();
    audit_md_ = node_->get_parameter("audit_md").as_string();

    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    model_ = loader_->getModel();
    if (!model_)
      throw std::runtime_error("Robot model or SRDF could not be loaded");
    whole_body_ = requiredGroup("whole_body");
    left_arm_ = requiredGroup("left_arm");
    active_group_ = requiredGroup("left_arm_with_torso");
    if (!left_arm_->getSolverInstance())
      throw std::runtime_error("left_arm IK solver is unavailable");
    validateImmutableInputs();
    buildRays();
    initializeOutputs();
  }

  bool run()
  {
    createReferenceEnvelope();
    for (const auto& ray : rays_)
      for (double lift : lifts_)
      {
        RCLCPP_INFO(node_->get_logger(), "BOUNDARY ray=%s lift=%.2f", ray.id.c_str(), lift);
        boundaries_.push_back(runRay(ray, lift));
      }
    writeSummary();
    writeYaml();
    writeAudit();
    return boundaries_.size() == rays_.size() * lifts_.size();
  }

private:
  const moveit::core::JointModelGroup* requiredGroup(const std::string& name) const
  {
    const auto* group = model_->getJointModelGroup(name);
    if (!group)
      throw std::runtime_error("Required SRDF group missing: " + name);
    return group;
  }

  void validateImmutableInputs() const
  {
    if (!scene_config_.top_open_reference || std::abs(scene_config_.box_width - 0.600) > 1e-12 ||
        std::abs(scene_config_.box_depth - 0.400) > 1e-12 ||
        std::abs(scene_config_.box_height - 0.150) > 1e-12 ||
        std::abs(scene_config_.target_size[0] - 0.050) > 1e-12 ||
        std::abs(scene_config_.target_size[1] - 0.050) > 1e-12 ||
        std::abs(scene_config_.target_size[2] - 0.050) > 1e-12)
      throw std::runtime_error("Approved box/target geometry changed");
    const auto& lift = model_->getVariableBounds("lift_joint");
    if (!lift.position_bounded_ || std::abs(lift.min_position_) > 1e-12 ||
        std::abs(lift.max_position_ - 0.7) > 1e-12)
      throw std::runtime_error("Unexpected lift bounds; required [0.0, 0.7]");
  }

  void buildRays()
  {
    const double d = 1.0 / std::sqrt(2.0);
    rays_ = { { "Near", -1, 0 }, { "Far", 1, 0 }, { "Left", 0, 1 }, { "Right", 0, -1 },
              { "Near-Left", -d, d }, { "Near-Right", -d, -d },
              { "Far-Left", d, d }, { "Far-Right", d, -d } };
  }

  moveit_msgs::msg::CollisionObject boxObject(const std::string& id,
                                               const std::vector<double>& size,
                                               const std::vector<double>& p) const
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = scene_config_.frame_id;
    object.id = id;
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions.assign(size.begin(), size.end());
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = p[0]; pose.position.y = p[1]; pose.position.z = p[2];
    object.primitives.push_back(shape);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
  }

  void resetScene(const Target& target)
  {
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    const auto& c = scene_config_.box_center;
    const double w = scene_config_.box_width, d = scene_config_.box_depth;
    const double h = scene_config_.box_height, t = scene_config_.wall_thickness;
    const double floor = scene_config_.floor_thickness;
    const std::vector<moveit_msgs::msg::CollisionObject> objects{
      boxObject("box_bottom", { d + 2*t, w + 2*t, floor }, { c[0], c[1], c[2]-h/2-floor/2 }),
      boxObject("box_left_wall", { d, t, h }, { c[0], c[1]+w/2+t/2, c[2] }),
      boxObject("box_right_wall", { d, t, h }, { c[0], c[1]-w/2-t/2, c[2] }),
      boxObject("box_back_wall", { t, w+2*t, h }, { c[0]+d/2+t/2, c[1], c[2] }),
      boxObject("box_front_wall", { t, w+2*t, h }, { c[0]-d/2-t/2, c[1], c[2] }),
      boxObject(target_id_, scene_config_.target_size, { target.x, target.y, target.z }) };
    for (const auto& object : objects)
      if (!scene_->processCollisionObjectMsg(object))
        throw std::runtime_error("PlanningScene rejected " + object.id);
    attached_target_in_tcp_ = Eigen::Isometry3d::Identity();
  }

  geometry_msgs::msg::Pose graspPose(const Target& target) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = target.x;
    pose.position.y = target.y;
    pose.position.z = target.z - scene_config_.target_size[2]/2 +
                      scene_config_.grasp_height + scene_config_.tcp_to_grasp_center;
    tf2::Quaternion q;
    q.setRPY(scene_config_.eef_rpy[0], scene_config_.eef_rpy[1], scene_config_.eef_rpy[2]);
    q.normalize();
    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
    return pose;
  }

  Eigen::Isometry3d poseTransform(const geometry_msgs::msg::Pose& pose) const
  {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    t.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                    pose.orientation.y, pose.orientation.z).toRotationMatrix();
    return t;
  }

  moveit::core::RobotState seedState(double lift, double yaw, double pitch,
                                     std::uint64_t key, int seed) const
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setVariablePosition("lift_joint", lift);
    state.setVariablePosition("waist_yaw_joint", yaw);
    state.setVariablePosition("waist_pitch_joint", pitch);
    state.setVariablePosition("openarm_left_finger_joint1", scene_config_.left_finger);
    state.setVariablePosition("openarm_right_finger_joint1", scene_config_.right_finger);
    if (seed > 0)
    {
      std::mt19937_64 rng(202608150000ULL + key * 1000003ULL + static_cast<std::uint64_t>(seed));
      for (const auto& name : left_arm_->getVariableNames())
      {
        const auto& b = model_->getVariableBounds(name);
        std::uniform_real_distribution<double> distribution(b.min_position_, b.max_position_);
        state.setVariablePosition(name, distribution(rng));
      }
    }
    state.update();
    return state;
  }

  double margin(const moveit::core::RobotState& state, const std::string& name) const
  {
    const auto& b = model_->getVariableBounds(name);
    if (!b.position_bounded_)
      return kInf;
    const double q = state.getVariablePosition(name);
    return std::min(q-b.min_position_, b.max_position_-q);
  }

  Metrics evaluate(moveit::core::RobotState& state, bool attached = false) const
  {
    state.update();
    collision_detection::CollisionRequest req;
    req.contacts = true; req.max_contacts = 1000; req.max_contacts_per_pair = 50;
    collision_detection::CollisionResult self, full;
    scene_->checkSelfCollision(req, self, state);
    scene_->checkCollision(req, full, state);
    Metrics m;
    m.bounds = state.satisfiesBounds(whole_body_);
    m.self_collision = self.collision;
    std::set<std::pair<std::string,std::string>> all, env, self_pairs;
    for (const auto& entry : self.contacts)
    {
      all.insert(entry.first); self_pairs.insert(entry.first);
    }
    for (const auto& entry : full.contacts)
      for (const auto& contact : entry.second)
        if (contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT ||
            contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
        {
          m.environment_collision = true; all.insert(entry.first); env.insert(entry.first); break;
        }
    m.all_pairs = pairString(all); m.environment_pairs = pairString(env); m.self_pairs = pairString(self_pairs);
    const auto& acm = scene_->getAllowedCollisionMatrix();
    m.environment_clearance = scene_->getCollisionEnv()->distanceRobot(state, acm);
    m.self_clearance = scene_->getCollisionEnv()->distanceSelf(state, acm);
    m.active_margin = kInf;
    for (const auto& name : active_group_->getVariableNames())
      m.active_margin = std::min(m.active_margin, margin(state, name));
    m.joint3_margin = margin(state, "openarm_left_joint3");
    m.joint5_margin = margin(state, "openarm_left_joint5");
    if (attached)
    {
      const Eigen::Isometry3d object = state.getGlobalLinkTransform(tcp_link_) * attached_target_in_tcp_;
      const Eigen::Vector3d half(scene_config_.target_size[0]/2, scene_config_.target_size[1]/2,
                                 scene_config_.target_size[2]/2);
      double min_z = kInf;
      for (double x : {-half.x(), half.x()}) for (double y : {-half.y(), half.y()})
        for (double z : {-half.z(), half.z()}) min_z = std::min(min_z, (object*Eigen::Vector3d(x,y,z)).z());
      m.object_clearance = min_z - (scene_config_.box_center[2]+scene_config_.box_height/2);
    }
    return m;
  }

  bool valid(const Metrics& m) const
  {
    return m.bounds && !m.self_collision && !m.environment_collision;
  }

  bool better(const Candidate& a, const Candidate& b) const
  {
    if (a.metrics.active_margin != b.metrics.active_margin)
      return a.metrics.active_margin > b.metrics.active_margin;
    const double ac = std::min(a.metrics.environment_clearance, a.metrics.self_clearance);
    const double bc = std::min(b.metrics.environment_clearance, b.metrics.self_clearance);
    if (ac != bc) return ac > bc;
    const double ap = std::abs(a.yaw)+std::abs(a.pitch), bp = std::abs(b.yaw)+std::abs(b.pitch);
    if (ap != bp) return ap < bp;
    return a.seed < b.seed;
  }

  void searchPosture(const Target& target, double lift, double yaw_deg, double pitch_deg,
                     int count, std::uint64_t key, bool fine, Search& out)
  {
    const auto& yb = model_->getVariableBounds("waist_yaw_joint");
    const auto& pb = model_->getVariableBounds("waist_pitch_joint");
    const double yaw = std::clamp(yaw_deg*kPi/180.0, yb.min_position_, yb.max_position_);
    const double pitch = std::clamp(pitch_deg*kPi/180.0, pb.min_position_, pb.max_position_);
    const auto pose = graspPose(target);
    for (int seed=0; seed<count; ++seed)
    {
      ++out.attempts; fine ? ++out.fine_attempts : ++out.coarse_attempts;
      auto state = std::make_shared<moveit::core::RobotState>(seedState(lift,yaw,pitch,key,seed));
      if (!state->setFromIK(left_arm_, pose, tcp_link_, scene_config_.ik_timeout)) continue;
      ++out.raw_ik;
      Metrics m = evaluate(*state);
      if (!valid(m))
      {
        if (!out.collision_seen && (m.self_collision || m.environment_collision))
        { out.collision_seen=true; out.collision_example=m; }
        continue;
      }
      ++out.collision_free_ik;
      Candidate c{state,m,yaw,pitch,seed};
      if (!out.found || better(c,out.best)) { out.found=true; out.best=c; }
    }
  }

  Search search(const Target& target, double lift, bool posture, std::uint64_t key)
  {
    resetScene(target);
    Search out;
    if (!posture)
    {
      searchPosture(target,lift,0,0,locked_multistart_,key,false,out);
      return out;
    }
    const auto& pb = model_->getVariableBounds("waist_pitch_joint");
    const double actual_pitch_max = std::min(pitch_max_deg_, pb.max_position_*180.0/kPi);
    std::uint64_t posture_key=key*1009;
    for (double y=yaw_min_deg_; y<=yaw_max_deg_+1e-9; y+=coarse_deg_)
      for (double p=pitch_min_deg_; p<=actual_pitch_max+1e-9; p+=coarse_deg_)
        searchPosture(target,lift,y,p,posture_multistart_,++posture_key,false,out);
    // Test the exact URDF upper endpoint when the degree grid misses it.
    if (std::fmod(actual_pitch_max-pitch_min_deg_,coarse_deg_) > 1e-8)
      for (double y=yaw_min_deg_; y<=yaw_max_deg_+1e-9; y+=coarse_deg_)
        searchPosture(target,lift,y,actual_pitch_max,posture_multistart_,++posture_key,false,out);
    if (!out.found) return out;
    const double by=out.best.yaw*180/kPi, bp=out.best.pitch*180/kPi;
    for (double y=std::max(yaw_min_deg_,by-refine_half_deg_);
         y<=std::min(yaw_max_deg_,by+refine_half_deg_)+1e-9; y+=fine_deg_)
      for (double p=std::max(pitch_min_deg_,bp-refine_half_deg_);
           p<=std::min(actual_pitch_max,bp+refine_half_deg_)+1e-9; p+=fine_deg_)
        searchPosture(target,lift,y,p,posture_multistart_,++posture_key,true,out);
    return out;
  }

  void mergeMetric(Result& r, const Metrics& m)
  {
    r.joint3_margin=std::min(r.joint3_margin,m.joint3_margin);
    r.joint5_margin=std::min(r.joint5_margin,m.joint5_margin);
    r.active_margin=std::min(r.active_margin,m.active_margin);
    r.environment_clearance=std::min(r.environment_clearance,m.environment_clearance);
    r.self_clearance=std::min(r.self_clearance,m.self_clearance);
    if (std::isfinite(m.object_clearance)) r.object_clearance=m.object_clearance;
  }

  std::string collisionCause(const Metrics& m) const
  {
    const std::string pairs = m.all_pairs;
    if (m.self_collision && pairs.find("lift") != std::string::npos &&
        pairs.find("openarm_left") != std::string::npos) return "LIFT_STRUCTURE_ARM_SELF_COLLISION";
    if (m.environment_collision && pairs.find("box_") != std::string::npos &&
        pairs.find("openarm_left") != std::string::npos) return "ARM_WALL_COLLISION";
    if (m.self_collision) return "SELF_COLLISION";
    if (m.environment_collision) return "ENVIRONMENT_COLLISION";
    return "NONE";
  }

  void failFromMetrics(Result& r, const std::string& stage, const std::string& label,
                       const Metrics& m)
  {
    r.failure_stage=stage; r.failure_label=label; r.failure_cause=collisionCause(m);
    r.collision_pairs=m.all_pairs;
  }

  bool liftPath(Result& result, const moveit::core::RobotState& locked,
                const moveit::core::RobotState& start, double target_lift, bool attached,
                const std::string& stage, moveit::core::RobotState& finish)
  {
    const double q0=start.getVariablePosition("lift_joint");
    const int n=std::max(1,static_cast<int>(std::ceil(std::abs(target_lift-q0)/sample_spacing_)));
    for (int i=0;i<=n;++i)
    {
      moveit::core::RobotState state=start;
      state.setVariablePosition("lift_joint",q0+(target_lift-q0)*static_cast<double>(i)/n);
      state.update();
      Metrics m=evaluate(state,attached); mergeMetric(result,m);
      if (!valid(m))
      {
        failFromMetrics(result,stage,stage=="LIFT_VERTICAL_DESCENT" ?
          "LIFT_DESCENT_COLLISION_FAILURE":"LIFT_ASCENT_COLLISION_FAILURE",m);
        return false;
      }
      for (const auto& name:left_arm_->getVariableNames())
        if (std::abs(state.getVariablePosition(name)-locked.getVariablePosition(name))>1e-12)
        { result.failure_stage=stage; result.failure_label="ARM_POSTURE_LOCK_FAILURE"; return false; }
      if (std::abs(state.getVariablePosition("waist_yaw_joint")-locked.getVariablePosition("waist_yaw_joint"))>1e-12 ||
          std::abs(state.getVariablePosition("waist_pitch_joint")-locked.getVariablePosition("waist_pitch_joint"))>1e-12)
      { result.failure_stage=stage; result.failure_label="TORSO_POSTURE_LOCK_FAILURE"; return false; }
      finish=state;
    }
    return true;
  }

  void allowFingerTarget(bool allowed)
  {
    auto& acm=scene_->getAllowedCollisionMatrixNonConst();
    for (const auto& finger:finger_links_) acm.setEntry(finger,target_id_,allowed);
  }

  bool closeAndAttach(Result& result, const moveit::core::RobotState& initial,
                      moveit::core::RobotState& grasped)
  {
    allowFingerTarget(true);
    const double q0=initial.getVariablePosition("openarm_left_finger_joint1");
    for (int i=0;i<=10;++i)
    {
      moveit::core::RobotState state=initial;
      state.setVariablePosition("openarm_left_finger_joint1",q0+(scene_config_.q_contact-q0)*i/10.0);
      state.update(); Metrics m=evaluate(state); mergeMetric(result,m);
      if (!valid(m)) { failFromMetrics(result,"GRASP","GRASP_GEOMETRY_FAILURE",m); return false; }
      grasped=state;
    }
    scene_->setCurrentState(grasped); allowFingerTarget(false);
    Eigen::Isometry3d target_world=Eigen::Isometry3d::Identity();
    target_world.translation()=Eigen::Vector3d(result.target.x,result.target.y,result.target.z);
    attached_target_in_tcp_=grasped.getGlobalLinkTransform(tcp_link_).inverse()*target_world;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name=tcp_link_; attached.touch_links={finger_links_[0],finger_links_[1]};
    attached.object.header.frame_id=tcp_link_; attached.object.id=target_id_;
    shape_msgs::msg::SolidPrimitive shape; shape.type=shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions.assign(scene_config_.target_size.begin(), scene_config_.target_size.end());
    attached.object.primitives.push_back(shape);
    geometry_msgs::msg::Pose pose;
    pose.position.x=attached_target_in_tcp_.translation().x(); pose.position.y=attached_target_in_tcp_.translation().y();
    pose.position.z=attached_target_in_tcp_.translation().z();
    const Eigen::Quaterniond q(attached_target_in_tcp_.rotation());
    pose.orientation.x=q.x(); pose.orientation.y=q.y(); pose.orientation.z=q.z(); pose.orientation.w=q.w();
    attached.object.primitive_poses.push_back(pose); attached.object.operation=moveit_msgs::msg::CollisionObject::ADD;
    if (!scene_->processAttachedCollisionObjectMsg(attached))
      throw std::runtime_error("PlanningScene rejected target attachment");
    return true;
  }

  void validatePath(const Search& search_result, Result& result)
  {
    resetScene(result.target);
    moveit::core::RobotState locked=*search_result.best.state;
    locked.copyJointGroupPositions(left_arm_,result.arm);
    result.joint3_margin=result.joint5_margin=result.active_margin=kInf;
    result.environment_clearance=result.self_clearance=kInf;
    const double box_top=scene_config_.box_center[2]+scene_config_.box_height/2;
    const double object_bottom=result.target.z-scene_config_.target_size[2]/2;
    const double rise=box_top+clearance_required_-object_bottom;
    const double upper=result.lift-rise;
    result.descent_distance=rise; result.ascent_distance=rise;
    const auto& b=model_->getVariableBounds("lift_joint");
    if (upper<b.min_position_-1e-12 || upper>b.max_position_+1e-12)
    { result.failure_stage="LIFT_VERTICAL_DESCENT"; result.failure_label="LIFT_DESCENT_LIMIT_FAILURE"; return; }
    moveit::core::RobotState start=locked; start.setVariablePosition("lift_joint",upper); start.update();
    moveit::core::RobotState at_grasp=start;
    if (!liftPath(result,locked,start,result.lift,false,"LIFT_VERTICAL_DESCENT",at_grasp)) return;
    moveit::core::RobotState grasped=at_grasp;
    if (!closeAndAttach(result,at_grasp,grasped)) return;
    moveit::core::RobotState cleared=grasped;
    if (!liftPath(result,locked,grasped,upper,true,"LIFT_ACTUATED_CLEARANCE",cleared)) return;
    Metrics final=evaluate(cleared,true); mergeMetric(result,final);
    if (final.object_clearance+1e-9<clearance_required_)
    { result.failure_stage="LIFT_ACTUATED_CLEARANCE"; result.failure_label="ATTACHED_OBJECT_CLEARANCE_FAILURE"; return; }
    result.success=true; result.failure_label="LIFT_ACTUATED_EXTRACTION_SUCCESS"; result.failure_cause="NONE";
  }

  Result runMode(const Target& target, double lift, bool posture, std::uint64_t key)
  {
    const auto begin=std::chrono::steady_clock::now();
    Result r; r.target=target; r.lift=lift; r.mode=posture?"YAW_PITCH":"LOCKED"; r.evaluated=true;
    if (target.object_wall_overlap || target.gripper_envelope_infeasible)
    {
      r.failure_stage="PHYSICAL_FEASIBILITY";
      r.failure_label=target.object_wall_overlap?"OBJECT_WALL_OVERLAP":"GRIPPER_ENVELOPE_INFEASIBLE";
      r.failure_cause=r.failure_label; r.collision_pairs=target.physical_pair;
    }
    else
    {
      Search s=search(target,lift,posture,key);
      r.attempts=s.attempts; r.raw_ik=s.raw_ik; r.collision_free_ik=s.collision_free_ik;
      r.coarse_attempts=s.coarse_attempts; r.fine_attempts=s.fine_attempts;
      if (!s.found)
      {
        r.failure_stage="GRASP_CONFIGURATION_SEARCH";
        r.failure_label=s.raw_ik==0?"GRASP_CONFIGURATION_IK_FAILURE":"GRASP_CONFIGURATION_COLLISION_FAILURE";
        r.failure_cause=s.raw_ik==0?"PURE_IK_ABSENCE":collisionCause(s.collision_example);
        if (s.collision_seen) r.collision_pairs=s.collision_example.all_pairs;
      }
      else
      {
        r.yaw=s.best.yaw; r.pitch=s.best.pitch;
        s.best.state->copyJointGroupPositions(left_arm_,r.arm);
        validatePath(s,r);
      }
    }
    r.computation_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    return r;
  }

  double maximumDistance(const Ray& ray) const
  {
    const double cx=scene_config_.box_center[0], cy=scene_config_.box_center[1];
    const double xmin=cx-scene_config_.box_depth/2+scene_config_.target_size[0]/2;
    const double xmax=cx+scene_config_.box_depth/2-scene_config_.target_size[0]/2;
    const double ymin=cy-scene_config_.box_width/2+scene_config_.target_size[1]/2;
    const double ymax=cy+scene_config_.box_width/2-scene_config_.target_size[1]/2;
    double maximum=kInf;
    if (ray.dx>0) maximum=std::min(maximum,(xmax-cx)/ray.dx);
    if (ray.dx<0) maximum=std::min(maximum,(xmin-cx)/ray.dx);
    if (ray.dy>0) maximum=std::min(maximum,(ymax-cy)/ray.dy);
    if (ray.dy<0) maximum=std::min(maximum,(ymin-cy)/ray.dy);
    return maximum;
  }

  bool overlaps(const Eigen::Vector3d& amin, const Eigen::Vector3d& amax,
                const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax) const
  {
    return (amin.array()<=bmax.array()).all() && (amax.array()>=bmin.array()).all();
  }

  Target makeTarget(const Ray& ray, double distance) const
  {
    Target t;
    t.ray=ray.id;
    t.distance=distance;
    t.x=scene_config_.box_center[0]+ray.dx*distance;
    t.y=scene_config_.box_center[1]+ray.dy*distance;
    t.z=scene_config_.target_position[2];
    const double xmin=scene_config_.box_center[0]-scene_config_.box_depth/2;
    const double xmax=scene_config_.box_center[0]+scene_config_.box_depth/2;
    const double ymin=scene_config_.box_center[1]-scene_config_.box_width/2;
    const double ymax=scene_config_.box_center[1]+scene_config_.box_width/2;
    const double hx=scene_config_.target_size[0]/2, hy=scene_config_.target_size[1]/2;
    t.object_wall_overlap=t.x-hx<xmin-1e-12 || t.x+hx>xmax+1e-12 ||
                          t.y-hy<ymin-1e-12 || t.y+hy>ymax+1e-12;
    if (t.object_wall_overlap) t.physical_pair="target_object--box_inner_wall";
    if (envelope_ready_)
    {
      const Eigen::Vector3d shift(t.x-scene_config_.box_center[0],t.y-scene_config_.box_center[1],0);
      const Eigen::Vector3d amin=envelope_.minimum+shift, amax=envelope_.maximum+shift;
      const auto& c=scene_config_.box_center; const double w=scene_config_.box_width,d=scene_config_.box_depth;
      const double h=scene_config_.box_height,th=scene_config_.wall_thickness;
      const std::array<std::tuple<std::string,Eigen::Vector3d,Eigen::Vector3d>,4> walls{{
        {"box_left_wall",{c[0]-d/2,c[1]+w/2,c[2]-h/2},{c[0]+d/2,c[1]+w/2+th,c[2]+h/2}},
        {"box_right_wall",{c[0]-d/2,c[1]-w/2-th,c[2]-h/2},{c[0]+d/2,c[1]-w/2,c[2]+h/2}},
        {"box_back_wall",{c[0]+d/2,c[1]-w/2-th,c[2]-h/2},{c[0]+d/2+th,c[1]+w/2+th,c[2]+h/2}},
        {"box_front_wall",{c[0]-d/2-th,c[1]-w/2-th,c[2]-h/2},{c[0]-d/2,c[1]+w/2+th,c[2]+h/2}} }};
      for (const auto& wall:walls)
        if (overlaps(amin,amax,std::get<1>(wall),std::get<2>(wall)))
        { t.gripper_envelope_infeasible=true; if (!t.physical_pair.empty()) t.physical_pair+=';';
          t.physical_pair+="gripper_swept_envelope--"+std::get<0>(wall); }
    }
    return t;
  }

  void createReferenceEnvelope()
  {
    const Ray center{"CENTER",0,0}; Target target=makeTarget(center,0); resetScene(target);
    Search reference; searchPosture(target,0.35,0,0,std::max(locked_multistart_,100),1,false,reference);
    if (!reference.found) throw std::runtime_error("Could not obtain center reference state for gripper envelope");
    std::ofstream out(envelope_csv_,std::ios::trunc);
    out<<"sample,finger_joint,link,shape_index,min_x,min_y,min_z,max_x,max_y,max_z\n";
    const std::array<std::string,3> links{"openarm_left_link7",finger_links_[0],finger_links_[1]};
    for (int sample=0;sample<envelope_samples_;++sample)
    {
      const double ratio=envelope_samples_==1?0.0:static_cast<double>(sample)/(envelope_samples_-1);
      moveit::core::RobotState state=*reference.best.state;
      const double q=scene_config_.left_finger+(scene_config_.q_contact-scene_config_.left_finger)*ratio;
      state.setVariablePosition("openarm_left_finger_joint1",q); state.update();
      for (const auto& link_name:links)
      {
        const auto* link=model_->getLinkModel(link_name);
        if (!link) throw std::runtime_error("Missing gripper link "+link_name);
        const auto& shapes=link->getShapes(); const auto& origins=link->getCollisionOriginTransforms();
        for (std::size_t i=0;i<shapes.size();++i)
        {
          std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shapes[i].get()));
          body->setPose(state.getGlobalLinkTransform(link)*origins[i]);
          bodies::AABB box; body->computeBoundingBox(box);
          envelope_.minimum=envelope_.minimum.cwiseMin(box.min());
          envelope_.maximum=envelope_.maximum.cwiseMax(box.max());
          out<<std::setprecision(15)<<sample<<','<<q<<','<<link_name<<','<<i<<','
             <<box.min().x()<<','<<box.min().y()<<','<<box.min().z()<<','
             <<box.max().x()<<','<<box.max().y()<<','<<box.max().z()<<'\n';
        }
      }
    }
    envelope_ready_=true;
  }

  long long cacheKey(double distance) const { return std::llround(distance*1000000.0); }

  Result& lockedAt(const Ray& ray, double lift, double distance, const std::string& phase)
  {
    const auto key=std::make_tuple(ray.id,static_cast<int>(std::llround(lift*100)),cacheKey(distance));
    auto it=locked_cache_.find(key);
    if (it==locked_cache_.end())
    {
      Target target=makeTarget(ray,distance);
      const std::uint64_t seed_key=std::hash<std::string>{}(ray.id)^static_cast<std::uint64_t>(cacheKey(distance)+lift*1000);
      Result result=runMode(target,lift,false,seed_key);
      it=locked_cache_.emplace(key,std::move(result)).first;
      phases_[key]=phase;
      RCLCPP_INFO(node_->get_logger(),"LOCKED ray=%s lift=%.2f d=%.3f success=%d label=%s",
                  ray.id.c_str(),lift,distance,it->second.success,it->second.failure_label.c_str());
    }
    return it->second;
  }

  Result postureAt(const Ray& ray, double lift, double distance)
  {
    Target target=makeTarget(ray,distance);
    const std::uint64_t key=std::hash<std::string>{}(ray.id)^static_cast<std::uint64_t>(cacheKey(distance)+lift*10000+7);
    Result r=runMode(target,lift,true,key);
    RCLCPP_INFO(node_->get_logger(),"YAW_PITCH ray=%s lift=%.2f d=%.3f success=%d label=%s",
                ray.id.c_str(),lift,distance,r.success,r.failure_label.c_str());
    return r;
  }

  std::string classify(const Result& locked, const Result& posture) const
  {
    if (locked.target.gripper_envelope_infeasible) return "GRIPPER_ENVELOPE_INFEASIBLE";
    if (locked.failure_cause=="ARM_WALL_COLLISION" && posture.failure_cause=="ARM_WALL_COLLISION")
      return "ARM_BOX_COLLISION_INFEASIBLE";
    if (locked.failure_cause.find("SELF_COLLISION")!=std::string::npos &&
        posture.failure_cause.find("SELF_COLLISION")!=std::string::npos) return "SELF_COLLISION_INFEASIBLE";
    if (!locked.success && posture.success) return "YAW_PITCH_FEASIBILITY_RECOVERY";
    if (!locked.success && !posture.success) return "BOTH_KINEMATICALLY_INFEASIBLE";
    if (locked.success && posture.success)
    {
      if (posture.active_margin-locked.active_margin>=margin_improvement_) return "YAW_PITCH_MARGIN_IMPROVEMENT";
      if (locked.active_margin<low_margin_) return "LOCKED_LOW_MARGIN";
      return "BOTH_FEASIBLE";
    }
    return locked.success?"BOTH_FEASIBLE":"BOTH_KINEMATICALLY_INFEASIBLE";
  }

  void appendSequence(const PointComparison& p)
  {
    sequence_.push_back(p);
    std::ofstream out(sequence_csv_,std::ios::app);
    out<<std::setprecision(15)<<p.target.ray<<','<<p.lift<<','<<p.phase<<','<<p.target.distance<<','
       <<p.target.x<<','<<p.target.y<<','<<p.target.z<<','<<p.target.distance<<','
       <<p.target.object_wall_overlap<<','<<p.target.gripper_envelope_infeasible<<','
       <<csvEscape(p.target.physical_pair)<<','<<p.locked.success<<','<<p.locked.attempts<<','
       <<p.locked.raw_ik<<','<<p.locked.collision_free_ik<<','<<p.locked.yaw<<','<<p.locked.pitch;
    for (std::size_t i=0;i<left_arm_->getVariableNames().size();++i)
      out<<','<<(i<p.locked.arm.size()?p.locked.arm[i]:kNan);
    out<<','<<p.locked.joint3_margin<<','<<p.locked.joint5_margin<<','<<p.locked.active_margin<<','
       <<p.locked.environment_clearance<<','<<p.locked.self_clearance<<','<<p.locked.descent_distance<<','
       <<p.locked.ascent_distance<<','<<p.locked.object_clearance<<','<<csvEscape(p.locked.failure_stage)<<','
       <<csvEscape(p.locked.failure_label)<<','<<csvEscape(p.locked.failure_cause)<<','
       <<csvEscape(p.locked.collision_pairs)<<','<<p.locked.computation_ms<<','<<p.posture.evaluated<<','
       <<p.posture.success<<','<<p.posture.attempts<<','<<p.posture.raw_ik<<','
       <<p.posture.collision_free_ik<<','<<p.posture.yaw<<','<<p.posture.pitch;
    for (std::size_t i=0;i<left_arm_->getVariableNames().size();++i)
      out<<','<<(i<p.posture.arm.size()?p.posture.arm[i]:kNan);
    out<<','<<p.posture.joint3_margin<<','<<p.posture.joint5_margin<<','<<p.posture.active_margin<<','
       <<p.posture.environment_clearance<<','<<p.posture.self_clearance<<','<<p.posture.descent_distance<<','
       <<p.posture.ascent_distance<<','<<p.posture.object_clearance<<','<<csvEscape(p.posture.failure_stage)<<','
       <<csvEscape(p.posture.failure_label)<<','<<csvEscape(p.posture.failure_cause)<<','
       <<csvEscape(p.posture.collision_pairs)<<','<<p.posture.computation_ms<<','<<p.classification<<'\n';
  }

  Boundary runRay(const Ray& ray, double lift)
  {
    Boundary b; b.ray=ray.id; b.lift=lift;
    const double maxd=maximumDistance(ray);
    std::vector<double> coarse;
    double last_success=-1, first_failure=-1;
    for (double d=0;d<=maxd+1e-9;d+=coarse_m_)
    {
      d=std::min(d,maxd); Result& r=lockedAt(ray,lift,d,"COARSE_20MM"); coarse.push_back(d);
      if (r.success) last_success=d;
      else { first_failure=d; break; }
      if (maxd-d<1e-9) break;
      if (d+coarse_m_>maxd) d=maxd-coarse_m_;
    }
    if (first_failure<0 && last_success>=0 && last_success<maxd-1e-9)
    { first_failure=maxd; lockedAt(ray,lift,first_failure,"COARSE_EDGE"); }
    if (last_success>=0 && first_failure>=0)
    {
      for (double d=last_success+medium_m_;d<first_failure-1e-9;d+=medium_m_)
      { Result& r=lockedAt(ray,lift,d,"REFINE_5MM"); if (r.success) last_success=d; else { first_failure=d; break; } }
      for (double d=last_success+fine_m_;d<first_failure-1e-9;d+=fine_m_)
      { Result& r=lockedAt(ray,lift,d,"REFINE_1MM"); if (r.success) last_success=d; else { first_failure=d; break; } }
    }
    if (last_success>=0)
    { b.has_last_success=true; b.last_success=makeTarget(ray,last_success); b.last_success_result=lockedAt(ray,lift,last_success,"BOUNDARY"); }
    if (first_failure>=0)
    { b.has_first_failure=true; b.first_failure=makeTarget(ray,first_failure); b.first_failure_result=lockedAt(ray,lift,first_failure,"BOUNDARY"); }

    std::set<long long> targeted;
    if (last_success>=0) targeted.insert(cacheKey(last_success));
    if (first_failure>=0)
      for (int i=0;i<=post_failure_points_;++i)
      {
        const double d=std::min(maxd,first_failure+i*medium_m_);
        targeted.insert(cacheKey(d));
        // Materialize the requested first-failure and 1--2 points beyond it.
        // The result cache avoids duplicate IK work when a point was already
        // visited by the coarse or boundary-refinement passes.
        lockedAt(ray,lift,d,i==0?"BOUNDARY":"POST_FAILURE_5MM");
      }
    // Add the successful point with the largest observed active-margin drop.
    double sharp_d=-1, sharp_drop=0, previous_margin=kNan;
    for (const auto& entry:locked_cache_)
      if (std::get<0>(entry.first)==ray.id && std::get<1>(entry.first)==static_cast<int>(std::llround(lift*100)) && entry.second.success)
      {
        const double m=entry.second.active_margin;
        if (std::isfinite(previous_margin) && previous_margin-m>sharp_drop) { sharp_drop=previous_margin-m; sharp_d=entry.second.target.distance; }
        previous_margin=m;
      }
    if (sharp_d>=0) targeted.insert(cacheKey(sharp_d));

    // Emit every LOCKED point in distance order; execute Y/P only at targeted points.
    std::vector<Result> rows;
    for (const auto& entry:locked_cache_)
      if (std::get<0>(entry.first)==ray.id && std::get<1>(entry.first)==static_cast<int>(std::llround(lift*100))) rows.push_back(entry.second);
    std::sort(rows.begin(),rows.end(),[](const Result&a,const Result&c){return a.target.distance<c.target.distance;});
    for (const auto& locked:rows)
    {
      Result posture; posture.target=locked.target; posture.lift=lift; posture.mode="YAW_PITCH";
      if (targeted.count(cacheKey(locked.target.distance))) posture=postureAt(ray,lift,locked.target.distance);
      const std::string classification=posture.evaluated?classify(locked,posture):
        (locked.target.gripper_envelope_infeasible?"GRIPPER_ENVELOPE_INFEASIBLE":
         (locked.success?(locked.active_margin<low_margin_?"LOCKED_LOW_MARGIN":"BOTH_FEASIBLE"):
          "BOTH_KINEMATICALLY_INFEASIBLE"));
      appendSequence({locked.target,lift,phases_[std::make_tuple(ray.id,static_cast<int>(std::llround(lift*100)),cacheKey(locked.target.distance))],locked,posture,classification});
      if (locked.target.gripper_envelope_infeasible && !b.has_gripper_infeasible)
      { b.has_gripper_infeasible=true; b.first_gripper_infeasible=locked.target; }
      if (posture.evaluated && !locked.success && posture.success && !b.has_recovery)
      { b.has_recovery=true; b.first_recovery=locked.target; b.recovery_result=posture; }
      if (posture.evaluated && !locked.success && !posture.success && !b.has_both_failure &&
          !locked.target.gripper_envelope_infeasible)
      { b.has_both_failure=true; b.both_failure=locked.target; }
      if (posture.evaluated && locked.success && posture.success)
      { const double improvement=posture.active_margin-locked.active_margin;
        if (!std::isfinite(b.best_margin_improvement) || improvement>b.best_margin_improvement) b.best_margin_improvement=improvement; }
    }
    return b;
  }

  void initializeOutputs() const
  {
    std::ofstream out(sequence_csv_,std::ios::trunc);
    out<<"ray,lift,phase,distance_m,target_x,target_y,target_z,center_distance_m,object_wall_overlap,"
          "gripper_envelope_infeasible,physical_pair,locked_success,locked_attempts,locked_raw_ik,"
          "locked_collision_free_ik,locked_yaw_rad,locked_pitch_rad";
    for (const auto& n:left_arm_->getVariableNames()) out<<",locked_"<<n;
    out<<",locked_joint3_margin,locked_joint5_margin,locked_active_margin,locked_environment_clearance,"
          "locked_self_clearance,locked_descent_m,locked_ascent_m,locked_object_clearance,locked_failure_stage,"
          "locked_failure_label,locked_failure_cause,locked_collision_pairs,locked_computation_ms,"
          "yaw_pitch_evaluated,yaw_pitch_success,yaw_pitch_attempts,yaw_pitch_raw_ik,yaw_pitch_collision_free_ik,"
          "selected_yaw_rad,selected_pitch_rad";
    for (const auto& n:left_arm_->getVariableNames()) out<<",yaw_pitch_"<<n;
    out<<",yaw_pitch_joint3_margin,yaw_pitch_joint5_margin,yaw_pitch_active_margin,"
          "yaw_pitch_environment_clearance,yaw_pitch_self_clearance,yaw_pitch_descent_m,yaw_pitch_ascent_m,"
          "yaw_pitch_object_clearance,yaw_pitch_failure_stage,yaw_pitch_failure_label,yaw_pitch_failure_cause,"
          "yaw_pitch_collision_pairs,yaw_pitch_computation_ms,classification\n";
  }

  static std::string targetYaml(const Target& t)
  {
    std::ostringstream s; s<<std::setprecision(15)<<"["<<t.x<<", "<<t.y<<", "<<t.z<<"]"; return s.str();
  }

  void writeSummary()
  {
    std::ofstream out(summary_csv_,std::ios::trunc);
    out<<"ray,lift,last_locked_success_distance,last_locked_success_xyz,first_locked_failure_distance,"
          "first_locked_failure_xyz,first_yaw_pitch_recovery_distance,first_yaw_pitch_recovery_xyz,"
          "both_fail_start_distance,both_fail_start_xyz,first_gripper_infeasible_distance,"
          "first_gripper_infeasible_xyz,recovery_yaw_rad,recovery_pitch_rad,recovery_raw_ik,"
          "recovery_collision_free_ik,recovery_joint3_margin,recovery_joint5_margin,recovery_active_margin,"
          "recovery_environment_clearance,recovery_self_clearance,best_margin_improvement\n";
    for (const auto& b:boundaries_)
      out<<std::setprecision(15)<<b.ray<<','<<b.lift<<','<<(b.has_last_success?b.last_success.distance:kNan)<<','
         <<csvEscape(b.has_last_success?targetYaml(b.last_success):"")<<','
         <<(b.has_first_failure?b.first_failure.distance:kNan)<<','<<csvEscape(b.has_first_failure?targetYaml(b.first_failure):"")<<','
         <<(b.has_recovery?b.first_recovery.distance:kNan)<<','<<csvEscape(b.has_recovery?targetYaml(b.first_recovery):"")<<','
         <<(b.has_both_failure?b.both_failure.distance:kNan)<<','<<csvEscape(b.has_both_failure?targetYaml(b.both_failure):"")<<','
         <<(b.has_gripper_infeasible?b.first_gripper_infeasible.distance:kNan)<<','
         <<csvEscape(b.has_gripper_infeasible?targetYaml(b.first_gripper_infeasible):"")<<','
         <<(b.has_recovery?b.recovery_result.yaw:kNan)<<','<<(b.has_recovery?b.recovery_result.pitch:kNan)<<','
         <<(b.has_recovery?b.recovery_result.raw_ik:0)<<','<<(b.has_recovery?b.recovery_result.collision_free_ik:0)<<','
         <<(b.has_recovery?b.recovery_result.joint3_margin:kNan)<<','<<(b.has_recovery?b.recovery_result.joint5_margin:kNan)<<','
         <<(b.has_recovery?b.recovery_result.active_margin:kNan)<<','
         <<(b.has_recovery?b.recovery_result.environment_clearance:kNan)<<','
         <<(b.has_recovery?b.recovery_result.self_clearance:kNan)<<','<<b.best_margin_improvement<<'\n';
  }

  void writeYaml() const
  {
    std::ofstream out(result_yaml_,std::ios::trunc);
    out<<"protocol: ADAPTIVE_TARGET_BOUNDARY_SEARCH_V1\nplanning_only: true\nmove_group_started: false\n"
          "ompl_started: false\ntrajectory_execution_performed: false\nrviz_started: false\n"
          "gripper_envelope_world_aabb:\n  minimum: ["<<envelope_.minimum.x()<<", "<<envelope_.minimum.y()<<", "<<envelope_.minimum.z()<<"]\n"
          "  maximum: ["<<envelope_.maximum.x()<<", "<<envelope_.maximum.y()<<", "<<envelope_.maximum.z()<<"]\n"
          "boundaries:\n";
    for (const auto& b:boundaries_)
      out<<"  - ray: "<<b.ray<<"\n    lift: "<<b.lift<<"\n    last_locked_success_m: "
         <<(b.has_last_success?b.last_success.distance:kNan)<<"\n    first_locked_failure_m: "
         <<(b.has_first_failure?b.first_failure.distance:kNan)<<"\n    yaw_pitch_recovery: "
         <<(b.has_recovery?"true":"false")<<"\n    first_gripper_infeasible_m: "
         <<(b.has_gripper_infeasible?b.first_gripper_infeasible.distance:kNan)<<"\n";
  }

  void writeAudit() const
  {
    int recovery=0,gripper=0; for (const auto& b:boundaries_) { recovery+=b.has_recovery; gripper+=b.has_gripper_infeasible; }
    std::ofstream out(audit_md_,std::ios::trunc);
    out<<"# Adaptive target boundary search v1\n\nGenerated: "<<timestampNow()<<"\n\n"
          "- Scope: 8 center rays x 2 fixed Lift values; adaptive 20/5/1 mm LOCKED boundary pilot.\n"
          "- Yaw/Pitch was evaluated only at the last LOCKED success, first failure, up to two post-failure points, and a sharp-margin point.\n"
          "- Qualified YAW_PITCH_FEASIBILITY_RECOVERY boundaries: "<<recovery<<".\n"
          "- Rays containing a GRIPPER_ENVELOPE_INFEASIBLE sample: "<<gripper<<".\n"
          "- The swept envelope uses actual collision shapes and collision-origin transforms of openarm_left_link7 and both finger links across open-to-q_contact motion.\n"
          "- Arm and selected torso posture remained fixed during <=1 mm Lift-only descent/ascent; attached-object clearance target was 20 mm.\n"
          "- No move_group, OMPL, controller, ros2_control, hardware, trajectory execution, or RViz was started.\n";
  }

  rclcpp::Node::SharedPtr node_;
  SceneConfig scene_config_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* whole_body_{nullptr};
  const moveit::core::JointModelGroup* left_arm_{nullptr};
  const moveit::core::JointModelGroup* active_group_{nullptr};
  planning_scene::PlanningScenePtr scene_;
  std::vector<Ray> rays_; std::vector<double> lifts_; std::vector<Boundary> boundaries_;
  std::vector<PointComparison> sequence_;
  std::map<std::tuple<std::string,int,long long>,Result> locked_cache_;
  std::map<std::tuple<std::string,int,long long>,std::string> phases_;
  Envelope envelope_; bool envelope_ready_{false};
  double coarse_m_{},medium_m_{},fine_m_{}; int post_failure_points_{};
  int locked_multistart_{},posture_multistart_{},envelope_samples_{};
  double yaw_min_deg_{},yaw_max_deg_{},pitch_min_deg_{},pitch_max_deg_{},coarse_deg_{},fine_deg_{},refine_half_deg_{};
  double sample_spacing_{},clearance_required_{},low_margin_{},margin_improvement_{};
  Eigen::Isometry3d attached_target_in_tcp_{Eigen::Isometry3d::Identity()};
  const std::string tcp_link_{"openarm_left_hand_tcp"}; const std::string target_id_{"target_object"};
  const std::array<std::string,2> finger_links_{"openarm_left_left_finger","openarm_left_right_finger"};
  std::string sequence_csv_,summary_csv_,envelope_csv_,result_yaml_,audit_md_;
};
}  // namespace adaptive_target_boundary_search_v1

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options; options.automatically_declare_parameters_from_overrides(true);
  auto node=std::make_shared<rclcpp::Node>("adaptive_target_boundary_search_v1",options);
  rclcpp::executors::MultiThreadedExecutor executor; executor.add_node(node);
  std::thread spin_thread([&executor](){executor.spin();});
  int code=1;
  std::unique_ptr<adaptive_target_boundary_search_v1::Pilot> pilot;
  try
  {
    pilot=std::make_unique<adaptive_target_boundary_search_v1::Pilot>(node);
    code=pilot->run()?0:2;
  }
  catch (const std::exception& e) { RCLCPP_ERROR(node->get_logger(),"Adaptive boundary pilot failed: %s",e.what()); }
  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  pilot.reset();
  node.reset();
  rclcpp::shutdown();
  return code;
}
