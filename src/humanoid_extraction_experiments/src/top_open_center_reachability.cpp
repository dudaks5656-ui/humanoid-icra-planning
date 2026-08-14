#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
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
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr char kSceneId[] = "TOP_OPEN_BOX_600X400X150_GEOMETRY_VALIDATION";

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

std::string csvEscape(const std::string& input)
{
  std::string output = "\"";
  for (const char character : input)
    output += character == '"' ? "\"\"" : std::string(1, character);
  return output + "\"";
}

std::string pairString(const std::set<std::pair<std::string, std::string>>& pairs)
{
  std::ostringstream output;
  bool first = true;
  for (const auto& pair : pairs)
  {
    if (!first)
      output << ';';
    output << pair.first << "<->" << pair.second;
    first = false;
  }
  return output.str();
}

struct AuditConfig
{
  std::string frame_id;
  double width{};
  double depth{};
  double height{};
  double center_y{};
  double floor_z{};
  double front_x{};
  double wall_thickness{};
  double floor_thickness{};
  std::array<double, 3> target_size{};
  std::array<double, 3> target_center{};
  double q_open{};
  double tcp_to_grasp_center{};
  double approach_clearance{};
  double pre_grasp_clearance{};
  int seed_count{};
  std::uint32_t seed_base{};
  double ik_timeout{};

  double backX() const { return front_x + depth; }
  double minY() const { return center_y - width / 2.0; }
  double maxY() const { return center_y + width / 2.0; }
  double topZ() const { return floor_z + height; }
  double centerX() const { return front_x + depth / 2.0; }
  double centerZ() const { return floor_z + height / 2.0; }
};

AuditConfig loadConfig(const std::string& path)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (root["scene_id"].as<std::string>() != kSceneId)
    throw std::runtime_error("Unexpected top-open scene ID");
  AuditConfig config;
  config.frame_id = root["frame_id"].as<std::string>();
  const YAML::Node box = root["box"];
  config.width = box["internal_width_y"].as<double>();
  config.depth = box["internal_depth_x"].as<double>();
  config.height = box["internal_height_z"].as<double>();
  config.center_y = box["box_center_y"].as<double>();
  config.floor_z = box["box_floor_z"].as<double>();
  config.front_x = box["front_inner_plane_x"].as<double>();
  config.wall_thickness = box["wall_thickness"].as<double>();
  config.floor_thickness = box["floor_thickness"].as<double>();
  const YAML::Node target = root["target"];
  for (std::size_t index = 0; index < 3; ++index)
  {
    config.target_size[index] = target["size_xyz"][index].as<double>();
    config.target_center[index] = target["center_xyz"][index].as<double>();
  }
  config.q_open = root["display"]["initial_left_finger"].as<double>();
  const YAML::Node audit = root["reachability_audit"];
  config.tcp_to_grasp_center = audit["tcp_to_grasp_center"].as<double>();
  config.approach_clearance = audit["approach_clearance_above_rim"].as<double>();
  config.pre_grasp_clearance = audit["pre_grasp_clearance_above_object"].as<double>();
  config.seed_count = audit["ik_seed_count"].as<int>();
  config.seed_base = audit["ik_seed_base"].as<std::uint32_t>();
  config.ik_timeout = audit["ik_timeout_s"].as<double>();
  return config;
}

struct Candidate
{
  std::string mode;
  std::string id;
  double lift{};
  double yaw{};
  double pitch{};
  bool within_limits{ false };
};

struct CollisionAudit
{
  bool bounds_valid{ false };
  bool self_collision{ false };
  bool robot_box_collision{ false };
  bool gripper_box_collision{ false };
  bool robot_target_collision{ false };
  bool target_box_collision{ false };
  double joint_margin{ std::numeric_limits<double>::quiet_NaN() };
  double self_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double environment_clearance{ std::numeric_limits<double>::quiet_NaN() };
  double minimum_clearance{ std::numeric_limits<double>::quiet_NaN() };
  std::set<std::pair<std::string, std::string>> pairs;

  bool collisionFree() const
  {
    return bounds_valid && !self_collision && !robot_box_collision && !robot_target_collision &&
           !target_box_collision;
  }
};

struct StageAggregate
{
  int seeds_tested{};
  int ik_success_count{};
  int collision_free_count{};
  double best_clearance{ -std::numeric_limits<double>::infinity() };
  double best_joint_margin{ -std::numeric_limits<double>::infinity() };
  std::set<std::pair<std::string, std::string>> failure_pairs;
};

struct CandidateAggregate
{
  Candidate candidate;
  std::map<std::string, StageAggregate> stages;
  bool allStagesPossible() const
  {
    for (const char* stage : { "APPROACH", "PRE_GRASP", "GRASP" })
    {
      const auto found = stages.find(stage);
      if (found == stages.end() || found->second.collision_free_count == 0)
        return false;
    }
    return true;
  }
  double taskClearance() const
  {
    double value = std::numeric_limits<double>::infinity();
    for (const char* stage : { "APPROACH", "PRE_GRASP", "GRASP" })
      value = std::min(value, stages.at(stage).best_clearance);
    return value;
  }
};

moveit_msgs::msg::CollisionObject boxObject(const AuditConfig& config, const std::string& id,
                                            const std::array<double, 3>& size,
                                            const std::array<double, 3>& center)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = config.frame_id;
  object.id = id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.assign(size.begin(), size.end());
  geometry_msgs::msg::Pose pose;
  pose.position.x = center[0];
  pose.position.y = center[1];
  pose.position.z = center[2];
  pose.orientation.w = 1.0;
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

std::vector<moveit_msgs::msg::CollisionObject> sceneObjects(const AuditConfig& config)
{
  const double outside_width = config.width + 2.0 * config.wall_thickness;
  const double outside_depth = config.depth + 2.0 * config.wall_thickness;
  return {
    boxObject(config, "box_floor", { outside_depth, outside_width, config.floor_thickness },
              { config.centerX(), config.center_y, config.floor_z - config.floor_thickness / 2.0 }),
    boxObject(config, "box_front_wall", { config.wall_thickness, outside_width, config.height },
              { config.front_x - config.wall_thickness / 2.0, config.center_y, config.centerZ() }),
    boxObject(config, "box_back_wall", { config.wall_thickness, outside_width, config.height },
              { config.backX() + config.wall_thickness / 2.0, config.center_y, config.centerZ() }),
    boxObject(config, "box_left_wall", { config.depth, config.wall_thickness, config.height },
              { config.centerX(), config.maxY() + config.wall_thickness / 2.0, config.centerZ() }),
    boxObject(config, "box_right_wall", { config.depth, config.wall_thickness, config.height },
              { config.centerX(), config.minY() - config.wall_thickness / 2.0, config.centerZ() }),
    boxObject(config, "target_object", config.target_size, config.target_center),
  };
}
}  // namespace

class TopOpenCenterReachability
{
public:
  explicit TopOpenCenterReachability(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
  }

  void run()
  {
    const std::string config_path = node_->get_parameter("geometry_config").as_string();
    output_csv_ = node_->get_parameter("output_csv").as_string();
    output_audit_ = node_->get_parameter("output_audit").as_string();
    config_ = loadConfig(config_path);
    validateGeometry();

    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    robot_model_ = loader_->getModel();
    if (!robot_model_)
      throw std::runtime_error("RobotModelLoader failed");
    left_arm_group_ = robot_model_->getJointModelGroup("left_arm");
    whole_body_group_ = robot_model_->getJointModelGroup("whole_body");
    left_with_torso_group_ = robot_model_->getJointModelGroup("left_arm_with_torso");
    if (!left_arm_group_ || !whole_body_group_ || !left_with_torso_group_)
      throw std::runtime_error("Required SRDF planning group missing");
    if (!left_arm_group_->getSolverInstance())
      throw std::runtime_error("left_arm kinematics solver is unavailable");

    scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    for (const auto& object : sceneObjects(config_))
    {
      if (!scene_->processCollisionObjectMsg(object))
        throw std::runtime_error("PlanningScene rejected object: " + object.id);
    }

    stages_ = makeStages();
    candidates_ = makeCandidates();
    executeAudit();
  }

private:
  void validateGeometry() const
  {
    constexpr double tolerance = 1e-12;
    const double bottom = config_.target_center[2] - config_.target_size[2] / 2.0;
    const double top = config_.target_center[2] + config_.target_size[2] / 2.0;
    if (std::abs(bottom - config_.floor_z) > tolerance || std::abs(top - 0.990) > tolerance)
      throw std::runtime_error("Target does not touch the floor exactly at the requested Z bounds");
    if (config_.target_center[0] - config_.target_size[0] / 2.0 <= config_.front_x ||
        config_.target_center[0] + config_.target_size[0] / 2.0 >= config_.backX() ||
        config_.target_center[1] - config_.target_size[1] / 2.0 <= config_.minY() ||
        config_.target_center[1] + config_.target_size[1] / 2.0 >= config_.maxY())
      throw std::runtime_error("Target intersects or crosses a vertical box wall");
  }

  std::map<std::string, geometry_msgs::msg::Pose> makeStages() const
  {
    const auto pose_at_grasp_center_z = [&](double grasp_center_z) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = config_.target_center[0];
      pose.position.y = config_.target_center[1];
      pose.position.z = grasp_center_z + config_.tcp_to_grasp_center;
      // RPY [0, pi, 0]: TCP local +Z -> world -Z; local +Y -> world +Y.
      pose.orientation.x = 0.0;
      pose.orientation.y = 1.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 0.0;
      return pose;
    };
    const double target_top = config_.target_center[2] + config_.target_size[2] / 2.0;
    return {
      { "APPROACH", pose_at_grasp_center_z(config_.topZ() + config_.approach_clearance) },
      { "PRE_GRASP", pose_at_grasp_center_z(target_top + config_.pre_grasp_clearance) },
      { "GRASP", pose_at_grasp_center_z(config_.target_center[2]) },
    };
  }

  bool withinBound(const std::string& variable, double value) const
  {
    const moveit::core::VariableBounds& bounds = robot_model_->getVariableBounds(variable);
    return !bounds.position_bounded_ || (value >= bounds.min_position_ && value <= bounds.max_position_);
  }

  std::vector<Candidate> makeCandidates() const
  {
    std::vector<Candidate> result;
    std::vector<double> lifts;
    for (int millimeters = 0; millimeters <= 700; millimeters += 50)
      lifts.push_back(static_cast<double>(millimeters) / 1000.0);
    const std::array<double, 5> yaw_degrees = { -10.0, -5.0, 0.0, 5.0, 10.0 };
    const std::array<double, 6> pitch_degrees = { -10.0, 0.0, 10.0, 20.0, 30.0, 45.0 };
    int id = 0;
    for (const double lift : lifts)
    {
      Candidate candidate;
      candidate.mode = "LIFT_ONLY";
      candidate.id = "LIFT_ONLY_" + std::to_string(id++);
      candidate.lift = lift;
      candidate.within_limits = withinBound("lift_joint", lift) && withinBound("waist_yaw_joint", 0.0) &&
                                withinBound("waist_pitch_joint", 0.0);
      result.push_back(candidate);
    }
    id = 0;
    for (const double lift : lifts)
    {
      for (const double yaw_degrees_value : yaw_degrees)
      {
        for (const double pitch_degrees_value : pitch_degrees)
        {
          Candidate candidate;
          candidate.mode = "LIFT_YAW_PITCH";
          candidate.id = "LIFT_YAW_PITCH_" + std::to_string(id++);
          candidate.lift = lift;
          candidate.yaw = yaw_degrees_value * kPi / 180.0;
          candidate.pitch = pitch_degrees_value * kPi / 180.0;
          candidate.within_limits = withinBound("lift_joint", candidate.lift) &&
                                    withinBound("waist_yaw_joint", candidate.yaw) &&
                                    withinBound("waist_pitch_joint", candidate.pitch);
          result.push_back(candidate);
        }
      }
    }
    return result;
  }

  moveit::core::RobotState seededState(const Candidate& candidate, std::uint32_t explicit_seed) const
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setVariablePosition("lift_joint", candidate.lift);
    state.setVariablePosition("waist_yaw_joint", candidate.yaw);
    state.setVariablePosition("waist_pitch_joint", candidate.pitch);
    state.setVariablePosition("openarm_left_finger_joint1", config_.q_open);
    state.setVariablePosition("openarm_right_finger_joint1", config_.q_open);

    std::mt19937 generator(explicit_seed);
    for (const std::string& variable : left_arm_group_->getVariableNames())
    {
      const moveit::core::VariableBounds& bounds = robot_model_->getVariableBounds(variable);
      if (!bounds.position_bounded_)
        continue;
      std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
      state.setVariablePosition(variable, distribution(generator));
    }
    state.update();
    return state;
  }

  double jointMargin(const moveit::core::RobotState& state) const
  {
    double margin = std::numeric_limits<double>::infinity();
    for (const std::string& variable : left_with_torso_group_->getVariableNames())
    {
      const moveit::core::VariableBounds& bounds = robot_model_->getVariableBounds(variable);
      if (!bounds.position_bounded_)
        continue;
      const double value = state.getVariablePosition(variable);
      margin = std::min(margin, std::min(value - bounds.min_position_, bounds.max_position_ - value));
    }
    return margin;
  }

  bool isBox(const std::string& name) const
  {
    return name == "box_floor" || name == "box_front_wall" || name == "box_back_wall" ||
           name == "box_left_wall" || name == "box_right_wall";
  }

  bool isLeftGripper(const std::string& name) const
  {
    return name == "openarm_left_link7" || name == "openarm_left_left_finger" ||
           name == "openarm_left_right_finger";
  }

  CollisionAudit checkState(moveit::core::RobotState& state) const
  {
    state.update();
    CollisionAudit audit;
    audit.bounds_valid = state.satisfiesBounds(whole_body_group_);
    audit.joint_margin = jointMargin(state);
    audit.target_box_collision = false;  // Verified analytically in validateGeometry(); world/world is not an FCL robot query.

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;
    collision_detection::CollisionResult self_result;
    scene_->checkSelfCollision(request, self_result, state);
    audit.self_collision = self_result.collision;
    for (const auto& entry : self_result.contacts)
      audit.pairs.insert(entry.first);

    collision_detection::CollisionResult full_result;
    scene_->checkCollision(request, full_result, state);
    for (const auto& entry : full_result.contacts)
    {
      bool world_contact = false;
      for (const auto& contact : entry.second)
      {
        if (contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT ||
            contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
          world_contact = true;
      }
      if (!world_contact)
        continue;
      audit.pairs.insert(entry.first);
      const bool first_box = isBox(entry.first.first);
      const bool second_box = isBox(entry.first.second);
      if (first_box || second_box)
      {
        audit.robot_box_collision = true;
        const std::string& robot_link = first_box ? entry.first.second : entry.first.first;
        if (isLeftGripper(robot_link))
          audit.gripper_box_collision = true;
      }
      if (entry.first.first == "target_object" || entry.first.second == "target_object")
        audit.robot_target_collision = true;
    }

    const collision_detection::AllowedCollisionMatrix& acm = scene_->getAllowedCollisionMatrix();
    audit.environment_clearance = scene_->getCollisionEnv()->distanceRobot(state, acm);
    audit.self_clearance = scene_->getCollisionEnv()->distanceSelf(state, acm);
    audit.minimum_clearance = std::min(audit.environment_clearance, audit.self_clearance);
    return audit;
  }

  void executeAudit()
  {
    std::filesystem::create_directories(std::filesystem::path(output_csv_).parent_path());
    std::ofstream output(output_csv_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write reachability CSV");
    output << "timestamp,mode,candidate_id,lift_m,yaw_rad,pitch_rad,stage,seed_id,explicit_seed,"
              "candidate_within_limits,ik_success,joint_limits_valid,self_collision,robot_box_collision,"
              "gripper_box_collision,robot_target_collision,target_box_collision,collision_free,"
              "minimum_joint_margin,min_self_clearance_m,min_environment_clearance_m,min_collision_clearance_m,"
              "collision_link_pairs,ik_time_ms\n";

    std::map<std::string, CandidateAggregate> aggregates;
    std::size_t candidate_index = 0;
    std::size_t excluded_candidates = 0;
    for (const Candidate& candidate : candidates_)
    {
      CandidateAggregate aggregate;
      aggregate.candidate = candidate;
      if (!candidate.within_limits)
      {
        ++excluded_candidates;
        for (const auto& stage : stages_)
        {
          output << timestampNow() << ',' << candidate.mode << ',' << candidate.id << ','
                 << std::setprecision(15) << candidate.lift << ',' << candidate.yaw << ',' << candidate.pitch << ','
                 << stage.first << ",-1,0,false,false,false,false,false,false,false,false,false,nan,nan,nan,nan,"
                 << csvEscape("CANDIDATE_OUTSIDE_URDF_LIMIT") << ",0\n";
        }
        aggregates[candidate.id] = aggregate;
        ++candidate_index;
        continue;
      }

      std::size_t stage_index = 0;
      for (const auto& stage : stages_)
      {
        StageAggregate& stage_aggregate = aggregate.stages[stage.first];
        for (int seed_id = 0; seed_id < config_.seed_count; ++seed_id)
        {
          const std::uint32_t explicit_seed = config_.seed_base +
            static_cast<std::uint32_t>(candidate_index * 1000 + stage_index * 100 + seed_id);
          moveit::core::RobotState state = seededState(candidate, explicit_seed);
          const auto start = std::chrono::steady_clock::now();
          const bool ik = state.setFromIK(left_arm_group_, stage.second, "openarm_left_hand_tcp", config_.ik_timeout);
          const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
          ++stage_aggregate.seeds_tested;
          CollisionAudit audit;
          if (ik)
          {
            ++stage_aggregate.ik_success_count;
            audit = checkState(state);
            if (audit.collisionFree())
            {
              ++stage_aggregate.collision_free_count;
              stage_aggregate.best_clearance = std::max(stage_aggregate.best_clearance, audit.minimum_clearance);
              stage_aggregate.best_joint_margin = std::max(stage_aggregate.best_joint_margin, audit.joint_margin);
            }
            else
            {
              stage_aggregate.failure_pairs.insert(audit.pairs.begin(), audit.pairs.end());
            }
          }
          output << timestampNow() << ',' << candidate.mode << ',' << candidate.id << ','
                 << std::setprecision(15) << candidate.lift << ',' << candidate.yaw << ',' << candidate.pitch << ','
                 << stage.first << ',' << seed_id << ',' << explicit_seed << ",true," << (ik ? "true" : "false") << ','
                 << (ik && audit.bounds_valid ? "true" : "false") << ','
                 << (ik && audit.self_collision ? "true" : "false") << ','
                 << (ik && audit.robot_box_collision ? "true" : "false") << ','
                 << (ik && audit.gripper_box_collision ? "true" : "false") << ','
                 << (ik && audit.robot_target_collision ? "true" : "false") << ','
                 << (ik && audit.target_box_collision ? "true" : "false") << ','
                 << (ik && audit.collisionFree() ? "true" : "false") << ','
                 << (ik ? audit.joint_margin : std::numeric_limits<double>::quiet_NaN()) << ','
                 << (ik ? audit.self_clearance : std::numeric_limits<double>::quiet_NaN()) << ','
                 << (ik ? audit.environment_clearance : std::numeric_limits<double>::quiet_NaN()) << ','
                 << (ik ? audit.minimum_clearance : std::numeric_limits<double>::quiet_NaN()) << ','
                 << csvEscape(ik ? pairString(audit.pairs) : "IK_FAILURE") << ',' << elapsed_ms << '\n';
        }
        ++stage_index;
      }
      aggregates[candidate.id] = aggregate;
      if ((candidate_index + 1) % 25 == 0)
        RCLCPP_INFO(node_->get_logger(), "Reachability candidates processed: %zu / %zu",
                    candidate_index + 1, candidates_.size());
      ++candidate_index;
    }
    output.flush();
    if (!output)
      throw std::runtime_error("Reachability CSV flush failed");
    output.close();
    writeSummary(aggregates, excluded_candidates);
  }

  void writeSummary(const std::map<std::string, CandidateAggregate>& aggregates,
                    std::size_t excluded_candidates) const
  {
    const CandidateAggregate* best_lift_only = nullptr;
    const CandidateAggregate* best_proposed = nullptr;
    std::map<std::string, bool> baseline_stage_possible;
    std::map<std::string, bool> proposed_stage_possible;
    std::set<std::pair<std::string, std::string>> all_failure_pairs;
    for (const auto& entry : aggregates)
    {
      const CandidateAggregate& aggregate = entry.second;
      for (const auto& stage : aggregate.stages)
      {
        const bool possible = stage.second.collision_free_count > 0;
        if (aggregate.candidate.mode == "LIFT_ONLY")
          baseline_stage_possible[stage.first] = baseline_stage_possible[stage.first] || possible;
        else
          proposed_stage_possible[stage.first] = proposed_stage_possible[stage.first] || possible;
        all_failure_pairs.insert(stage.second.failure_pairs.begin(), stage.second.failure_pairs.end());
      }
      if (!aggregate.allStagesPossible())
        continue;
      if (aggregate.candidate.mode == "LIFT_ONLY")
      {
        if (!best_lift_only || aggregate.taskClearance() > best_lift_only->taskClearance())
          best_lift_only = &aggregate;
      }
      else if (!best_proposed || aggregate.taskClearance() > best_proposed->taskClearance())
      {
        best_proposed = &aggregate;
      }
    }

    std::ofstream output(output_audit_, std::ios::trunc);
    if (!output)
      throw std::runtime_error("Cannot write reachability audit report");
    output << "# Top-open center reachability audit\n\n";
    output << "- Scene: `" << kSceneId << "`\n";
    output << "- Scope: explicit-seed IK and static RobotState collision checks only\n";
    output << "- IK solver: `kdl_kinematics_plugin/KDLKinematicsPlugin` for `left_arm`\n";
    output << "- IK seeds per valid candidate/stage: " << config_.seed_count << "\n";
    output << "- Per-seed IK timeout: " << config_.ik_timeout << " s\n";
    output << "- Candidates excluded by exact URDF limits: " << excluded_candidates << "\n";
    output << "- MoveGroup/OMPL/trajectory/attach/execution/controller/hardware: not used\n\n";
    output << "## Stage poses\n\n";
    for (const auto& stage : stages_)
      output << "- " << stage.first << " TCP: [" << stage.second.position.x << ", "
             << stage.second.position.y << ", " << stage.second.position.z
             << "], quaternion xyzw [0, 1, 0, 0]\n";
    output << "\n## Stage feasibility across candidates\n\n";
    output << "| Stage | LIFT_ONLY | LIFT_YAW_PITCH | Proposed-only |\n|---|---|---|---|\n";
    for (const char* stage : { "APPROACH", "PRE_GRASP", "GRASP" })
      output << '|' << stage << '|' << (baseline_stage_possible[stage] ? "possible" : "not found") << '|'
             << (proposed_stage_possible[stage] ? "possible" : "not found") << '|'
             << (!baseline_stage_possible[stage] && proposed_stage_possible[stage] ? "yes" : "no") << "|\n";
    output << "\n## Best complete candidates by minimum collision clearance\n\n";
    if (best_lift_only)
      output << "- LIFT_ONLY: Lift=" << best_lift_only->candidate.lift << " m, minimum three-stage clearance="
             << best_lift_only->taskClearance() << " m\n";
    else
      output << "- LIFT_ONLY: no candidate made all three stages collision-free\n";
    if (best_proposed)
      output << "- LIFT_YAW_PITCH: Lift=" << best_proposed->candidate.lift << " m, Yaw="
             << best_proposed->candidate.yaw << " rad, Pitch=" << best_proposed->candidate.pitch
             << " rad, minimum three-stage clearance=" << best_proposed->taskClearance() << " m\n";
    else
      output << "- LIFT_YAW_PITCH: no candidate made all three stages collision-free\n";
    output << "\nObserved failed-solution collision pairs: `" << pairString(all_failure_pairs) << "`\n\n";
    output << "## Gate for reference generation\n\n";
    output << (best_lift_only ?
      "LIFT_ONLY has at least one collision-free IK solution for APPROACH, PRE_GRASP, and GRASP. The IK-only gate is satisfied; OMPL reference generation still requires explicit user approval.\n" :
      "LIFT_ONLY did not make all three stages feasible. Do not generate an OMPL reference path.\n");
    output.flush();
    if (!output)
      throw std::runtime_error("Reachability audit report flush failed");

    RCLCPP_INFO(node_->get_logger(), "Reachability audit complete. LIFT_ONLY all stages: %s",
                best_lift_only ? "YES" : "NO");
  }

  rclcpp::Node::SharedPtr node_;
  AuditConfig config_;
  std::string output_csv_;
  std::string output_audit_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* left_arm_group_{ nullptr };
  const moveit::core::JointModelGroup* whole_body_group_{ nullptr };
  const moveit::core::JointModelGroup* left_with_torso_group_{ nullptr };
  planning_scene::PlanningScenePtr scene_;
  std::map<std::string, geometry_msgs::msg::Pose> stages_;
  std::vector<Candidate> candidates_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<rclcpp::Node>(
      "top_open_center_reachability",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    TopOpenCenterReachability audit(node);
    audit.run();
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("top_open_center_reachability"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
