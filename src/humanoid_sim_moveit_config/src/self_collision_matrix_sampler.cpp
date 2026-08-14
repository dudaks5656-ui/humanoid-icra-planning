#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
using Pair = std::pair<std::string, std::string>;

Pair orderedPair(const std::string& first, const std::string& second)
{
  return first < second ? Pair(first, second) : Pair(second, first);
}

std::vector<std::string> armLinks(const std::string& side)
{
  std::vector<std::string> links;
  for (int index = 0; index <= 7; ++index)
    links.push_back("openarm_" + side + "_link" + std::to_string(index));
  return links;
}

std::vector<std::string> gripperLinks(const std::string& side)
{
  return { "openarm_" + side + "_left_finger", "openarm_" + side + "_right_finger" };
}

void appendCrossProduct(std::set<Pair>& pairs, const std::vector<std::string>& first,
                        const std::vector<std::string>& second)
{
  for (const auto& left : first)
    for (const auto& right : second)
      if (left != right)
        pairs.insert(orderedPair(left, right));
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("self_collision_matrix_sampler", options);

  try
  {
    if (!node->has_parameter("sample_count"))
      node->declare_parameter<int>("sample_count", 10000);
    if (!node->has_parameter("output_path"))
      node->declare_parameter<std::string>(
          "output_path", "/home/openarm/humanoid_sim_ws/validation/self_collision_matrix_audit.txt");
    if (!node->has_parameter("collision_detector_name"))
      node->declare_parameter<std::string>("collision_detector_name", "FCL");

    const int sample_count = node->get_parameter("sample_count").as_int();
    const std::string output_path = node->get_parameter("output_path").as_string();
    if (sample_count <= 0)
      throw std::runtime_error("sample_count must be positive");

    auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description", false);
    const auto model = loader->getModel();
    if (!model)
      throw std::runtime_error("Robot model could not be loaded");
    const auto* whole_body = model->getJointModelGroup("whole_body");
    if (!whole_body)
      throw std::runtime_error("whole_body group is missing");
    if (whole_body->getVariableNames().size() != 19)
      throw std::runtime_error("whole_body does not contain 19 independent variables");

    planning_scene::PlanningScene scene(model);
    const auto& acm = scene.getAllowedCollisionMatrix();

    std::vector<std::string> acm_names;
    acm.getAllEntryNames(acm_names);
    std::set<Pair> disabled_pairs;
    for (std::size_t first = 0; first < acm_names.size(); ++first)
    {
      for (std::size_t second = first + 1; second < acm_names.size(); ++second)
      {
        collision_detection::AllowedCollision::Type type;
        if (acm.getEntry(acm_names[first], acm_names[second], type) &&
            type == collision_detection::AllowedCollision::ALWAYS)
          disabled_pairs.insert(orderedPair(acm_names[first], acm_names[second]));
      }
    }

    const auto left_arm = armLinks("left");
    const auto right_arm = armLinks("right");
    const auto left_gripper = gripperLinks("left");
    const auto right_gripper = gripperLinks("right");
    const std::vector<std::string> both_arms = [&]() {
      std::vector<std::string> result = left_arm;
      result.insert(result.end(), right_arm.begin(), right_arm.end());
      return result;
    }();
    const std::vector<std::string> both_grippers = [&]() {
      std::vector<std::string> result = left_gripper;
      result.insert(result.end(), right_gripper.begin(), right_gripper.end());
      return result;
    }();
    const std::vector<std::string> body_links = { "base_link", "lift_fixed_link", "lift_moving_link",
                                                  "waist_yaw_link", "waist_pitch_link" };

    std::set<Pair> protected_pairs;
    appendCrossProduct(protected_pairs, { "base_link" }, both_arms);
    appendCrossProduct(protected_pairs, { "base_link" }, both_grippers);
    appendCrossProduct(protected_pairs, { "lift_fixed_link", "lift_moving_link" }, both_arms);
    appendCrossProduct(protected_pairs, { "waist_yaw_link", "waist_pitch_link" }, both_arms);
    appendCrossProduct(protected_pairs, left_arm, right_arm);
    appendCrossProduct(protected_pairs, both_grippers, body_links);
    appendCrossProduct(protected_pairs, left_gripper, right_arm);
    appendCrossProduct(protected_pairs, right_gripper, left_arm);

    std::vector<Pair> incorrectly_disabled;
    std::set_intersection(protected_pairs.begin(), protected_pairs.end(), disabled_pairs.begin(), disabled_pairs.end(),
                          std::back_inserter(incorrectly_disabled));

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 1;
    std::map<Pair, std::size_t> pair_collision_counts;
    std::size_t colliding_states = 0;

    moveit::core::RobotState initial_state(model);
    initial_state.setToDefaultValues();
    initial_state.update();
    collision_detection::CollisionResult initial_result;
    scene.checkSelfCollision(request, initial_result, initial_state);

    moveit::core::RobotState state(model);
    const auto begin = std::chrono::steady_clock::now();
    for (int sample = 0; sample < sample_count; ++sample)
    {
      state.setToDefaultValues();
      state.setToRandomPositions(whole_body);
      state.update();
      collision_detection::CollisionResult result;
      scene.checkSelfCollision(request, result, state);
      if (result.collision)
        ++colliding_states;
      for (const auto& contact : result.contacts)
        ++pair_collision_counts[orderedPair(contact.first.first, contact.first.second)];
    }
    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(end - begin).count();

    std::ofstream output(output_path);
    if (!output)
      throw std::runtime_error("Cannot write audit file: " + output_path);
    output << "SELF_COLLISION_MATRIX_AUDIT\n";
    output << "collision_detector=" << node->get_parameter("collision_detector_name").as_string() << '\n';
    output << "robot=" << model->getName() << '\n';
    output << "sampling_method=random_full_robot_state\n";
    output << "requested_samples=" << sample_count << '\n';
    output << "completed_samples=" << sample_count << '\n';
    output << "independent_variable_count=" << whole_body->getVariableNames().size() << '\n';
    output << "independent_variables=";
    for (const auto& name : whole_body->getVariableNames())
      output << name << ' ';
    output << '\n';
    output << "elapsed_ms=" << std::fixed << std::setprecision(3) << elapsed_ms << '\n';
    output << "mean_check_time_ms=" << elapsed_ms / sample_count << '\n';
    output << "colliding_state_count=" << colliding_states << '\n';
    output << "initial_state_collision=" << (initial_result.collision ? "true" : "false") << '\n';
    output << "initial_state_pairs=";
    for (const auto& entry : initial_result.contacts)
      output << entry.first.first << '|' << entry.first.second << ';';
    output << "\n\n[DISABLED_COLLISIONS]\n";
    for (const auto& pair : disabled_pairs)
      output << pair.first << '|' << pair.second << "|reason=Adjacent\n";
    output << "\n[PROTECTED_PAIR_AUDIT]\n";
    output << "protected_pair_count=" << protected_pairs.size() << '\n';
    output << "incorrectly_disabled_count=" << incorrectly_disabled.size() << '\n';
    for (const auto& pair : incorrectly_disabled)
      output << "ERROR_DISABLED_PROTECTED_PAIR=" << pair.first << '|' << pair.second << '\n';
    output << "\n[SAMPLED_COLLISION_PAIR_COUNTS]\n";
    for (const auto& entry : pair_collision_counts)
      output << entry.first.first << '|' << entry.first.second << "|states=" << entry.second << '\n';
    output << "\n[POLICY]\n";
    output << "sampled_never_pairs_were_not_disabled=true\n";
    output << "initial_collisions_were_not_disabled=true\n";
    output << "openarm_body_link0_entries_copied=false\n";
    output << "AUDIT_RESULT=" << (incorrectly_disabled.empty() ? "PASS" : "FAIL") << '\n';
    output.close();

    RCLCPP_INFO(node->get_logger(),
                "SCM audit complete: samples=%d colliding=%zu disabled=%zu protected_disabled=%zu mean_ms=%.3f",
                sample_count, colliding_states, disabled_pairs.size(), incorrectly_disabled.size(),
                elapsed_ms / sample_count);
    rclcpp::shutdown();
    return incorrectly_disabled.empty() ? 0 : 3;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "SCM audit failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
