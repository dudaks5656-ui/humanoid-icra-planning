#include <array>
#include <cstdint>
#include <random>
#include <unordered_map>

#define main preserved_fixed_base_workspace_main
#include "../../fixed_base_workspace_analysis/src/fixed_base_workspace.cpp"
#undef main

namespace radial_workspace_validation
{
using namespace fixed_base_workspace;

struct Ray
{
  std::string name;
  Eigen::Vector3d direction;
  double first{};
  double last{};
  std::vector<double> distances;
};

struct RadialEvaluation
{
  Result result;
  std::shared_ptr<moveit::core::RobotState> state;
};

struct CsvPoint
{
  double x{}, y{}, z{};
};

std::vector<std::string> splitCsv(const std::string& line)
{
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i)
  {
    const char c = line[i];
    if (quoted)
    {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') { field.push_back('"'); ++i; }
      else if (c == '"') quoted = false;
      else field.push_back(c);
    }
    else if (c == '"') quoted = true;
    else if (c == ',') { fields.push_back(field); field.clear(); }
    else field.push_back(c);
  }
  fields.push_back(field);
  return fields;
}

class RadialRunner : public Runner
{
public:
  explicit RadialRunner(const rclcpp::Node::SharedPtr& node) : Runner(node)
  {
    comparison_csv_ = parameter<std::string>("comparison_csv");
    radial_min_ = parameter<double>("radial_min");
    radial_max_ = parameter<double>("radial_max");
    radial_step_ = parameter<double>("radial_step");
    max_points_per_ray_ = parameter<int>("max_points_per_ray");
    ray_count_hard_ = parameter<int>("ray_count_hard");
    config_count_hard_ = parameter<int>("config_count_hard");
    origin_offset_ = parameter<double>("origin_x_offset_from_grid_min");
    left_ratio_ = parameter<double>("front_left_ratio");
    up_ratio_ = parameter<double>("front_up_ratio");
    random_seed_ = parameter<int>("random_seed");
    max_special_ik_seeds_ = parameter<int>("max_special_ik_seeds");
    max_special_points_ = parameter<int>("max_special_points");
    for (const auto& flag : { "trajectory_execution", "controller_enabled", "ros2_control_enabled",
                              "hardware_enabled", "amr_motion_enabled" })
      if (parameter<bool>(flag)) throw std::runtime_error(std::string("Forbidden safety flag: ") + flag);
  }

  void runRadial()
  {
    start_time_ = Clock::now();
    timestamp_ = isoTimestamp();
    std::filesystem::create_directories(output_dir_);
    loadValidatedBounds();
    buildCandidates();
    buildRays();
    preflightRadial();

    std::size_t point_id = 0;
    for (std::size_t ray_index = 0; ray_index < rays_.size(); ++ray_index)
      for (const double distance : rays_[ray_index].distances)
      {
        Point point;
        point.id = point_id++;
        point.i = static_cast<int>(ray_index);
        point.j = static_cast<int>(std::llround((distance - radial_min_) / radial_step_));
        point.k = 0;
        point.xyz = origin_ + rays_[ray_index].direction * distance;
        for (const auto config : { Configuration::LIFT_ONLY, Configuration::LIFT_YAW_PITCH })
        {
          const auto& candidates = config == Configuration::LIFT_ONLY ? c0_candidates_ : c3_candidates_;
          auto evaluation = evaluate(point, config, candidates, max_ik_seeds_);
          records_.push_back({ ray_index, distance, std::move(evaluation) });
          const auto& result = records_.back().evaluation.result;
          RCLCPP_INFO(node_->get_logger(),
            "RADIAL_WORKSPACE config=%s ray=%s r=%.2f xyz=(%.4f,%.4f,%.4f) result=%s reason=%s seeds=%d",
            configName(config).c_str(), rays_[ray_index].name.c_str(), distance,
            point.xyz.x(), point.xyz.y(), point.xyz.z(), result.success ? "PASS" : "FAIL",
            result.failure_reason.c_str(), result.seeds_tested);
        }
      }
    revalidateInternalAnomalies();
    writeOutputs();
  }

private:
  struct Record
  {
    std::size_t ray_index{};
    double distance{};
    RadialEvaluation evaluation;
  };

  void loadValidatedBounds()
  {
    std::ifstream input(comparison_csv_);
    if (!input) throw std::runtime_error("Cannot read immutable comparison CSV: " + comparison_csv_);
    std::string line;
    std::getline(input, line);
    const auto header = splitCsv(line);
    std::unordered_map<std::string, std::size_t> column;
    for (std::size_t i = 0; i < header.size(); ++i) column[header[i]] = i;
    for (const auto& required : { "point_id", "tcp_x", "tcp_y", "tcp_z" })
      if (!column.count(required)) throw std::runtime_error(std::string("Missing source column: ") + required);
    std::set<std::string> ids;
    std::vector<double> xs, ys, zs;
    while (std::getline(input, line))
    {
      if (line.empty()) continue;
      const auto fields = splitCsv(line);
      if (fields.size() != header.size()) throw std::runtime_error("Malformed source comparison CSV");
      if (!ids.insert(fields[column["point_id"]]).second) throw std::runtime_error("Duplicate source point ID");
      xs.push_back(std::stod(fields[column["tcp_x"]]));
      ys.push_back(std::stod(fields[column["tcp_y"]]));
      zs.push_back(std::stod(fields[column["tcp_z"]]));
    }
    if (ids.size() != 1440) throw std::runtime_error("Source point count is not the validated 1,440");
    auto unique = [](std::vector<double> values) {
      std::sort(values.begin(), values.end());
      values.erase(std::unique(values.begin(), values.end(), [](double a, double b) { return std::abs(a-b)<1e-11; }), values.end());
      return values;
    };
    axes_[0] = unique(xs); axes_[1] = unique(ys); axes_[2] = unique(zs);
    if (axes_[0].size()!=12 || axes_[1].size()!=10 || axes_[2].size()!=12)
      throw std::runtime_error("Source grid dimensions changed");
    for (int axis = 0; axis < 3; ++axis)
    {
      spacing_[axis] = axes_[axis][1] - axes_[axis][0];
      outer_min_[axis] = axes_[axis].front() - 0.5 * spacing_[axis];
      outer_max_[axis] = axes_[axis].back() + 0.5 * spacing_[axis];
    }
    origin_ = { axes_[0].front() - origin_offset_,
                0.5 * (axes_[1].front() + axes_[1].back()),
                0.5 * (axes_[2].front() + axes_[2].back()) };
  }

  bool inside(const std::string& joint, double value) const
  {
    const auto& bound = model_->getVariableBounds(joint);
    return value > bound.min_position_ + exact_bound_epsilon_ && value < bound.max_position_ - exact_bound_epsilon_;
  }

  void buildCandidates()
  {
    const std::vector<double> lifts{0.25,0.35,0.30,0.20,0.40,0.15,0.45,0.05,0.55,0.65};
    const double degree = std::acos(-1.0) / 180.0;
    const std::vector<double> yaws{0,5*degree,-5*degree,10*degree,-10*degree,15*degree,-15*degree,20*degree,-20*degree};
    const std::vector<double> pitches{0,2.5*degree,-2.5*degree,5*degree,-5*degree,7.5*degree,-7.5*degree,10*degree,-10*degree};
    for (const double lift : lifts) if (inside("lift_joint", lift)) c0_candidates_.push_back({lift,0,0});
    if (c0_candidates_.empty()) throw std::runtime_error("No valid lift candidates");
    c3_candidates_ = c0_candidates_;
    std::vector<TorsoCandidate> extra;
    for (const double lift : lifts) if (inside("lift_joint", lift))
      for (const double yaw : yaws) if (inside("waist_yaw_joint", yaw))
        for (const double pitch : pitches) if (inside("waist_pitch_joint", pitch) && (std::abs(yaw)>1e-12 || std::abs(pitch)>1e-12))
          extra.push_back({lift,yaw,pitch});
    std::stable_sort(extra.begin(), extra.end(), [](const TorsoCandidate& a, const TorsoCandidate& b) {
      const double da = std::abs(a.yaw) + std::abs(a.pitch);
      const double db = std::abs(b.yaw) + std::abs(b.pitch);
      if (std::abs(da-db)>1e-12) return da < db;
      return std::abs(a.lift-0.30) < std::abs(b.lift-0.30);
    });
    for (const auto& candidate : extra)
      if (c3_candidates_.size() < static_cast<std::size_t>(max_torso_candidates_)) c3_candidates_.push_back(candidate);
  }

  std::pair<double,double> intersectAabb(const Eigen::Vector3d& direction) const
  {
    double near = -kInf, far = kInf;
    for (int axis = 0; axis < 3; ++axis)
    {
      if (std::abs(direction[axis]) < 1e-12)
      {
        if (origin_[axis] < outer_min_[axis] || origin_[axis] > outer_max_[axis]) return {1,0};
        continue;
      }
      double a = (outer_min_[axis] - origin_[axis]) / direction[axis];
      double b = (outer_max_[axis] - origin_[axis]) / direction[axis];
      if (a > b) std::swap(a,b);
      near = std::max(near,a); far = std::min(far,b);
    }
    return {std::max(radial_min_, near), std::min(radial_max_, far)};
  }

  void buildRays()
  {
    std::vector<std::pair<std::string,Eigen::Vector3d>> definitions{
      {"FRONT", {1,0,0}}, {"FRONT_LEFT", {1,left_ratio_,0}}, {"FRONT_RIGHT", {1,-left_ratio_,0}},
      {"FRONT_UP", {1,0,up_ratio_}}, {"FRONT_DOWN", {1,0,-up_ratio_}} };
    for (auto& definition : definitions)
    {
      definition.second.normalize();
      const auto [first,last] = intersectAabb(definition.second);
      if (last + 1e-12 < first) throw std::runtime_error("Ray misses validated envelope AABB: " + definition.first);
      Ray ray{definition.first, definition.second, first, last, {}};
      const int first_index = static_cast<int>(std::ceil((first-radial_min_-1e-12)/radial_step_));
      for (int i = std::max(0,first_index); ; ++i)
      {
        const double distance = radial_min_ + i * radial_step_;
        if (distance > last + 1e-12) break;
        ray.distances.push_back(distance);
      }
      if (ray.distances.empty() || ray.distances.size() > static_cast<std::size_t>(max_points_per_ray_))
        throw std::runtime_error("Invalid radial sample count for " + definition.first);
      rays_.push_back(ray);
    }
  }

  void preflightRadial() const
  {
    if (rays_.size()!=static_cast<std::size_t>(ray_count_hard_) || config_count_hard_!=2)
      throw std::runtime_error("Ray/config hard contract mismatch");
    std::size_t physical = 0;
    for (const auto& ray : rays_) physical += ray.distances.size();
    const std::size_t evaluations = physical * 2;
    if (evaluations > 400 || evaluations > static_cast<std::size_t>(max_configuration_evaluations_))
      throw std::runtime_error("Radial evaluation hard cap exceeded");
    if (evaluations * static_cast<std::size_t>(max_ik_seeds_) > static_cast<std::size_t>(max_total_ik_attempts_))
      throw std::runtime_error("Radial IK attempt hard cap exceeded");
    RCLCPP_INFO(node_->get_logger(),
      "RADIAL_WORKSPACE PREFLIGHT manifests=PASS source_points=1440 rays=5 configs=2 physical=%zu evaluations=%zu max_ik_attempts=%zu origin=(%.6f,%.6f,%.6f) execution=NO controller=NO hardware=NO",
      physical, evaluations, evaluations*static_cast<std::size_t>(max_ik_seeds_), origin_.x(),origin_.y(),origin_.z());
    for (const auto& ray : rays_)
      RCLCPP_INFO(node_->get_logger(), "RAY %s vector=(%.9f,%.9f,%.9f) AABB_range=[%.4f,%.4f] samples=%zu",
                  ray.name.c_str(),ray.direction.x(),ray.direction.y(),ray.direction.z(),ray.first,ray.last,ray.distances.size());
  }

  void seedDeterministic(moveit::core::RobotState& state, int seed, const Point& point) const
  {
    if (seed == 0) return;
    const auto& names = arm_group_->getVariableNames();
    if (seed % 2 == 1)
    {
      seedArm(state, static_cast<std::size_t>(seed), point.id);
      return;
    }
    std::mt19937_64 engine(static_cast<std::uint64_t>(random_seed_) + point.id*1000003ULL + static_cast<std::uint64_t>(seed)*9176ULL);
    for (const auto& name : names)
    {
      const auto& bound = model_->getVariableBounds(name);
      const double inset = std::max(exact_bound_epsilon_*10.0,(bound.max_position_-bound.min_position_)*1e-6);
      std::uniform_real_distribution<double> distribution(bound.min_position_+inset,bound.max_position_-inset);
      state.setVariablePosition(name,distribution(engine));
    }
  }

  RadialEvaluation evaluate(const Point& point, Configuration config, const std::vector<TorsoCandidate>& candidates,
                            int maximum_seeds)
  {
    const auto begin = Clock::now();
    RadialEvaluation output;
    output.result.point = point; output.result.configuration = config;
    std::map<std::string,int> failures;
    const auto active = activeNames(config);
    for (int seed=0; seed<maximum_seeds; ++seed)
    {
      enforceWallTime();
      auto state = std::make_shared<moveit::core::RobotState>(nominalState());
      const auto& torso = candidates[static_cast<std::size_t>(seed)%candidates.size()];
      setTorso(*state,torso); seedDeterministic(*state,seed,point); state->update();
      const auto target = targetPoseInModel(point,*state);
      ++output.result.seeds_tested;
      if (!state->setFromIK(arm_group_,target,tcp_frame_,ik_timeout_s_)) { ++failures["NO_IK"]; continue; }
      setTorso(*state,torso); state->update();
      if (!state->satisfiesBounds()) { ++failures["JOINT_LIMIT_VIOLATION"]; continue; }
      const double margin = jointMargin(*state,active);
      if (!(margin>exact_bound_epsilon_)) { ++failures["ACTIVE_JOINT_AT_BOUND"]; continue; }
      const double orientation = orientationError(*state);
      if (!(orientation<=orientation_tolerance_)) { ++failures["ORIENTATION_ERROR"]; continue; }
      collision_detection::CollisionRequest request; request.contacts=true; request.max_contacts=1000; request.max_contacts_per_pair=50;
      collision_detection::CollisionResult collision; scene_->checkSelfCollision(request,collision,*state);
      if (collision.collision) { ++failures["SELF_COLLISION"]; continue; }
      auto& result = output.result;
      result.success=true; result.failure_reason="REACHABLE"; result.valid_count=1; result.selected_seed=seed;
      result.lift=torso.lift; result.yaw=torso.yaw; result.pitch=torso.pitch; result.joint_margin=margin;
      result.active_revolute_margin=activeRevoluteMargin(*state,active);
      result.self_clearance=scene_->getCollisionEnv()->distanceSelf(*state,scene_->getAllowedCollisionMatrix());
      result.orientation_error=orientation; output.state=state;
      break;
    }
    if (!output.result.success)
    {
      const std::vector<std::string> priority{"SELF_COLLISION","ACTIVE_JOINT_AT_BOUND","JOINT_LIMIT_VIOLATION","ORIENTATION_ERROR","NO_IK"};
      int largest=-1;
      for (const auto& reason:priority) if (failures[reason]>largest) { largest=failures[reason]; output.result.failure_reason=reason; }
    }
    output.result.runtime_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
    return output;
  }

  void revalidateInternalAnomalies()
  {
    std::vector<std::size_t> targets;
    for (const auto config:{Configuration::LIFT_ONLY,Configuration::LIFT_YAW_PITCH})
      for (std::size_t ray=0;ray<rays_.size();++ray)
      {
        std::vector<std::size_t> indices;
        for (std::size_t i=0;i<records_.size();++i)
          if (records_[i].ray_index==ray && records_[i].evaluation.result.configuration==config) indices.push_back(i);
        std::sort(indices.begin(),indices.end(),[&](std::size_t a,std::size_t b){return records_[a].distance<records_[b].distance;});
        int first=-1,last=-1;
        for (int i=0;i<static_cast<int>(indices.size());++i) if (records_[indices[i]].evaluation.result.success) { if (first<0) first=i; last=i; }
        if (first<0) continue;
        for (int i=first+1;i<last;++i) if (!records_[indices[i]].evaluation.result.success) targets.push_back(indices[i]);
      }
    if (targets.size()>static_cast<std::size_t>(max_special_points_))
      throw std::runtime_error("Internal anomaly count exceeds special validation hard cap");
    for (const std::size_t index:targets)
    {
      auto& record=records_[index];
      const auto config=record.evaluation.result.configuration;
      const auto& candidates=config==Configuration::LIFT_ONLY?c0_candidates_:c3_candidates_;
      RCLCPP_INFO(node_->get_logger(),"RADIAL_SPECIAL_REVALIDATION config=%s ray=%s r=%.2f seeds=%d",
                  configName(config).c_str(),rays_[record.ray_index].name.c_str(),record.distance,max_special_ik_seeds_);
      record.evaluation=evaluate(record.evaluation.result.point,config,candidates,max_special_ik_seeds_);
      RCLCPP_INFO(node_->get_logger(),"RADIAL_SPECIAL_RESULT config=%s ray=%s r=%.2f result=%s reason=%s seeds=%d",
                  configName(config).c_str(),rays_[record.ray_index].name.c_str(),record.distance,
                  record.evaluation.result.success?"PASS":"FAIL",record.evaluation.result.failure_reason.c_str(),
                  record.evaluation.result.seeds_tested);
    }
    special_revalidated_=targets.size();
  }

  std::vector<const Record*> series(std::size_t ray, Configuration config) const
  {
    std::vector<const Record*> out;
    for (const auto& record:records_)
      if (record.ray_index==ray && record.evaluation.result.configuration==config) out.push_back(&record);
    std::sort(out.begin(),out.end(),[](const Record* a,const Record* b){return a->distance<b->distance;});
    return out;
  }

  std::vector<std::pair<int,int>> runs(const std::vector<const Record*>& rows, bool success) const
  {
    std::vector<std::pair<int,int>> out;
    for (int i=0;i<static_cast<int>(rows.size());)
    {
      if (rows[static_cast<std::size_t>(i)]->evaluation.result.success!=success) { ++i; continue; }
      int end=i;
      while (end+1<static_cast<int>(rows.size()) && rows[static_cast<std::size_t>(end+1)]->evaluation.result.success==success) ++end;
      out.push_back({i,end}); i=end+1;
    }
    return out;
  }

  void writeState(std::ofstream& out, const Record& record, const std::string& role) const
  {
    if (!record.evaluation.state) return;
    const auto& names=model_->getVariableNames();
    out << configName(record.evaluation.result.configuration) << ',' << rays_[record.ray_index].name << ',' << role << ',' << number(record.distance) << ',';
    for (std::size_t i=0;i<names.size();++i) { if (i) out << ';'; out << names[i]; }
    out << ',';
    for (std::size_t i=0;i<names.size();++i) { if (i) out << ';'; out << number(record.evaluation.state->getVariablePosition(names[i])); }
    out << "\n";
  }

  void writeOutputs() const
  {
    {
      std::ofstream out(output_dir_+"/radial_workspace_validation_points.csv",std::ios::trunc);
      out << "configuration,ray_name,ray_index,distance,tcp_x,tcp_y,tcp_z,success,failure_reason,ik_seeds_tested,selected_lift,selected_yaw,selected_pitch,joint_margin,self_clearance,orientation_error\n";
      for (const auto& record:records_)
      {
        const auto& r=record.evaluation.result;
        out << configName(r.configuration) << ',' << rays_[record.ray_index].name << ',' << record.ray_index << ',' << number(record.distance) << ','
            << number(r.point.xyz.x()) << ',' << number(r.point.xyz.y()) << ',' << number(r.point.xyz.z()) << ',' << (r.success?1:0) << ',' << r.failure_reason << ','
            << r.seeds_tested << ',' << number(r.lift) << ',' << number(r.yaw) << ',' << number(r.pitch) << ',' << number(r.joint_margin) << ','
            << number(r.self_clearance) << ',' << number(r.orientation_error) << '\n';
      }
    }
    std::ofstream intervals(output_dir_+"/radial_workspace_validation_intervals.csv",std::ios::trunc);
    std::ofstream holes(output_dir_+"/radial_workspace_validation_holes.csv",std::ios::trunc);
    std::ofstream summary(output_dir_+"/radial_workspace_validation_summary.csv",std::ios::trunc);
    std::ofstream states(output_dir_+"/radial_workspace_validation_states.csv",std::ios::trunc);
    intervals << "configuration,ray_name,interval_index,start_distance,end_distance,interval_length,sample_count\n";
    holes << "configuration,ray_name,hole_index,start_distance,end_distance,hole_length\n";
    summary << "configuration,ray_name,first_feasible_distance,last_feasible_distance,feasible_span,feasible_interval_count,hole_count,largest_hole,pass_count,fail_count\n";
    states << "configuration,ray_name,pose_role,distance,joint_names,joint_positions\n";
    for (const auto config:{Configuration::LIFT_ONLY,Configuration::LIFT_YAW_PITCH})
      for (std::size_t ray=0;ray<rays_.size();++ray)
      {
        const auto rows=series(ray,config); const auto feasible=runs(rows,true);
        int pass=0; for (const auto* row:rows) pass += row->evaluation.result.success?1:0;
        for (std::size_t i=0;i<feasible.size();++i)
        {
          const auto [a,b]=feasible[i];
          intervals << configName(config) << ',' << rays_[ray].name << ',' << i+1 << ',' << number(rows[a]->distance) << ',' << number(rows[b]->distance) << ','
                    << number(rows[b]->distance-rows[a]->distance) << ',' << b-a+1 << '\n';
        }
        std::vector<std::pair<int,int>> internal_holes;
        if (!feasible.empty())
          for (const auto& hole:runs(rows,false)) if (hole.first>feasible.front().first && hole.second<feasible.back().second) internal_holes.push_back(hole);
        double largest=0;
        for (std::size_t i=0;i<internal_holes.size();++i)
        {
          const auto [a,b]=internal_holes[i]; const double width=rows[b]->distance-rows[a]->distance+radial_step_; largest=std::max(largest,width);
          holes << configName(config) << ',' << rays_[ray].name << ',' << i+1 << ',' << number(rows[a]->distance) << ',' << number(rows[b]->distance) << ',' << number(width) << '\n';
        }
        const double first=feasible.empty()?kNaN:rows[feasible.front().first]->distance;
        const double last=feasible.empty()?kNaN:rows[feasible.back().second]->distance;
        summary << configName(config) << ',' << rays_[ray].name << ',' << number(first) << ',' << number(last) << ',' << number(last-first) << ','
                << feasible.size() << ',' << internal_holes.size() << ',' << number(largest) << ',' << pass << ',' << rows.size()-pass << '\n';
        if (!feasible.empty()) { writeState(states,*rows[feasible.front().first],"FIRST_FEASIBLE"); writeState(states,*rows[feasible.back().second],"LAST_FEASIBLE"); }
      }
    std::ofstream metadata(output_dir_+"/radial_workspace_validation_metadata.csv",std::ios::trunc);
    metadata << "key,value\n" << "timestamp," << timestamp_ << '\n' << "source_comparison," << comparison_csv_ << '\n'
             << "base_frame," << base_frame_ << '\n' << "tcp_frame," << tcp_frame_ << '\n'
             << "origin_x," << number(origin_.x()) << '\n' << "origin_y," << number(origin_.y()) << '\n' << "origin_z," << number(origin_.z()) << '\n'
             << "radial_min," << number(radial_min_) << '\n' << "radial_max," << number(radial_max_) << '\n' << "radial_step," << number(radial_step_) << '\n'
             << "max_ik_seeds," << max_ik_seeds_ << '\n' << "physical_points," << records_.size()/2 << '\n' << "configuration_evaluations," << records_.size() << '\n';
    metadata << "max_special_ik_seeds," << max_special_ik_seeds_ << '\n' << "special_revalidated_points," << special_revalidated_ << '\n';
    for (const auto& ray:rays_)
      metadata << "ray_" << ray.name << "," << number(ray.direction.x()) << ';' << number(ray.direction.y()) << ';' << number(ray.direction.z()) << '\n';
  }

  std::string comparison_csv_;
  double radial_min_{}, radial_max_{}, radial_step_{}, origin_offset_{}, left_ratio_{}, up_ratio_{};
  int max_points_per_ray_{},ray_count_hard_{},config_count_hard_{},random_seed_{};
  int max_special_ik_seeds_{},max_special_points_{};
  std::size_t special_revalidated_{};
  std::array<std::vector<double>,3> axes_;
  Eigen::Vector3d spacing_{Eigen::Vector3d::Zero()},outer_min_{Eigen::Vector3d::Zero()},outer_max_{Eigen::Vector3d::Zero()},origin_{Eigen::Vector3d::Zero()};
  std::vector<TorsoCandidate> c0_candidates_,c3_candidates_;
  std::vector<Ray> rays_;
  std::vector<Record> records_;
};
}  // namespace radial_workspace_validation

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("radial_workspace_validation");
  try { radial_workspace_validation::RadialRunner runner(node); runner.runRadial(); }
  catch (const std::exception& error) { RCLCPP_FATAL(node->get_logger(),"RADIAL_WORKSPACE failed: %s",error.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
