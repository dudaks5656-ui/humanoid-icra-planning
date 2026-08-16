#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include <geometric_shapes/body_operations.h>

// Prime all dependencies in the global namespace.  The preserved adaptive
// implementation is then included in a private namespace so its audited
// PlanningScene/Lift-only validation can be reused without modifying it.
#define main preserved_reference_main_for_axis_ablation
#include "reference_trajectory_generator.cpp"
#undef main
namespace preserved_adaptive
{
#define private public
#include "adaptive_target_boundary_search_v1.cpp"
#undef private
}

namespace torso_axis_ablation_v1
{
constexpr double kPi=3.14159265358979323846;
constexpr double kNan=std::numeric_limits<double>::quiet_NaN();

struct InputTarget { std::string id,ray; double lift{},distance{},x{},y{},z{}; };
struct Row
{
  InputTarget target;
  std::string mode;
  preserved_adaptive::adaptive_target_boundary_search_v1::Result result;
  double arm_min{kNan},yaw_margin{kNan},pitch_margin{kNan},active_min{kNan};
  std::vector<double> normalized;
  std::string classification;
};

std::vector<std::string> csvFields(const std::string& line)
{
  std::vector<std::string> fields; std::string value; bool quoted=false;
  for (std::size_t i=0;i<line.size();++i)
  {
    const char c=line[i];
    if (c=='"') { if (quoted && i+1<line.size() && line[i+1]=='"') { value+='"'; ++i; } else quoted=!quoted; }
    else if (c==',' && !quoted) { fields.push_back(value); value.clear(); }
    else value+=c;
  }
  fields.push_back(value); return fields;
}

std::vector<std::map<std::string,std::string>> readCsv(const std::string& path)
{
  std::ifstream in(path); if (!in) throw std::runtime_error("Cannot read "+path);
  std::string line; std::getline(in,line); const auto header=csvFields(line);
  std::vector<std::map<std::string,std::string>> rows;
  while (std::getline(in,line))
  {
    if (line.empty()) continue; const auto values=csvFields(line);
    std::map<std::string,std::string> row;
    for (std::size_t i=0;i<header.size() && i<values.size();++i) row[header[i]]=values[i];
    rows.push_back(std::move(row));
  }
  return rows;
}

class Ablation
{
public:
  explicit Ablation(const rclcpp::Node::SharedPtr& node):node_(node)
  {
    input_dir_=node_->get_parameter("input_dir").as_string();
    results_csv_=node_->get_parameter("results_csv").as_string();
    summary_csv_=node_->get_parameter("summary_csv").as_string();
    result_yaml_=node_->get_parameter("result_yaml").as_string();
    audit_md_=node_->get_parameter("audit_md").as_string();
    loadTargets();
    runner_=std::make_unique<preserved_adaptive::adaptive_target_boundary_search_v1::Pilot>(node_);
    runner_->locked_multistart_=8;
    runner_->posture_multistart_=8;
    arm_names_=runner_->left_arm_->getVariableNames();
    initialize();
  }

  bool run()
  {
    for (std::size_t i=0;i<targets_.size();++i)
    {
      std::vector<Row> target_rows;
      for (const std::string mode:{"LOCKED","YAW_ONLY","PITCH_ONLY","YAW_PITCH"})
      {
        RCLCPP_INFO(node_->get_logger(),"ABLATION %s mode=%s",targets_[i].id.c_str(),mode.c_str());
        Row row=runMode(targets_[i],mode,static_cast<std::uint64_t>(i+1));
        append(row); rows_.push_back(row); target_rows.push_back(row);
      }
      summarize(target_rows);
    }
    writeYaml(); writeAudit();
    return rows_.size()==120 && summaries_==30;
  }

private:
  void loadTargets()
  {
    const auto boundaries=readCsv(input_dir_+"/ray_boundary_summary.csv");
    const auto sequence=readCsv(input_dir_+"/ray_state_sequence.csv");
    for (const auto& b:boundaries)
    {
      if (b.at("first_yaw_pitch_recovery_distance")=="nan") continue;
      const double first=std::stod(b.at("first_locked_failure_distance"));
      for (int offset_index=0;offset_index<3;++offset_index)
      {
        const double distance=first+0.005*offset_index;
        const auto it=std::find_if(sequence.begin(),sequence.end(),[&](const auto& r){
          return r.at("ray")==b.at("ray") && r.at("lift")==b.at("lift") &&
                 std::abs(std::stod(r.at("distance_m"))-distance)<1e-10; });
        if (it==sequence.end() || it->at("locked_success")!="0" || it->at("yaw_pitch_success")!="1")
          throw std::runtime_error("Stored recovery target is missing or inconsistent");
        InputTarget t;
        t.ray=b.at("ray"); t.lift=std::stod(b.at("lift")); t.distance=distance;
        t.x=std::stod(it->at("target_x")); t.y=std::stod(it->at("target_y")); t.z=std::stod(it->at("target_z"));
        t.id=t.ray+"_L"+(t.lift<0.375?"035":"040")+"_D"+std::to_string(static_cast<int>(std::llround(distance*1000)));
        targets_.push_back(t);
      }
    }
    if (targets_.size()!=30) throw std::runtime_error("Expected exactly 30 stored recovery targets");
  }

  double margin(double value,const std::string& name) const
  {
    const auto& b=runner_->model_->getVariableBounds(name);
    return b.position_bounded_?std::min(value-b.min_position_,b.max_position_-value):kNan;
  }
  double normalized(double value,const std::string& name) const
  {
    const auto& b=runner_->model_->getVariableBounds(name);
    return b.position_bounded_?margin(value,name)/(b.max_position_-b.min_position_):kNan;
  }

  Row runMode(const InputTarget& input,const std::string& mode,std::uint64_t key)
  {
    auto& r=*runner_;
    r.yaw_min_deg_=-10; r.yaw_max_deg_=10; r.pitch_min_deg_=-10; r.pitch_max_deg_=45;
    bool posture=true;
    if (mode=="LOCKED") posture=false;
    else if (mode=="YAW_ONLY") r.pitch_min_deg_=r.pitch_max_deg_=0;
    else if (mode=="PITCH_ONLY") r.yaw_min_deg_=r.yaw_max_deg_=0;
    preserved_adaptive::adaptive_target_boundary_search_v1::Target target;
    target.ray=input.ray; target.distance=input.distance; target.x=input.x; target.y=input.y; target.z=input.z;
    Row row; row.target=input; row.mode=mode; row.result=r.runMode(target,input.lift,posture,key);
    if (row.result.success)
    {
      row.arm_min=std::numeric_limits<double>::infinity();
      for (std::size_t i=0;i<arm_names_.size();++i)
      { row.arm_min=std::min(row.arm_min,margin(row.result.arm[i],arm_names_[i])); row.normalized.push_back(normalized(row.result.arm[i],arm_names_[i])); }
      row.yaw_margin=margin(row.result.yaw,"waist_yaw_joint");
      row.pitch_margin=margin(row.result.pitch,"waist_pitch_joint");
      row.active_min=std::min({row.arm_min,row.yaw_margin,row.pitch_margin});
      row.normalized.push_back(normalized(row.result.yaw,"waist_yaw_joint"));
      row.normalized.push_back(normalized(row.result.pitch,"waist_pitch_joint"));
    }
    else row.normalized.assign(arm_names_.size()+2,kNan);
    if (mode=="LOCKED") row.classification=row.result.success?"LOCKED_UNEXPECTED_SUCCESS":"LOCKED_FAILURE_CONFIRMED";
    else if (row.result.success) row.classification=mode+"_RECOVERY";
    else row.classification="ALL_MODES_INFEASIBLE";
    return row;
  }

  void initialize() const
  {
    std::ofstream out(results_csv_); out<<"target_id,ray,distance_m,target_x,target_y,target_z,lift,mode,success,raw_ik,collision_free_ik,yaw_rad,pitch_rad";
    for (const auto& n:arm_names_) out<<','<<n;
    out<<",arm_joint_1_7_min_margin,joint3_margin,joint5_margin,yaw_margin,pitch_margin,active_joint_min_margin";
    for (const auto& n:arm_names_) out<<",normalized_margin_"<<n;
    out<<",normalized_margin_yaw,normalized_margin_pitch,environment_clearance,self_clearance,lift_extraction_success,descent_m,ascent_m,computation_ms,failure_label,collision_pairs,classification\n";
    std::ofstream summary(summary_csv_); summary<<"target_id,ray,distance_m,target_x,target_y,target_z,lift,locked_success,yaw_only_success,pitch_only_success,yaw_pitch_success,minimum_required_axis,arm_margin_improvement\n";
  }

  void append(const Row& row) const
  {
    const auto& x=row.result; std::ofstream out(results_csv_,std::ios::app);
    out<<std::setprecision(15)<<row.target.id<<','<<row.target.ray<<','<<row.target.distance<<','<<row.target.x<<','<<row.target.y<<','<<row.target.z<<','<<row.target.lift<<','<<row.mode<<','<<x.success<<','<<x.raw_ik<<','<<x.collision_free_ik<<','<<x.yaw<<','<<x.pitch;
    for (std::size_t i=0;i<arm_names_.size();++i) out<<','<<(i<x.arm.size()?x.arm[i]:kNan);
    out<<','<<row.arm_min<<','<<x.joint3_margin<<','<<x.joint5_margin<<','<<row.yaw_margin<<','<<row.pitch_margin<<','<<row.active_min;
    for (double v:row.normalized) out<<','<<v;
    out<<','<<x.environment_clearance<<','<<x.self_clearance<<','<<x.success<<','<<x.descent_distance<<','<<x.ascent_distance<<','<<x.computation_ms<<','<<x.failure_label<<','<<csvEscape(x.collision_pairs)<<','<<row.classification<<'\n';
  }

  void summarize(const std::vector<Row>& rows)
  {
    const bool locked=rows[0].result.success,yaw=rows[1].result.success,pitch=rows[2].result.success,both=rows[3].result.success;
    std::string minimum;
    if (locked) minimum="LOCKED_UNEXPECTED_SUCCESS";
    else if (yaw && pitch) minimum="EITHER_SINGLE_AXIS";
    else if (yaw) minimum="YAW_ONLY_RECOVERY";
    else if (pitch) minimum="PITCH_ONLY_RECOVERY";
    else if (both) minimum="YAW_PITCH_RECOVERY";
    else minimum="ALL_MODES_INFEASIBLE";
    double best_single=kNan;
    for (std::size_t i=1;i<rows.size();++i) if (rows[i].result.success) best_single=std::isfinite(best_single)?std::max(best_single,rows[i].arm_min):rows[i].arm_min;
    std::ofstream out(summary_csv_,std::ios::app); const auto& t=rows[0].target;
    out<<std::setprecision(15)<<t.id<<','<<t.ray<<','<<t.distance<<','<<t.x<<','<<t.y<<','<<t.z<<','<<t.lift<<','<<locked<<','<<yaw<<','<<pitch<<','<<both<<','<<minimum<<','<<best_single<<'\n';
    ++summaries_;
  }

  void writeYaml() const
  {
    int success[4]={0,0,0,0}; for (const auto& row:rows_) for (int i=0;i<4;++i) if (row.mode==std::vector<std::string>{"LOCKED","YAW_ONLY","PITCH_ONLY","YAW_PITCH"}[i] && row.result.success) ++success[i];
    std::ofstream out(result_yaml_); out<<"protocol: TORSO_AXIS_ABLATION_V1\nplanning_only: true\ninput_dir: "<<input_dir_<<"\ntargets: 30\nresults: 120\nmode_success: {LOCKED: "<<success[0]<<", YAW_ONLY: "<<success[1]<<", PITCH_ONLY: "<<success[2]<<", YAW_PITCH: "<<success[3]<<"}\nmove_group_started: false\nompl_started: false\ntrajectory_execution_performed: false\nrviz_started: false\n";
  }
  void writeAudit() const
  {
    std::ofstream out(audit_md_); out<<"# Torso axis ablation v1\n\nGenerated: "<<timestampNow()<<"\n\n- Input targets were read from the preserved adaptive run; no boundary search was repeated.\n- 30 targets x 4 modes = 120 results.\n- Every posture used the same 8 deterministic Arm IK seeds.\n- Arm and selected torso joints remained fixed during 0.17 m Lift-only descent/ascent.\n- No move_group, OMPL, hardware, controller, ros2_control, execution, or RViz was started.\n";
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<preserved_adaptive::adaptive_target_boundary_search_v1::Pilot> runner_;
  std::vector<InputTarget> targets_; std::vector<Row> rows_; std::vector<std::string> arm_names_;
  std::string input_dir_,results_csv_,summary_csv_,result_yaml_,audit_md_; int summaries_{};
};
}

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv); rclcpp::NodeOptions options; options.automatically_declare_parameters_from_overrides(true);
  auto node=std::make_shared<rclcpp::Node>("torso_axis_ablation_v1",options);
  rclcpp::executors::MultiThreadedExecutor executor; executor.add_node(node); std::thread spin([&](){executor.spin();});
  int code=1; std::unique_ptr<torso_axis_ablation_v1::Ablation> experiment;
  try { experiment=std::make_unique<torso_axis_ablation_v1::Ablation>(node); code=experiment->run()?0:2; }
  catch (const std::exception& e) { RCLCPP_ERROR(node->get_logger(),"Axis ablation failed: %s",e.what()); }
  executor.cancel(); if (spin.joinable()) spin.join(); experiment.reset(); node.reset(); rclcpp::shutdown(); return code;
}
