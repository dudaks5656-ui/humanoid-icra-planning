#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>
#include <unistd.h>

// Prime every dependency used by the preserved implementations in the global
// namespace.  Include guards then prevent third-party headers from being
// reopened below paper_reuse (which would incorrectly create paper_reuse::std
// and paper_reuse::boost).
#include <geometric_shapes/body_operations.h>

#define main preserved_reference_main_for_paper_dataset
#include "reference_trajectory_generator.cpp"
#undef main
namespace paper_reuse
{
#define private public
#include "torso_axis_ablation_v1.cpp"
#undef private
}

namespace paper_main_simulation_dataset_v1
{
constexpr double kPi=3.14159265358979323846;
constexpr double kNan=std::numeric_limits<double>::quiet_NaN();
std::atomic_bool stop_requested{false};
void signalHandler(int) { stop_requested=true; }

struct Task { std::string phase,target_id,ray,mode; double angle{},lift{}; int seed_bank{}; };
struct Eval
{
  std::string key,phase,target_id,ray,mode,label,pairs,classification;
  double angle{},distance{},x{},y{},z{},lift{},yaw{kNan},pitch{kNan};
  bool success{}; int seed_bank{},raw{},collision_free{}; std::vector<double> arm;
  double arm_margin{kNan},j3{kNan},j5{kNan},yaw_margin{kNan},pitch_margin{kNan},active{kNan};
  std::vector<double> normalized;
  double environment{kNan},self{kNan},descent{},ascent{},object_clearance{kNan},time_ms{};
};
struct Boundary { Task task; bool last_ok{},first_fail{}; Eval last,fail; };

std::string safe(const std::string& s) { std::string o=s; for(char& c:o) if(!std::isalnum(c)) c='_'; return o; }
std::string dkey(double d) { std::ostringstream s; s<<std::fixed<<std::setprecision(3)<<d; return s.str(); }

class Runner
{
public:
  explicit Runner(const rclcpp::Node::SharedPtr& node):node_(node)
  {
    run_root_=node_->get_parameter("run_root").as_string();
    deadline_epoch_=node_->get_parameter("deadline_epoch").as_int();
    helper_=std::make_unique<paper_reuse::preserved_adaptive::adaptive_target_boundary_search_v1::Pilot>(node_);
    helper_->locked_multistart_=8;
    helper_->posture_multistart_=8;
    model_=helper_->model_;
    arm_names_=helper_->left_arm_->getVariableNames();
    helper_->createReferenceEnvelope();
    loadCache(); initializeFiles(); buildTasks();
  }

  bool run()
  {
    appendHeartbeat("runner_started pid="+std::to_string(::getpid()));
    for (const auto& task:tasks_)
    {
      if (deadlineReached() || stop_requested) break;
      if (completed_tasks_.count(taskKey(task))) continue;
      current_task_=taskKey(task); writeResume("RUNNING");
      try
      {
        Boundary b=runBoundary(task);
        appendBoundary(b); completed_tasks_.insert(taskKey(task));
        appendTaskCompletion(task,"COMPLETED","");
      }
      catch (const std::exception& e)
      {
        if(std::string(e.what())=="SAFE_STOP_DURING_BOUNDARY")
        {
          appendHeartbeat("safe_stop_during_boundary task="+current_task_);
          stop_requested=true;
          break;
        }
        ++consecutive_infra_failures_; appendTaskCompletion(task,"INFRASTRUCTURE_FAILURE",e.what());
        {std::ofstream out(run_root_+"/failed_cases.csv",std::ios::app);out<<csvEscape(taskKey(task))<<','<<csvEscape(e.what())<<'\n';out.flush();}
        appendHeartbeat("infrastructure_failure task="+current_task_+" error="+e.what());
        if (consecutive_infra_failures_>=10) { stop_requested=true; break; }
      }
      if (consecutive_infra_failures_==0 && completed_tasks_.size()%10==0) checkpoint();
    }
    if(!deadlineReached()&&!stop_requested&&completed_tasks_.size()==tasks_.size()) runPhase3();
    writeResume(deadlineReached()?"DEADLINE_REACHED":(stop_requested?"SAFE_STOP":"QUEUE_COMPLETE"));
    checkpoint(); appendHeartbeat("runner_stopped state="+resume_state_);
    return true;
  }

private:
  std::string taskKey(const Task& t) const
  { return t.phase+"|"+t.target_id+"|"+t.ray+"|adaptive|"+dkey(t.lift)+"|"+t.mode+"|"+std::to_string(t.seed_bank); }
  std::string evalKey(const Task& t,double d,const std::string& mode) const
  { return t.phase+"|"+t.ray+"_"+dkey(d)+"|"+t.ray+"|"+dkey(d)+"|"+dkey(t.lift)+"|"+mode+"|"+std::to_string(t.seed_bank); }

  bool deadlineReached() const
  { return std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())>=deadline_epoch_; }

  void buildTasks()
  {
    for(int i=0;i<16;++i)
    {
      const double angle=22.5*i;
      for(double lift:{0.35,0.40}) for(const std::string mode:{"LOCKED","YAW_ONLY","PITCH_ONLY","YAW_PITCH"})
        tasks_.push_back({"PHASE1","P1_R"+std::to_string(i),"R"+std::to_string(i),mode,angle,lift,0});
    }
    for(int i=0;i<8;++i)
    {
      const double angle=45.0*i;
      for(double lift:{0.30,0.45}) for(const std::string mode:{"LOCKED","YAW_ONLY","PITCH_ONLY","YAW_PITCH"})
        tasks_.push_back({"PHASE2","P2_R"+std::to_string(i),"R"+std::to_string(2*i),mode,angle,lift,0});
    }
  }

  paper_reuse::torso_axis_ablation_v1::InputTarget inputTarget(const Task& task,double distance) const
  {
    paper_reuse::torso_axis_ablation_v1::InputTarget t;
    const double a=task.angle*kPi/180.0;
    t.id=task.phase+"_"+task.ray+"_D"+std::to_string(static_cast<int>(std::llround(distance*1000)));
    t.ray=task.ray; t.lift=task.lift; t.distance=distance;
    t.x=helper_->scene_config_.box_center[0]+std::cos(a)*distance;
    t.y=helper_->scene_config_.box_center[1]+std::sin(a)*distance;
    t.z=helper_->scene_config_.target_position[2]; return t;
  }

  double maxDistance(const Task& task) const
  {
    paper_reuse::preserved_adaptive::adaptive_target_boundary_search_v1::Ray ray{task.ray,std::cos(task.angle*kPi/180),std::sin(task.angle*kPi/180)};
    return helper_->maximumDistance(ray);
  }

  double margin(double value,const std::string& name) const
  {
    const auto& b=model_->getVariableBounds(name);
    return b.position_bounded_?std::min(value-b.min_position_,b.max_position_-value):kNan;
  }
  double normalized(double value,const std::string& name) const
  {
    const auto& b=model_->getVariableBounds(name);
    return b.position_bounded_?margin(value,name)/(b.max_position_-b.min_position_):kNan;
  }

  paper_reuse::torso_axis_ablation_v1::Row runMode(
    const paper_reuse::torso_axis_ablation_v1::InputTarget& input,
    const std::string& mode,std::uint64_t key)
  {
    helper_->yaw_min_deg_=-10; helper_->yaw_max_deg_=10;
    helper_->pitch_min_deg_=-10; helper_->pitch_max_deg_=45;
    bool posture=true;
    if(mode=="LOCKED") posture=false;
    else if(mode=="YAW_ONLY") helper_->pitch_min_deg_=helper_->pitch_max_deg_=0;
    else if(mode=="PITCH_ONLY") helper_->yaw_min_deg_=helper_->yaw_max_deg_=0;
    paper_reuse::preserved_adaptive::adaptive_target_boundary_search_v1::Target target;
    target.ray=input.ray; target.distance=input.distance; target.x=input.x; target.y=input.y; target.z=input.z;
    paper_reuse::torso_axis_ablation_v1::Row row; row.target=input; row.mode=mode;
    row.result=helper_->runMode(target,input.lift,posture,key);
    if(row.result.success)
    {
      row.arm_min=std::numeric_limits<double>::infinity();
      for(std::size_t i=0;i<arm_names_.size();++i)
      {
        row.arm_min=std::min(row.arm_min,margin(row.result.arm[i],arm_names_[i]));
        row.normalized.push_back(normalized(row.result.arm[i],arm_names_[i]));
      }
      row.yaw_margin=margin(row.result.yaw,"waist_yaw_joint");
      row.pitch_margin=margin(row.result.pitch,"waist_pitch_joint");
      row.active_min=std::min({row.arm_min,row.yaw_margin,row.pitch_margin});
      row.normalized.push_back(normalized(row.result.yaw,"waist_yaw_joint"));
      row.normalized.push_back(normalized(row.result.pitch,"waist_pitch_joint"));
    }
    else row.normalized.assign(arm_names_.size()+2,kNan);
    return row;
  }

  bool better(const Eval& a,const Eval& b) const
  {
    if(a.success!=b.success) return a.success;
    if(a.arm_margin!=b.arm_margin) return a.arm_margin>b.arm_margin;
    const double aj=std::min(a.j3,a.j5),bj=std::min(b.j3,b.j5); if(aj!=bj) return aj>bj;
    if(a.environment!=b.environment) return a.environment>b.environment;
    if(a.self!=b.self) return a.self>b.self;
    return std::abs(a.yaw)+std::abs(a.pitch)<std::abs(b.yaw)+std::abs(b.pitch);
  }

  Eval convert(const Task& task,double distance,const std::string& mode,
               const paper_reuse::torso_axis_ablation_v1::Row& row) const
  {
    Eval e; e.key=evalKey(task,distance,mode); e.phase=task.phase; e.target_id=row.target.id; e.ray=task.ray;
    e.mode=mode; e.seed_bank=task.seed_bank; e.angle=task.angle; e.distance=distance; e.x=row.target.x; e.y=row.target.y; e.z=row.target.z;
    e.lift=task.lift; e.success=row.result.success; e.raw=row.result.raw_ik; e.collision_free=row.result.collision_free_ik;
    e.yaw=row.result.yaw; e.pitch=row.result.pitch; e.arm=row.result.arm; e.arm_margin=row.arm_min;
    e.j3=row.result.joint3_margin; e.j5=row.result.joint5_margin; e.yaw_margin=row.yaw_margin;
    e.pitch_margin=row.pitch_margin; e.active=row.active_min; e.normalized=row.normalized;
    e.environment=row.result.environment_clearance; e.self=row.result.self_clearance;
    e.descent=row.result.descent_distance; e.ascent=row.result.ascent_distance;
    e.object_clearance=row.result.object_clearance; e.time_ms=row.result.computation_ms;
    e.label=row.result.failure_label; e.pairs=row.result.collision_pairs;
    if(e.success) e.classification="NOMINAL_FEASIBLE";
    else if(e.label=="GRASP_CONFIGURATION_IK_FAILURE") e.classification="PURE_IK_ABSENCE";
    else if(e.label=="GRASP_CONFIGURATION_COLLISION_FAILURE") e.classification="GRASP_CONFIGURATION_COLLISION";
    else if(e.label=="LIFT_DESCENT_COLLISION_FAILURE") e.classification="LIFT_DESCENT_COLLISION";
    else if(e.label=="LIFT_ASCENT_COLLISION_FAILURE") e.classification="LIFT_ASCENT_COLLISION";
    else e.classification=e.label;
    return e;
  }

  Eval physicalFailure(const Task& task,double distance,const paper_reuse::preserved_adaptive::adaptive_target_boundary_search_v1::Target& target) const
  {
    Eval e; e.key=evalKey(task,distance,task.mode); e.phase=task.phase; e.target_id=task.phase+"_"+task.ray+"_D"+dkey(distance);
    e.ray=task.ray; e.mode=task.mode; e.seed_bank=task.seed_bank; e.angle=task.angle; e.distance=distance; e.x=target.x;e.y=target.y;e.z=target.z;e.lift=task.lift;
    e.label=target.gripper_envelope_infeasible?"GRIPPER_ENVELOPE_INFEASIBLE":"OBJECT_WALL_OVERLAP";
    e.classification=target.gripper_envelope_infeasible?"GRIPPER_ENVELOPE_INFEASIBLE":"ALL_MODES_INFEASIBLE"; e.pairs=target.physical_pair; return e;
  }

  Eval evaluateRaw(const Task& task,double distance,const std::string& mode,bool save=true)
  {
    const std::string key=evalKey(task,distance,mode);
    if(save){auto found=cache_.find(key); if(found!=cache_.end()) return found->second;}
    paper_reuse::preserved_adaptive::adaptive_target_boundary_search_v1::Ray ray{task.ray,std::cos(task.angle*kPi/180),std::sin(task.angle*kPi/180)};
    const auto physical=helper_->makeTarget(ray,distance);
    Eval e;
    if(physical.object_wall_overlap||physical.gripper_envelope_infeasible) e=physicalFailure(task,distance,physical),e.mode=mode,e.key=key;
    else
    {
      auto input=inputTarget(task,distance);
      auto row=runMode(input,mode,std::hash<std::string>{}(task.phase+task.ray+dkey(distance)+std::to_string(task.seed_bank)));
      e=convert(task,distance,mode,row);
    }
    if(save) persist(e); return e;
  }

  Eval evaluate(const Task& task,double distance)
  {
    if(task.mode!="YAW_PITCH") return evaluateRaw(task,distance,task.mode);
    const std::string key=evalKey(task,distance,"YAW_PITCH");
    auto found=cache_.find(key); if(found!=cache_.end()) return found->second;
    Eval yaw=evaluateRaw(task,distance,"YAW_ONLY"); Eval pitch=evaluateRaw(task,distance,"PITCH_ONLY");
    // The native two-axis search is evaluated without persistence.  Only the
    // dominance-corrected union winner is assigned the YAW_PITCH unique key.
    Eval combined=evaluateRaw(task,distance,"YAW_PITCH",false); Eval selected=combined;
    if(better(yaw,selected)) selected=yaw; if(better(pitch,selected)) selected=pitch;
    selected.key=evalKey(task,distance,"YAW_PITCH"); selected.mode="YAW_PITCH";
    selected.raw=combined.raw+yaw.raw+pitch.raw;
    selected.collision_free=combined.collision_free+yaw.collision_free+pitch.collision_free;
    selected.time_ms=combined.time_ms+yaw.time_ms+pitch.time_ms;
    if(selected.success && selected.arm_margin+1e-12<std::max(yaw.arm_margin,pitch.arm_margin))
      throw std::runtime_error("YAW_PITCH dominance invariant violated");
    persist(selected); return selected;
  }

  Boundary runBoundary(const Task& task)
  {
    Boundary b; b.task=task; const double maximum=maxDistance(task); double last=-1,fail=-1;
    for(double d=0;d<=maximum+1e-9;d+=0.010)
    {
      d=std::min(d,maximum); Eval e=evaluate(task,d);
      if(e.success) last=d; else {fail=d;break;} if(maximum-d<1e-9) break;
      if(d+0.010>maximum) d=maximum-0.010;
      if(deadlineReached()||stop_requested) throw std::runtime_error("SAFE_STOP_DURING_BOUNDARY");
    }
    if(last>=0&&fail>=0)
    {
      for(double d=last+0.002;d<fail-1e-9;d+=0.002) { Eval e=evaluate(task,d); if(e.success) last=d; else {fail=d;break;} }
      for(double d=last+0.001;d<fail-1e-9;d+=0.001) { Eval e=evaluate(task,d); if(e.success) last=d; else {fail=d;break;} }
    }
    if(last>=0){b.last_ok=true;b.last=evaluate(task,last);} if(fail>=0){b.first_fail=true;b.fail=evaluate(task,fail);} return b;
  }

  void runPhase3()
  {
    const std::string complete_marker=run_root_+"/summaries/phase3_complete.txt";
    if(std::filesystem::exists(complete_marker)) return;
    struct BRow {std::string ray,mode,label;double angle{},lift{},last{kNan},fail{kNan};};
    std::vector<BRow> bounds;
    for(const auto& r:paper_reuse::torso_axis_ablation_v1::readCsv(run_root_+"/summaries/mode_workspace_boundary.csv"))
    {
      if(r.at("phase")!="PHASE1") continue;
      BRow b;b.ray=r.at("ray");b.mode=r.at("mode");b.label=r.at("failure_label");
      b.angle=std::stod(r.at("ray_angle_deg"));b.lift=std::stod(r.at("lift"));
      b.last=std::stod(r.at("last_success_distance"));b.fail=std::stod(r.at("first_failure_distance"));bounds.push_back(b);
    }
    std::map<std::tuple<std::string,double,std::string>,BRow> by;
    for(const auto& b:bounds)by[{b.ray,b.lift,b.mode}]=b;
    struct Rep{std::string id,ray;double angle{},lift{},distance{};};std::vector<Rep> reps;
    reps.push_back({"COMMON_CENTER","R0",0,0.35,0});
    for(double lift:{0.35,0.40})for(int i=0;i<16;++i)
    {
      const std::string ray="R"+std::to_string(i);const auto lk=by.find({ray,lift,"LOCKED"});
      if(lk==by.end()||!std::isfinite(lk->second.last))continue;
      if(reps.size()==1)reps.push_back({"LOCKED_LAST_SUCCESS",ray,lk->second.angle,lift,lk->second.last});
      const auto y=by.find({ray,lift,"YAW_ONLY"}),p=by.find({ray,lift,"PITCH_ONLY"}),yp=by.find({ray,lift,"YAW_PITCH"});
      if(y!=by.end()&&y->second.last>lk->second.last+1e-9&&std::none_of(reps.begin(),reps.end(),[](const Rep& r){return r.id=="YAW_ONLY_FIRST_RECOVERY";}))
        reps.push_back({"YAW_ONLY_FIRST_RECOVERY",ray,lk->second.angle,lift,lk->second.fail});
      if(p!=by.end()&&p->second.last>lk->second.last+1e-9&&std::none_of(reps.begin(),reps.end(),[](const Rep& r){return r.id=="PITCH_ONLY_FIRST_RECOVERY";}))
        reps.push_back({"PITCH_ONLY_FIRST_RECOVERY",ray,lk->second.angle,lift,lk->second.fail});
      if(y!=by.end()&&p!=by.end()&&yp!=by.end()&&yp->second.last>std::max(y->second.last,p->second.last)+1e-9&&
         std::none_of(reps.begin(),reps.end(),[](const Rep& r){return r.id=="YAW_PITCH_SYNERGY";}))
        reps.push_back({"YAW_PITCH_SYNERGY",ray,lk->second.angle,lift,std::max(y->second.fail,p->second.fail)});
      if(std::isfinite(lk->second.fail)&&y!=by.end()&&p!=by.end()&&yp!=by.end()&&
         std::isfinite(y->second.fail)&&std::isfinite(p->second.fail)&&std::isfinite(yp->second.fail)&&
         std::none_of(reps.begin(),reps.end(),[](const Rep& r){return r.id=="COMMON_FAILURE";}))
        reps.push_back({"COMMON_FAILURE",ray,lk->second.angle,lift,std::max({lk->second.fail,y->second.fail,p->second.fail,yp->second.fail})});
      for(const auto* q:{&lk->second,y==by.end()?nullptr:&y->second,p==by.end()?nullptr:&p->second,yp==by.end()?nullptr:&yp->second})
        if(q&&q->label=="GRIPPER_ENVELOPE_INFEASIBLE"&&std::none_of(reps.begin(),reps.end(),[](const Rep& r){return r.id=="GRIPPER_ENVELOPE_FAILURE";}))
          reps.push_back({"GRIPPER_ENVELOPE_FAILURE",ray,lk->second.angle,lift,q->fail});
    }
    const std::string rep_path=run_root_+"/summaries/phase3_representatives.csv";
    if(!std::filesystem::exists(rep_path)){std::ofstream o(rep_path);o<<"representative_id,ray,ray_angle_deg,lift,distance_m\n";for(const auto& r:reps)o<<r.id<<','<<r.ray<<','<<r.angle<<','<<r.lift<<','<<r.distance<<'\n';}
    for(const auto& rep:reps)for(int bank=1;bank<=5;++bank)for(const std::string mode:{"LOCKED","YAW_ONLY","PITCH_ONLY","YAW_PITCH"})
    {
      if(deadlineReached()||stop_requested)return;
      Task t{"PHASE3",rep.id,rep.ray,mode,rep.angle,rep.lift,bank};
      (void)evaluate(t,rep.distance);
    }
    {std::ofstream o(complete_marker);o<<timestampNow()<<" representatives="<<reps.size()<<" seed_banks=5\n";}
    appendHeartbeat("phase3_complete representatives="+std::to_string(reps.size()));
  }

  std::string header() const
  {
    std::ostringstream o; o<<"unique_key,phase,target_id,ray,ray_angle_deg,distance_m,target_x,target_y,target_z,lift,mode,seed_bank,success,raw_ik,collision_free_ik,yaw_rad,pitch_rad";
    for(const auto& n:arm_names_)o<<','<<n;
    o<<",arm_joint_1_7_min_margin,joint3_margin,joint5_margin,yaw_margin,pitch_margin,active_joint_min_margin";
    for(const auto& n:arm_names_)o<<",normalized_margin_"<<n;
    o<<",normalized_margin_yaw,normalized_margin_pitch,environment_clearance,self_clearance,lift_descent_m,lift_ascent_m,arm_max_delta,tcp_position_error,tcp_orientation_error,object_box_top_clearance,computation_ms,success_stage,failure_label,collision_pairs,classification\n";return o.str();
  }
  std::string line(const Eval& e) const
  {
    std::ostringstream o; o<<std::setprecision(15)<<csvEscape(e.key)<<','<<e.phase<<','<<e.target_id<<','<<e.ray<<','<<e.angle<<','<<e.distance<<','<<e.x<<','<<e.y<<','<<e.z<<','<<e.lift<<','<<e.mode<<','<<e.seed_bank<<','<<e.success<<','<<e.raw<<','<<e.collision_free<<','<<e.yaw<<','<<e.pitch;
    for(std::size_t i=0;i<arm_names_.size();++i)o<<','<<(i<e.arm.size()?e.arm[i]:kNan);
    o<<','<<e.arm_margin<<','<<e.j3<<','<<e.j5<<','<<e.yaw_margin<<','<<e.pitch_margin<<','<<e.active;
    for(std::size_t i=0;i<arm_names_.size()+2;++i)o<<','<<(i<e.normalized.size()?e.normalized[i]:kNan);
    o<<','<<e.environment<<','<<e.self<<','<<e.descent<<','<<e.ascent<<",0,0,0,"<<e.object_clearance<<','<<e.time_ms<<','<<(e.success?"LIFT_EXTRACTION_COMPLETE":"")<<','<<e.label<<','<<csvEscape(e.pairs)<<','<<e.classification<<'\n'; return o.str();
  }

  void persist(const Eval& e)
  {
    const std::string raw=run_root_+"/raw/"+safe(e.key)+".csv",tmp=raw+".tmp";
    {std::ofstream out(tmp);out<<header()<<line(e);out.flush();} std::filesystem::rename(tmp,raw);
    {std::ofstream out(run_root_+"/all_case_results.csv",std::ios::app);out<<line(e);out.flush();}
    {std::ofstream out(run_root_+"/completed_cases.csv",std::ios::app);out<<csvEscape(e.key)<<','<<e.phase<<','<<e.target_id<<','<<e.ray<<','<<e.distance<<','<<e.lift<<','<<e.mode<<','<<e.seed_bank<<'\n';out.flush();}
    cache_[e.key]=e; ++new_cases_; if(new_cases_%10==0) checkpoint();
  }
  void replacePersisted(const Eval& e)
  {
    auto it=cache_.find(e.key); if(it==cache_.end()) persist(e); else { it->second=e; const std::string raw=run_root_+"/raw/"+safe(e.key)+".csv",tmp=raw+".tmp"; {std::ofstream out(tmp);out<<header()<<line(e);} std::filesystem::rename(tmp,raw); }
  }

  void appendBoundary(const Boundary& b)
  {
    std::ofstream out(run_root_+"/summaries/mode_workspace_boundary.csv",std::ios::app);
    out<<std::setprecision(15)<<b.task.phase<<','<<b.task.ray<<','<<b.task.angle<<','<<b.task.lift<<','<<b.task.mode<<','
       <<(b.last_ok?b.last.distance:kNan)<<','<<(b.first_fail?b.fail.distance:kNan)<<','<<(b.last_ok?b.last.arm_margin:kNan)<<','
       <<(b.first_fail?b.fail.label:"")<<','<<(b.first_fail?csvEscape(b.fail.pairs):"")<<'\n';out.flush();
    consecutive_infra_failures_=0;
  }
  void appendTaskCompletion(const Task& t,const std::string& state,const std::string& error)
  { std::ofstream out(run_root_+"/logs/task_status.csv",std::ios::app);out<<csvEscape(taskKey(t))<<','<<state<<','<<csvEscape(error)<<'\n'; }
  void appendHeartbeat(const std::string& message) const
  { std::ofstream out(run_root_+"/heartbeat.log",std::ios::app);out<<timestampNow()<<' '<<message<<'\n'; }

  void initializeFiles()
  {
    if(!std::filesystem::exists(run_root_+"/all_case_results.csv")){std::ofstream o(run_root_+"/all_case_results.csv");o<<header();}
    if(!std::filesystem::exists(run_root_+"/completed_cases.csv")){std::ofstream o(run_root_+"/completed_cases.csv");o<<"unique_key,phase,target_id,ray,distance,lift,mode,seed_bank\n";}
    if(!std::filesystem::exists(run_root_+"/failed_cases.csv")){std::ofstream o(run_root_+"/failed_cases.csv");o<<"unique_key,error\n";}
    const std::string boundary=run_root_+"/summaries/mode_workspace_boundary.csv";
    if(!std::filesystem::exists(boundary)){std::ofstream o(boundary);o<<"phase,ray,ray_angle_deg,lift,mode,last_success_distance,first_failure_distance,last_success_arm_margin,failure_label,collision_pairs\n";}
    const std::string status=run_root_+"/logs/task_status.csv"; if(!std::filesystem::exists(status)){std::ofstream o(status);o<<"task_key,state,error\n";}
  }
  void loadCache()
  {
    const std::string status=run_root_+"/logs/task_status.csv"; std::ifstream in(status); std::string line;
    if(in)
    {
      std::getline(in,line);
      while(std::getline(in,line))
      {
        const auto fields=paper_reuse::torso_axis_ablation_v1::csvFields(line);
        if(fields.size()>=2&&fields[1]=="COMPLETED")completed_tasks_.insert(fields[0]);
      }
    }
    const std::filesystem::path raw_dir=std::filesystem::path(run_root_)/"raw";
    if(!std::filesystem::exists(raw_dir)) return;
    for(const auto& entry:std::filesystem::directory_iterator(raw_dir))
    {
      if(!entry.is_regular_file()||entry.path().extension()!=".csv"||entry.path().filename()=="gripper_swept_envelope.csv") continue;
      try
      {
        const auto rows=paper_reuse::torso_axis_ablation_v1::readCsv(entry.path().string());
        if(rows.size()!=1||!rows[0].count("unique_key")) continue;
        const auto& r=rows[0]; Eval e;
        auto d=[&](const std::string& n){return std::stod(r.at(n));};
        e.key=r.at("unique_key");e.phase=r.at("phase");e.target_id=r.at("target_id");e.ray=r.at("ray");e.mode=r.at("mode");e.seed_bank=std::stoi(r.at("seed_bank"));
        e.angle=d("ray_angle_deg");e.distance=d("distance_m");e.x=d("target_x");e.y=d("target_y");e.z=d("target_z");e.lift=d("lift");
        e.success=r.at("success")=="1";e.raw=std::stoi(r.at("raw_ik"));e.collision_free=std::stoi(r.at("collision_free_ik"));
        e.yaw=d("yaw_rad");e.pitch=d("pitch_rad");
        for(const auto& n:arm_names_)e.arm.push_back(d(n));
        e.arm_margin=d("arm_joint_1_7_min_margin");e.j3=d("joint3_margin");e.j5=d("joint5_margin");
        e.yaw_margin=d("yaw_margin");e.pitch_margin=d("pitch_margin");e.active=d("active_joint_min_margin");
        for(const auto& n:arm_names_)e.normalized.push_back(d("normalized_margin_"+n));
        e.normalized.push_back(d("normalized_margin_yaw"));e.normalized.push_back(d("normalized_margin_pitch"));
        e.environment=d("environment_clearance");e.self=d("self_clearance");e.descent=d("lift_descent_m");e.ascent=d("lift_ascent_m");
        e.object_clearance=d("object_box_top_clearance");e.time_ms=d("computation_ms");e.label=r.at("failure_label");
        e.pairs=r.at("collision_pairs");e.classification=r.at("classification");cache_[e.key]=std::move(e);
      }
      catch(const std::exception& e)
      { appendHeartbeat("raw_cache_parse_warning file="+entry.path().string()+" error="+e.what()); }
    }
  }
  void checkpoint()
  {
    const std::string tmp=run_root_+"/progress.json.tmp";
    {std::ofstream o(tmp);o<<"{\n  \"updated\": \""<<timestampNow()<<"\",\n  \"completed_boundary_tasks\": "<<completed_tasks_.size()<<",\n  \"total_boundary_tasks\": "<<tasks_.size()<<",\n  \"new_cases\": "<<new_cases_<<",\n  \"current_task\": \""<<current_task_<<"\"\n}\n";} std::filesystem::rename(tmp,run_root_+"/progress.json");
    appendHeartbeat("checkpoint completed_tasks="+std::to_string(completed_tasks_.size())+" cases="+std::to_string(new_cases_));
  }
  void writeResume(const std::string& state)
  {
    resume_state_=state; const std::string tmp=run_root_+"/resume_state.yaml.tmp";
    {std::ofstream o(tmp);o<<"state: "<<state<<"\nupdated: "<<timestampNow()<<"\ncurrent_task: \""<<current_task_<<"\"\ncompleted_boundary_tasks: "<<completed_tasks_.size()<<"\ntotal_boundary_tasks: "<<tasks_.size()<<"\n";} std::filesystem::rename(tmp,run_root_+"/resume_state.yaml");
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<paper_reuse::preserved_adaptive::adaptive_target_boundary_search_v1::Pilot> helper_;
  moveit::core::RobotModelConstPtr model_; std::vector<std::string> arm_names_; std::vector<Task> tasks_;
  std::map<std::string,Eval> cache_; std::set<std::string> completed_tasks_; std::string run_root_,current_task_,resume_state_;
  std::int64_t deadline_epoch_{}; std::size_t new_cases_{}; int consecutive_infra_failures_{};
};
}

int main(int argc,char** argv)
{
  std::signal(SIGINT,paper_main_simulation_dataset_v1::signalHandler); std::signal(SIGTERM,paper_main_simulation_dataset_v1::signalHandler);
  rclcpp::init(argc,argv);rclcpp::NodeOptions options;options.automatically_declare_parameters_from_overrides(true);
  auto node=std::make_shared<rclcpp::Node>("paper_main_simulation_dataset_v1",options);
  rclcpp::executors::MultiThreadedExecutor executor;executor.add_node(node);std::thread spin([&](){executor.spin();});int code=1;
  std::unique_ptr<paper_main_simulation_dataset_v1::Runner> runner;
  try{runner=std::make_unique<paper_main_simulation_dataset_v1::Runner>(node);code=runner->run()?0:2;}catch(const std::exception& e){RCLCPP_ERROR(node->get_logger(),"Paper dataset runner failed: %s",e.what());}
  executor.cancel();if(spin.joinable())spin.join();runner.reset();node.reset();rclcpp::shutdown();return code;
}
