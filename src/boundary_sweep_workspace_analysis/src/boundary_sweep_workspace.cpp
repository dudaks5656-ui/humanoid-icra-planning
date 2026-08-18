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

namespace {
using Clock=std::chrono::steady_clock;
enum class Config{C0,C1,C2,C3};
enum class View{FRONT,RIGHT};
enum class Boundary{INNER,OUTER};
std::string cname(Config c){return c==Config::C0?"LIFT_ONLY":c==Config::C1?"LIFT_YAW":c==Config::C2?"LIFT_PITCH":"LIFT_YAW_PITCH";}
std::string vname(View v){return v==View::FRONT?"FRONT":"RIGHT";}
std::string bname(Boundary b){return b==Boundary::INNER?"INNER_BOUNDARY":"OUTER_BOUNDARY";}
double halton(std::uint64_t i,unsigned base){double f=1,v=0;while(i){f/=base;v+=f*(i%base);i/=base;}return v;}
std::string num(double v){if(!std::isfinite(v))return "";std::ostringstream o;o<<std::setprecision(15)<<v;return o.str();}
double wrap(double a){while(a>M_PI)a-=2*M_PI;while(a<-M_PI)a+=2*M_PI;return a;}

struct Eval{bool valid{false};bool collision{false};double margin{NAN},clearance{NAN},score{NAN};Eigen::Vector3d tcp{NAN,NAN,NAN};};
struct Counts{std::size_t checks{0},collisions{0},bounds{0},valid{0};};

class Runner{
public:
 explicit Runner(rclcpp::Node::SharedPtr n):node_(std::move(n)){
  params();loader_=std::make_shared<robot_model_loader::RobotModelLoader>(node_,"robot_description",true);model_=loader_->getModel();if(!model_)throw std::runtime_error("RobotModel load failed");
  arm_=group(arm_group_);full_=group(full_group_);base_=link(base_frame_);tcp_=link(tcp_frame_);reference_=link(reference_link_);scene_=std::make_shared<planning_scene::PlanningScene>(model_);
  names_={"lift_joint","waist_yaw_joint","waist_pitch_joint"};auto a=arm_->getVariableNames();names_.insert(names_.end(),a.begin(),a.end());if(a.size()!=7||names_!=full_->getVariableNames())throw std::runtime_error("Unexpected joint contract");
  sample_names_={"waist_yaw_joint","waist_pitch_joint","openarm_left_joint1","openarm_left_joint2","openarm_left_joint3","openarm_left_joint4","openarm_left_joint5","openarm_left_joint6","openarm_left_joint7"};
  primes_={2,3,5,7,11,13,17,19,23};
 }
 void run(){preflight();start_=Clock::now();open();scanLift();buildSlices();for(auto c:{Config::C0,Config::C1,Config::C2,Config::C3})for(double lift:slices_)for(auto v:{View::FRONT,View::RIGHT})sweep(c,lift,v);writeMetadata();RCLCPP_INFO(node_->get_logger(),"BOUNDARY_SWEEP COMPLETE checks=%zu collisions=%zu",counts_.checks,counts_.collisions);}
private:
 template<class T>T p(const std::string& k){if(!node_->has_parameter(k))node_->declare_parameter<T>(k);return node_->get_parameter(k).get_value<T>();}
 void params(){out_=p<std::string>("output_dir");base_frame_=p<std::string>("base_frame");tcp_frame_=p<std::string>("tcp_frame");reference_link_=p<std::string>("reference_link");arm_group_=p<std::string>("arm_group");full_group_=p<std::string>("full_group");lift_step_=p<double>("usable_lift_step_m");usable_reps_=p<int>("usable_representative_states");seed_attempts_=p<int>("seed_pool_attempts");seed_count_=p<int>("optimizer_seed_count");angle_step_=p<double>("support_angle_step_deg");coarse_fractions_=p<std::vector<double>>("coarse_line_fractions");coarse_passes_=p<int>("coarse_passes");refine_deg_=p<double>("refine_step_deg");final_deg_=p<double>("final_refine_step_deg");epsilon_=p<double>("exact_bound_epsilon");inner_penalty_=p<double>("inner_angle_penalty_m_per_rad");max_checks_=p<int>("max_total_collision_checks");max_wall_=p<double>("max_wall_time_s");progress_=p<int>("progress_every_directions");}
 const moveit::core::JointModelGroup* group(const std::string& n){auto g=model_->getJointModelGroup(n);if(!g)throw std::runtime_error("Missing group "+n);return g;}
 const moveit::core::LinkModel* link(const std::string& n){auto l=model_->getLinkModel(n);if(!l)throw std::runtime_error("Missing link "+n);return l;}
 bool yaw(Config c)const{return c==Config::C1||c==Config::C3;}bool pitch(Config c)const{return c==Config::C2||c==Config::C3;}
 std::vector<std::string> active(Config c)const{std::vector<std::string> a;if(yaw(c))a.push_back("waist_yaw_joint");if(pitch(c))a.push_back("waist_pitch_joint");auto arm=arm_->getVariableNames();a.insert(a.end(),arm.begin(),arm.end());return a;}
 moveit::core::RobotState nominal()const{moveit::core::RobotState s(model_);s.setToDefaultValues();for(const std::string n:{"openarm_left_finger_joint1","openarm_right_finger_joint1"}){auto b=model_->getVariableBounds(n);if(b.position_bounded_)s.setVariablePosition(n,.5*(b.min_position_+b.max_position_));}s.update();return s;}
 void setSeed(moveit::core::RobotState& s,Config c,double lift,int seed)const{s.setVariablePosition("lift_joint",lift);std::uint64_t index=20260819u+seed+1;for(std::size_t i=0;i<sample_names_.size();++i){auto& n=sample_names_[i];auto b=model_->getVariableBounds(n);double q=b.min_position_+halton(index,primes_[i])*(b.max_position_-b.min_position_);if(n=="waist_yaw_joint"&&!yaw(c))q=0;if(n=="waist_pitch_joint"&&!pitch(c))q=0;s.setVariablePosition(n,q);}s.update();}
 Eigen::Vector3d rel(const moveit::core::RobotState&s,const moveit::core::LinkModel*l)const{return s.getGlobalLinkTransform(base_).inverse()*s.getGlobalLinkTransform(l).translation();}
 double margin(const moveit::core::RobotState&s,const std::vector<std::string>& a)const{double m=std::numeric_limits<double>::infinity();for(auto& n:a){auto b=model_->getVariableBounds(n);double q=s.getVariablePosition(n);m=std::min(m,std::min(q-b.min_position_,b.max_position_-q));}return m;}
 Eval evaluate(moveit::core::RobotState&s,Config c,View view,Boundary boundary,double angle,const Eigen::Vector3d& ref){if(++counts_.checks>static_cast<std::size_t>(max_checks_))throw std::runtime_error("Collision-check hard cap exceeded");if(std::chrono::duration<double>(Clock::now()-start_).count()>max_wall_)throw std::runtime_error("Wall-time cap exceeded");Eval e;e.margin=margin(s,active(c));if(!s.satisfiesBounds()||!(e.margin>epsilon_)){++counts_.bounds;return e;}collision_detection::CollisionRequest req;collision_detection::CollisionResult res;scene_->checkSelfCollision(req,res,s);if(res.collision){e.collision=true;++counts_.collisions;return e;}e.tcp=rel(s,tcp_);if(!e.tcp.allFinite())return e;e.clearance=scene_->getCollisionEnv()->distanceSelf(s,scene_->getAllowedCollisionMatrix());double u=view==View::FRONT?e.tcp.y()-ref.y():e.tcp.x()-ref.x(),z=e.tcp.z()-ref.z();double r=std::hypot(u,z),a=std::atan2(z,u);e.score=boundary==Boundary::OUTER?u*std::cos(angle)+z*std::sin(angle):-(r+inner_penalty_*std::abs(wrap(a-angle)));e.valid=true;++counts_.valid;return e;}
 bool validOnly(moveit::core::RobotState&s,Config c){Eigen::Vector3d ref=rel(s,reference_);return evaluate(s,c,View::RIGHT,Boundary::OUTER,0,ref).valid;}
 void preflight(){if(lift_step_<=0||lift_step_>.05||usable_reps_<3||seed_attempts_>32||seed_count_>5||angle_step_<10||angle_step_>30||max_checks_>250000)throw std::runtime_error("Boundary sweep hard limit violation");std::filesystem::create_directories(out_);for(auto f:{"boundary_sweep_workspace_states.csv","boundary_sweep_workspace_lift_scan.csv","boundary_sweep_workspace_sampling_metadata.csv"})if(std::filesystem::exists(std::filesystem::path(out_)/f))throw std::runtime_error(std::string("Refusing overwrite ")+f);RCLCPP_INFO(node_->get_logger(),"BOUNDARY_SWEEP PREFLIGHT directed_coordinate_search=yes random_sampling=no IK=no OMPL=no max_checks=%d",max_checks_);}
 void open(){states_.open(std::filesystem::path(out_)/"boundary_sweep_workspace_states.csv");lift_.open(std::filesystem::path(out_)/"boundary_sweep_workspace_lift_scan.csv");states_<<"configuration,view,boundary_type,lift_value,sweep_parameter,valid,self_collision,tcp_x,tcp_y,tcp_z,joint_margin,self_clearance,seed_index,optimization_stage,joint_names,joint_values\n";lift_<<"lift_value,representative_states_tested,valid_representative_count,self_collision_count,usable\n";}
 std::string jnames()const{std::ostringstream o;for(size_t i=0;i<names_.size();++i){if(i)o<<';';o<<names_[i];}return o.str();}
 std::string jvalues(const moveit::core::RobotState&s)const{std::ostringstream o;o<<std::setprecision(15);for(size_t i=0;i<names_.size();++i){if(i)o<<';';o<<s.getVariablePosition(names_[i]);}return o.str();}
 void scanLift(){auto b=model_->getVariableBounds("lift_joint");std::vector<double> q;for(double x=b.min_position_;x<b.max_position_-1e-9;x+=lift_step_)q.push_back(x);q.push_back(b.max_position_);bool any=false;for(double x:q){int valid=0,collision=0;for(int seed=0;seed<usable_reps_;++seed){auto s=nominal();setSeed(s,Config::C0,x,seed);auto before=counts_.collisions;if(validOnly(s,Config::C0))++valid;else if(counts_.collisions>before)++collision;}bool usable=valid>0;lift_<<num(x)<<','<<usable_reps_<<','<<valid<<','<<collision<<','<<(usable?1:0)<<'\n';if(usable){if(!any){usable_bottom_=x;any=true;}usable_top_=x;}}if(!any)throw std::runtime_error("No usable lift position");RCLCPP_INFO(node_->get_logger(),"BOUNDARY_SWEEP usable_lift=[%.3f,%.3f]",usable_bottom_,usable_top_);}
 void buildSlices(){for(double r:{0.,.25,.5,.75,1.})slices_.push_back(usable_bottom_+r*(usable_top_-usable_bottom_));}
 std::vector<moveit::core::RobotState> seeds(Config c,double lift){std::vector<moveit::core::RobotState> result;for(int i=0;i<seed_attempts_&&static_cast<int>(result.size())<seed_count_;++i){auto s=nominal();setSeed(s,c,lift,i);if(validOnly(s,c))result.push_back(s);}return result;}
 bool better(const Eval&a,const Eval&b)const{return a.valid&&(!b.valid||a.score>b.score);}
 void coordinate(moveit::core::RobotState&s,Eval&best,Config c,View v,Boundary boundary,double angle,const Eigen::Vector3d&ref,bool global,double step){for(auto& n:active(c)){double original=s.getVariablePosition(n),chosen=original;Eval local=best;auto bound=model_->getVariableBounds(n);std::vector<double> candidates;if(global){for(double f:coarse_fractions_)candidates.push_back(bound.min_position_+f*(bound.max_position_-bound.min_position_));}else{candidates={std::max(bound.min_position_+epsilon_,original-step),std::min(bound.max_position_-epsilon_,original+step)};}for(double q:candidates){s.setVariablePosition(n,q);s.update();Eval e=evaluate(s,c,v,boundary,angle,ref);if(better(e,local)){local=e;chosen=q;}}s.setVariablePosition(n,chosen);s.update();best=local;}}
 void writeState(Config c,View v,Boundary b,double lift,double angle,const moveit::core::RobotState&s,const Eval&e,int seed){states_<<cname(c)<<','<<vname(v)<<','<<bname(b)<<','<<num(lift)<<','<<num(angle*180/M_PI)<<",1,0,"<<num(e.tcp.x())<<','<<num(e.tcp.y())<<','<<num(e.tcp.z())<<','<<num(e.margin)<<','<<num(e.clearance)<<','<<seed<<",COARSE_GLOBAL_PLUS_5DEG_PLUS_1DEG,"<<jnames()<<','<<jvalues(s)<<'\n';}
 void sweep(Config c,double lift,View v){auto seedpool=seeds(c,lift);if(seedpool.empty()){RCLCPP_WARN(node_->get_logger(),"No seed config=%s lift=%.3f",cname(c).c_str(),lift);return;}auto refstate=nominal();refstate.setVariablePosition("lift_joint",lift);refstate.update();auto ref=rel(refstate,reference_);int direction=0;for(double deg=-180;deg<180-1e-9;deg+=angle_step_,++direction){double angle=deg*M_PI/180.;for(auto boundary:{Boundary::INNER,Boundary::OUTER}){Eval global;moveit::core::RobotState winner(model_);int winseed=-1;for(int si=0;si<static_cast<int>(seedpool.size());++si){auto s=seedpool[si];Eval e=evaluate(s,c,v,boundary,angle,ref);if(!e.valid)continue;for(int pass=0;pass<coarse_passes_;++pass)coordinate(s,e,c,v,boundary,angle,ref,true,0);coordinate(s,e,c,v,boundary,angle,ref,false,refine_deg_*M_PI/180.);coordinate(s,e,c,v,boundary,angle,ref,false,final_deg_*M_PI/180.);if(better(e,global)){global=e;winner=s;winseed=si;}}if(global.valid)writeState(c,v,boundary,lift,angle,winner,global,winseed);}if((direction+1)%progress_==0)RCLCPP_INFO(node_->get_logger(),"BOUNDARY_SWEEP config=%s lift=%.3f view=%s dir=%d checks=%zu",cname(c).c_str(),lift,vname(v).c_str(),direction+1,counts_.checks);}}
 void writeMetadata(){std::ofstream o(std::filesystem::path(out_)/"boundary_sweep_workspace_sampling_metadata.csv");o<<"key,value\nmethod,DIRECTED_SUPPORT_COORDINATE_SWEEP\nrandom_workspace_sampling,false\nbase_frame,"<<base_frame_<<"\ntcp_frame,"<<tcp_frame_<<"\nreference_link,"<<reference_link_<<"\nusable_lift_top,"<<num(usable_top_)<<"\nusable_lift_bottom,"<<num(usable_bottom_)<<"\nlift_step,"<<num(lift_step_)<<"\nsupport_angle_step_deg,"<<num(angle_step_)<<"\ncoarse_line_fractions,0.1;0.3;0.5;0.7;0.9\nrefine_steps_deg,"<<num(refine_deg_)<<';'<<num(final_deg_)<<"\ncollision_checks,"<<counts_.checks<<"\nself_collision_rejections,"<<counts_.collisions<<"\njoint_bound_rejections,"<<counts_.bounds<<"\nvalid_evaluations,"<<counts_.valid<<"\n";for(size_t i=0;i<slices_.size();++i)o<<"lift_slice_"<<i<<','<<num(slices_[i])<<'\n';o<<"ik_used,false\nompl_used,false\ntrajectory_execution,false\ncontroller,false\nros2_control,false\nhardware,false\namr_motion,false\n";}

 rclcpp::Node::SharedPtr node_;robot_model_loader::RobotModelLoaderPtr loader_;moveit::core::RobotModelPtr model_;planning_scene::PlanningScenePtr scene_;const moveit::core::JointModelGroup *arm_{nullptr},*full_{nullptr};const moveit::core::LinkModel *base_{nullptr},*tcp_{nullptr},*reference_{nullptr};
 std::string out_,base_frame_,tcp_frame_,reference_link_,arm_group_,full_group_;double lift_step_{},angle_step_{},refine_deg_{},final_deg_{},epsilon_{},inner_penalty_{},max_wall_{},usable_top_{},usable_bottom_{};int usable_reps_{},seed_attempts_{},seed_count_{},coarse_passes_{},max_checks_{},progress_{};std::vector<double>coarse_fractions_,slices_;std::vector<std::string>names_,sample_names_;std::array<unsigned,9>primes_{};Counts counts_;Clock::time_point start_;std::ofstream states_,lift_;
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);auto n=std::make_shared<rclcpp::Node>("boundary_sweep_workspace");try{Runner(n).run();}catch(const std::exception&e){RCLCPP_FATAL(n->get_logger(),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
