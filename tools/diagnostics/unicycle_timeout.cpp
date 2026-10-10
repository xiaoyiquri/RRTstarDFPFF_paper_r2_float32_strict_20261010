#include "production/model.h"
#include "core/rrtstar.h"
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <iomanip>
using namespace rrtpff;
using namespace rrtpff::production;
using Base=FloatModel<BatchRobot::Unicycle>;
struct Check {bool valid{true};std::string reason{"valid"};float min_speed{INFINITY},max_speed{},max_omega{};};
struct Counts {std::map<std::string,std::size_t> reasons;std::size_t accepted{},rescue_tested{},rescued{},goal_rescued{};};
class Probe:public Base {
public:
    Probe(const PaperDataAdapter& a,const YAML::Node& y):Base(a,y),geometry(a.geometry()),counts(std::make_shared<Counts>()) {
        auto& d=a.problem();cfg.robot=BatchRobot::Unicycle;cfg.state_dim=3;cfg.control_dim=2;cfg.position_dim=2;
        cfg.epsilon=1e-6f;cfg.max_velocity=float(d.max_velocity);cfg.max_omega=float(d.max_omega);
        for(int k=0;k<3;++k){cfg.state_lower[k]=float(d.lower_state(k));cfg.state_upper[k]=float(d.upper_state(k));}
        for(int k=0;k<2;++k){cfg.control_lower[k]=float(d.lower_control(k));cfg.control_upper[k]=float(d.upper_control(k));}
        branch=y["lifting"]["start"]["signed_velocity"].as<float>()<0?-1:1;
        step=y["validation"]["max_step"].as<float>();maximum=y["validation"]["max_samples"].as<std::size_t>();
        min_time=y["pff"]["min_time"].as<float>();max_time=y["pff"]["max_time"].as<float>();
        FloatPFFConfig fc;fc.min_time=min_time;fc.max_time=max_time;fc.time_policy=FloatTimePolicy::OptimalOnly;
        Solver::Weight R=Solver::Weight::Zero();for(int k=0;k<2;++k)R(k,k)=y["pff"]["R_diagonal"][k].as<float>();
        solver=std::make_shared<Solver>(R,fc);
    }
    Check diagnose(const FlatTrajectory& tr)const {
        Check result;
        auto cert=certifyUnicycleSpeed(tr,cfg.epsilon);
        if(cert!=SpeedCertificate::Valid){result.valid=false;result.reason=cert==SpeedCertificate::Uncertain?"certificate_uncertain":"speed_zero";return result;}
        BernsteinCurve curve;if(!floatBernstein(tr,curve)){result.valid=false;result.reason="numeric";return result;}
        const auto n=std::ceil(tr.T/step)+1.f;
        if(n+1.f>float(maximum)){result.valid=false;result.reason="sample_limit";return result;}
        const std::size_t intervals=std::size_t(n),count=intervals+1,batches=(count+7)/8;
        for(std::size_t b=0;b<batches;++b){
            float times[8]{};unsigned active=0;
            for(int lane=0;lane<8;++lane){auto i=b+std::size_t(lane)*batches;if(i<count){active|=1u<<lane;times[lane]=float(i)/float(intervals);}}
            PhysicalBatch out;curveBatch(curve,times,cfg,branch,active,geometry,out);
            bool speed_max=false,omega=false,numeric=false,speed_zero=false;
            for(int lane=0;lane<8;++lane)if(active&(1u<<lane)){
                const float v=std::abs(out.controls[0][lane]),w=std::abs(out.controls[1][lane]);
                result.min_speed=std::min(result.min_speed,v);result.max_speed=std::max(result.max_speed,v);result.max_omega=std::max(result.max_omega,w);
                if(!(out.df_mask&(1u<<lane))){speed_max|=v>cfg.max_velocity;omega|=w>cfg.max_omega;
                    numeric|=!std::isfinite(v)||!std::isfinite(w);speed_zero|=v<=cfg.epsilon;}
            }
            if(result.valid&&out.valid_mask!=active){
                result.valid=false;
                if(out.df_mask!=active)result.reason=numeric?"numeric":(speed_zero?"speed_zero":(speed_max&&omega?"speed_max_and_omega":(speed_max?"speed_max":(omega?"omega":"df_other"))));
                else if(out.state_mask!=active)result.reason="state_bounds";
                else if(out.control_mask!=active)result.reason="control_bounds";
                else result.reason="collision";
            }
        }
        return result;
    }
    float rescue(const FlatTrajectory& tr)const {
        const bool goal=isGoal(tr.end,1e-5f);
        auto target=goal?Constraint::full(tr.end):Constraint::output(tr.end.output());
        std::vector<float> times;
        for(float factor:{1.1f,1.25f,1.5f,2.f,3.f})if(tr.T*factor<=max_time)times.push_back(tr.T*factor);
        for(int k=1;k<=12;++k)times.push_back(min_time+(max_time-min_time)*float(k)/13.f);
        for(float T:times){
            auto edge=solver->fixedTime(tr.start,target,T);
            if(!edge.success||!prepareEdge(tr.start,tr.metadata,edge.trajectory))continue;
            PlanResult unused;
            if(Base::trajectoryValid(edge.trajectory,unused))return T;
        }
        return 0.f;
    }
    bool trajectoryValid(const FlatTrajectory& tr,PlanResult& stats)const {
        const bool ok=Base::trajectoryValid(tr,stats);
        if(ok){++counts->accepted;return true;}
        auto check=diagnose(tr);if(check.valid)throw std::runtime_error("diagnostic/production mismatch");
        ++counts->reasons[check.reason];
        if(counts->rescue_tested<128){++counts->rescue_tested;if(rescue(tr)>0.f){++counts->rescued;if(isGoal(tr.end,1e-5f))++counts->goal_rescued;}}
        return false;
    }
    const SphereGeometry& geometry;PhysicalBatchConfig cfg;int branch{};float step{},min_time{},max_time{};std::size_t maximum{};
    std::shared_ptr<Counts> counts;std::shared_ptr<Solver> solver;
};
int main(int argc,char**argv){try{
    if(argc!=3)throw std::runtime_error("usage: unicycle_timeout CONFIG OUTPUT_DIRECTORY");
    auto file=std::filesystem::absolute(argv[1]);auto y=YAML::LoadFile(file.string());
    auto resolve=[&](std::string p){std::filesystem::path value(p);return value.is_absolute()?value:file.parent_path()/value;};
    auto source=resolve(y["dynobench"]["source"].as<std::string>());
    PaperDataAdapter adapter((source/"envs"/y["dynobench"]["environment_file"].as<std::string>()).string(),
        resolve(y["dynobench"]["model_file"].as<std::string>()).string(),.125,.1,.00001);
    Probe model(adapter,y);auto output=std::filesystem::path(argv[2]);std::filesystem::create_directories(output);
    std::ofstream angles(output/"local_angles.csv");angles<<std::setprecision(9)<<"angle_deg,Tstar,valid,reason,min_speed,max_speed,max_omega,rescue_T\n";
    for(int degrees=-180;degrees<=180;degrees+=15){
        const float angle=float(degrees)*3.141592653589793f/180.f,range=.3f;
        Base::FlatOutput target=model.start().output();target(0)+=range*std::cos(angle);target(1)+=range*std::sin(angle);
        auto edge=model.solver->solve(model.start(),Base::Constraint::output(target));
        if(!edge.success||!model.prepareEdge(model.start(),model.initialMetadata(),edge.trajectory))throw std::runtime_error("invalid local solve");
        PlanResult unused;auto check=model.diagnose(edge.trajectory);
        if(check.valid!=model.Base::trajectoryValid(edge.trajectory,unused))throw std::runtime_error("local diagnostic mismatch");
        angles<<degrees<<','<<edge.T<<','<<check.valid<<','<<check.reason<<','<<check.min_speed<<','<<check.max_speed<<','<<check.max_omega<<','<<(check.valid?0.f:model.rescue(edge.trajectory))<<'\n';
    }
    PlannerConfigT<float> c;c.timeout=60;c.max_nodes=0;c.max_iterations=2000;c.seed=41;c.range=.3f;c.goal_bias=.15f;c.rewire_factor=3;
    c.goal_tolerance=1e-6f;c.stop_at_first_solution=true;c.rewire_before_first_solution=false;c.prune=false;
    c.simd_enabled=true;c.simd_screen=false;c.explicit_goal_connection=true;c.goal_parent_limit=16;
    c.cache_goal_connections=true;c.minimum_parent_candidates=8;
    RRTstar<Probe> planner(model,c,*model.solver);auto stats=planner.solve();
    std::ofstream histogram(output/"rejections.csv");histogram<<"reason,count\n";
    for(auto [reason,count]:model.counts->reasons)histogram<<reason<<','<<count<<'\n';
    std::ofstream summary(output/"summary.csv");summary<<"iterations,nodes,accepted,rejected_rescue_tested,rescued,goal_rescued,success,termination\n"
        <<stats.iterations<<','<<stats.node_count<<','<<model.counts->accepted<<','<<model.counts->rescue_tested<<','<<model.counts->rescued<<','<<model.counts->goal_rescued<<','<<stats.success<<','<<stats.termination_reason<<'\n';
    std::cout<<"diagnosis iterations="<<stats.iterations<<" nodes="<<stats.node_count<<" rescued="<<model.counts->rescued<<'/'<<model.counts->rescue_tested<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
