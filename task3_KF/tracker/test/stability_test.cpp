#include "math_tools.hpp"
#include "tracker.hpp"
#include "simple_target.hpp"
#include <iostream>
#include <stdexcept>
#include <random>

void check(bool value, const char * message) { if (!value) throw std::runtime_error(message); }
int main() {
  using namespace task3;
  for (const Eigen::Vector3d p : {Eigen::Vector3d(0,0,3), Eigen::Vector3d(.3,-.2,5), Eigen::Vector3d(-.1,.4,2)}) {
    Eigen::Matrix3d numeric;
    for (int i=0;i<3;++i) {
      Eigen::Vector3d d=Eigen::Vector3d::Zero(); d[i]=1e-6;
      numeric.col(i)=(xyz2ypd(p+d)-xyz2ypd(p-d))/(2e-6);
    }
    check((numeric-xyz2ypd_jacobian(p)).norm()<1e-6, "camera Jacobian");
  }
  check(xyz2ypd({0,0,3}).head<2>().norm()==0, "forward angles");
  check(xyz2ypd({0,1,3})[1]>0, "camera down pitch");
  ExtendedKalmanFilter ekf(Eigen::VectorXd::Zero(3),Eigen::MatrixXd::Identity(3,3));
  ekf.innovation_gate=18.47;
  auto H=Eigen::MatrixXd::Identity(3,3).eval();
  ekf.update(Eigen::Vector3d(100,0,0),H,H);
  check(!ekf.update_accepted && ekf.x.norm()==0 && (ekf.P-H).norm()==0,"outlier rollback");
  ekf.update(Eigen::Vector3d(1,0,0),H,H);
  check(ekf.update_accepted && std::abs(ekf.last_nis-.5)<1e-9,"prior NIS");
  auto t=std::chrono::steady_clock::time_point{};
  ObservedArmor a; a.number="3"; a.xyz={0,0,3}; a.yaw=-M_PI/2;
  Tracker tracker(.2,1.); tracker.radius_override=.31;
  check(tracker.init({a},t),"init");
  for(int i=1;i<=60;++i) {
    t+=std::chrono::milliseconds(33);
    const int before=tracker.target->update_count();
    tracker.update({a,a},t);
    check(tracker.target.has_value(),"stationary track preserved");
    check(tracker.target->update_count()==before+1,"duplicate observation single update");
  }
  check(tracker.state==State::TRACKING,"track confirmed");
  for(int i=0;i<30;++i) {t+=std::chrono::milliseconds(33);tracker.update({},t);}
  check(tracker.state==State::TEMP_LOST && !tracker.target->diverged(),"occlusion coast");
  check(tracker.tracked_number=="3", "one second occlusion preserves vehicle identity");
  t+=std::chrono::milliseconds(33);tracker.update({a},t);
  check(tracker.state==State::TRACKING,"occlusion recovery");
  const auto previous=tracker.target->ekf_x();
  tracker.target->predict(t-std::chrono::seconds(1));
  check((previous-tracker.target->ekf_x()).norm()==0,"backwards time ignored");
  for(auto model:{SimpleTarget::Model::CV,SimpleTarget::Model::CA}) {
    SimpleTarget simple(a,model,t);
    for(int i=0;i<20;++i) {simple.predict(.033);simple.update(a);}
    check(!simple.diverged() && (simple.armor_xyz()-a.xyz).norm()<1e-8,"single plate regression");
  }
  Tracker rotating(.2, 1.0); rotating.radius_override=.31;
  std::mt19937 rng(42); std::normal_distribution<double> noise(0, .003);
  for (int frame=0;frame<300;++frame) {
    std::vector<ObservedArmor> observations;
    const double phase=-M_PI/2 + 2.0*.033*frame;
    for (int id=0;id<4;++id) {
      const double angle=limit_rad(phase+id*M_PI/2);
      if (std::sin(angle)>-.15) continue;
      ObservedArmor plate=a;plate.yaw=angle+noise(rng);
      plate.xyz={.31*std::cos(angle)+noise(rng),noise(rng),3.31+.31*std::sin(angle)+noise(rng)};
      observations.push_back(plate);
    }
    if (frame%2) std::reverse(observations.begin(),observations.end());
    if (frame>=150 && frame<158) observations.clear();
    t+=std::chrono::milliseconds(33);
    if(frame==0) check(rotating.init(observations,t), "rotating init");
    else rotating.update(observations,t);
    check(rotating.target.has_value() && !rotating.target->diverged(), "no rotating track loss");
    if(frame>50) check((rotating.target->center()-Eigen::Vector3d(0,0,3.31)).norm()<.2,"rotating center stable");
  }
  Tracker weak_track(.2,1.0);
  ObservedArmor weak=a; weak.confidence=.45;
  check(!weak_track.init({weak},t),"weak candidate cannot initialize");
  weak=a;weak.color_uncertain=true;
  check(!weak_track.init({weak},t),"unknown color cannot initialize");
  check(weak_track.init({a},t),"strong initialization");
  for(int i=0;i<7;++i){t+=std::chrono::milliseconds(33);weak_track.update({a},t);}
  check(weak_track.state==State::TRACKING,"confirmed strong track");
  weak=a;weak.confidence=.45;weak.number="4";weak.class_margin=.2;
  int count=weak_track.target->update_count();
  t+=std::chrono::milliseconds(33);weak_track.update({weak},t);
  check(weak_track.target->update_count()==count+1 && weak_track.tracked_number=="3", "ambiguous label coast preserves identity");
  weak.class_margin=3.;
  count=weak_track.target->update_count();t+=std::chrono::milliseconds(33);weak_track.update({weak},t);
  check(weak_track.target->update_count()==count,"confident wrong label rejected");
  weak=a;weak.confidence=.45;
  for(int i=0;i<140 && weak_track.target;++i){t+=std::chrono::milliseconds(33);weak_track.update({weak},t);}
  check(weak_track.state==State::LOST,"weak candidates cannot coast forever");
  Tracker handoff(.2,1.0);handoff.radius_override=.31;handoff.init({a},t);
  for(int i=0;i<7;++i){t+=std::chrono::milliseconds(33);handoff.update({a},t);}
  // 同时间戳隔离选板逻辑：零预测时间、零观测残差，显式控制模型相位和转速。
  auto select = [&](double angle_deg, double speed, std::initializer_list<int> ids) {
    auto x=handoff.target->ekf_x();
    x[0]=0; x[1]=0; x[2]=0; x[3]=0; x[4]=3.31; x[5]=0;
    x[6]=-M_PI/2+angle_deg*M_PI/180; x[7]=speed;
    x[8]=.31; x[9]=0; x[10]=0;
    handoff.target->set_x(x);
    const auto plates=handoff.target->armor_xyza_list();
    std::vector<ObservedArmor> observations;
    for(int id:ids) {
      auto obs=a;obs.xyz=plates[id].head<3>();obs.yaw=plates[id][3];
      observations.push_back(obs);
    }
    handoff.update(observations,t);
  };
  select(-40,0,{0,1});
  check(handoff.primary_id()==0,"most frontal plate initializes aim lock");
  select(-50,0,{0,1});
  check(handoff.primary_id()==0,"two eligible plates retain aim lock");
  select(-65,0,{0,1});
  check(handoff.primary_id()==1 && handoff.matched_armor.has_value(),
        "angle exit switches while previous plate still detected");
  select(35,3,{0,3});
  check(handoff.primary_id()==3,"positive spin selects incoming plate");
  select(-35,-3,{0,1});
  check(handoff.primary_id()==1,"negative spin selects incoming plate");
  select(25,3,{0});
  check(handoff.primary_id()==-1 && handoff.state==State::TRACKING,
        "no aim window candidate does not discard vehicle track");
  select(-65,0,{});
  check(handoff.primary_id()==1 && handoff.state==State::TEMP_LOST,
        "occlusion continues model based aim selection");
  for(int i=0;i<100 && handoff.target;++i) {
    t+=std::chrono::milliseconds(33);handoff.update({},t);
  }
  check(handoff.state==State::LOST,"occlusion expires at time limit");
  Tracker behind(.2,1.0);
  ObservedArmor stable_armor=a; stable_armor.xyz={3,0,-1}; stable_armor.yaw=M_PI;
  check(behind.init({stable_armor},t) && !behind.target->diverged(),
        "stable reference negative z is not camera negative depth");
  behind.reset();
  check(behind.state==State::LOST && !behind.target && behind.tracked_number.empty() &&
        !behind.last_obs && !behind.matched_armor, "coordinate reset clears old identity and state");
  std::cout<<"stability checks passed\n";
}
