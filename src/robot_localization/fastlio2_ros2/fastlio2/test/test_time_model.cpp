#include "map_builder/timed_motion.h"
#include <iostream>
#include <stdexcept>
int main(){
 std::vector<IMUData> imu;
 for(int k=0;k<=300;++k){double t=k*.002;imu.emplace_back(Eigen::Vector3d(.7*sin(2*t),.4*cos(3*t),9.81),Eigen::Vector3d(.2*cos(3*t),.3*sin(2*t),.5*cos(4*t)),t);}
 State anchor;anchor.time=.32;anchor.r_wi=fast_time::expSO3({.2,-.1,.3});anchor.v={.5,-.2,.1};anchor.bg={.01,.02,-.01};anchor.ba={.1,-.05,.02};
 std::vector<fast_time::TimedPoint> points={{{2.1,-1.2,.5},.17},{{2.2,-1.1,.45},.23},{{2.05,-1.17,.52},.42}};
 auto calc=[&](const State& current,Eigen::Matrix<double,3,22>*jac=nullptr){
  fast_time::Trajectory tr;if(!tr.build(current,imu,.05,.55,.01))throw std::runtime_error("coverage");
  Eigen::Vector3d world=Eigen::Vector3d::Zero();
  if(jac)jac->setZero();
  for(const auto&p:points){Eigen::Vector3d w;Eigen::Matrix<double,3,22> j;
    if(!tr.point(p,current.td,w,jac?&j:nullptr))throw std::runtime_error("point");
    world+=w;if(jac)*jac+=j;
  }
  if(jac)*jac/=double(points.size());
  return Eigen::Vector3d(world/double(points.size()));
 };
 double max_pose=0,max_dt=0,max_motion=0;const double eps=1e-6;
 for(double td:{-.05,-.02,0.,.02,.05}){
  State s=anchor;s.td=td;s.r_wi*=fast_time::expSO3({-.02,.01,.015});
  Eigen::Matrix<double,3,22> h;calc(s,&h);
  for(int k=0;k<22;++k){if(k>=6&&k<12)continue;State a=s,b=s;Eigen::Vector3d v=Eigen::Vector3d::Zero();v(k%3)=eps;
   if(k<3){a.r_wi*=fast_time::expSO3(v);b.r_wi*=fast_time::expSO3(-v);}else if(k<6){a.t_wi+=v;b.t_wi-=v;}else if(k<15){a.v+=v;b.v-=v;}else if(k<18){a.bg+=v;b.bg-=v;}else if(k<21){a.ba+=v;b.ba-=v;}else {a.td+=eps;b.td-=eps;}
   double error=((calc(a)-calc(b))/(2*eps)-h.col(k)).norm();if(k==21)max_dt=std::max(max_dt,error);else if(k<6)max_pose=std::max(max_pose,error);else max_motion=std::max(max_motion,error);
  }
 }
 // Coverage failures must not silently extrapolate or bridge missing samples.
 fast_time::Trajectory tr;
 if(tr.build(anchor,imu,-.01,.55,.01))return 2;
 if(tr.build(anchor,imu,.05,.61,.01))return 3;
 auto missing=imu;missing.erase(missing.begin()+100,missing.begin()+120);
 if(tr.build(anchor,missing,.05,.55,.01))return 4;
 auto duplicate=imu;duplicate[20].time=duplicate[19].time;
 if(tr.build(anchor,duplicate,.05,.55,.01))return 5;
 // Known constant rotation/velocity supports both backward and forward queries.
 std::vector<IMUData> constant;
 for(int i=0;i<=100;++i){double time=i*.01;constant.emplace_back(V3D(0,0,9.81),V3D(0,0,1),time);}
 State known;known.time=.5;known.v=V3D(1,0,0);
 if(!tr.build(known,constant,.1,.9,.011))return 6;
 for(double dt:{-.05,0.,.05}) {
  Eigen::Vector3d world;
  if(!tr.point({V3D(2,0,0),.5},dt,world))return 7;
  Eigen::Vector3d expected(2*cos(dt)+dt,2*sin(dt),0);
  if((world-expected).norm()>1e-9)return 8;
 }
 std::cout<<"{\"pose_jacobian_max_error\":"<<max_pose<<",\"dt_jacobian_max_error\":"<<max_dt<<",\"direct_motion_jacobian_max_error\":"<<max_motion<<"}\n";
 return max_pose<2e-4&&max_dt<2e-4&&max_motion<2e-4?0:1;
}
