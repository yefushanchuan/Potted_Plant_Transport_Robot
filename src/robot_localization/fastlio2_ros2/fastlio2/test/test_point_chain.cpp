#include "map_builder/timed_motion.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
using J = Eigen::Matrix<double, 3, 22>;
static bool reference(const fast_time::Trajectory &tr, const std::vector<IMUData> &imu,
                      const fast_time::TimedPoint &p, double td, V3D &world, J &jac) {
    State s; fast_time::Matrix phi;
    if (!tr.evaluate(p.time + td, s, &phi)) return false;
    const V3D q = s.r_il * p.point + s.t_il;
    world = s.r_wi * q + s.t_wi;
    J h = J::Zero();
    h.block<3,3>(0,3).setIdentity();
    h.block<3,3>(0,0) = -s.r_wi * fast_time::skew(q);
    h.block<3,3>(0,6) = -s.r_wi * s.r_il * fast_time::skew(p.point);
    h.block<3,3>(0,9) = s.r_wi;
    jac = h * phi;
    IMUData sample;
    if (!fast_time::interpolate(imu, p.time + td, .011, sample)) return false;
    jac.col(21) = s.v + s.r_wi * ((sample.gyro-s.bg).cross(q));
    return true;
}
int main() {
    std::vector<IMUData> imu;
    for (int i=0;i<=200;++i) {
        double t=i*.005;
        imu.emplace_back(V3D(.7*sin(t*7),.4*cos(t*5),9.81+.3*sin(t)),
                         V3D(.2*cos(t*3),.3*sin(t*2),.5*cos(t*4)),t);
    }
    State anchor;anchor.time=.5;anchor.r_wi=fast_time::expSO3({.2,-.1,.3});
    anchor.r_il=fast_time::expSO3({-.1,.04,.08});anchor.t_il={.12,-.05,.03};
    anchor.v={.5,-.2,.1};anchor.bg={.01,.02,-.01};anchor.ba={.1,-.05,.02};
    fast_time::Trajectory tr;
    if(!tr.build(anchor,imu,.05,.95,.011))return 1;
    std::vector<fast_time::TimedPoint> points;
    for(int i=0;i<5000;++i)points.push_back({V3D(2+.3*sin(i),-1+.2*cos(i),.5),.15+.7*i/4999.});
    double max_world=0,max_jac=0;
    for(double td:{-.05,-.02,0.,.02,.05})for(const auto &p:points) {
        V3D a,b;J ja,jb;
        if(!reference(tr,imu,p,td,a,ja)||!tr.point(p,td,b,&jb))return 2;
        max_world=std::max(max_world,(a-b).norm());max_jac=std::max(max_jac,(ja-jb).cwiseAbs().maxCoeff());
    }
    V3D w;J j;
    if(tr.point({V3D::Zero(),.01},0,w,&j))return 3;
    volatile double checksum=0;
    auto bench=[&](bool narrow) {
        auto t=std::chrono::steady_clock::now();
        for(int rep=0;rep<3;++rep)for(const auto&p:points) {
            if(narrow)tr.point(p,.02,w,&j);else reference(tr,imu,p,.02,w,j);
            checksum+=w.x()+j(0,0);
        }
        return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();
    };
    double old_ms=bench(false),new_ms=bench(true);
    std::cout<<"world_max_error="<<max_world<<" jacobian_max_abs_error="<<max_jac
             <<" reference_ms="<<old_ms<<" narrow_ms="<<new_ms<<" checksum="<<checksum<<"\n";
    return max_world<1e-12&&max_jac<1e-12?0:4;
}
