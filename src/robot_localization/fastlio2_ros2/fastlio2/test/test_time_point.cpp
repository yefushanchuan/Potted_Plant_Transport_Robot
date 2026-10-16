#include "map_builder/time_point_linearization.h"
#include <iostream>
int main() {
    State s; s.time=.5;s.td=.02;s.v={.5,-.2,.1};
    s.r_wi=fast_time::expSO3({.2,-.1,.3});s.r_il=fast_time::expSO3({-.1,.04,.08});
    s.t_il={.12,-.05,.03};
    std::vector<IMUData> imu;
    for(int i=0;i<=200;++i) {double t=i*.005;imu.emplace_back(V3D(.3,.2,9.81),V3D(.2,-.1,.5),t);}
    fast_time::Trajectory tr;if(!tr.build(s,imu,.2,.8,.011))return 1;
    double jerror=0,approx_error=0,base_error=0;
    for(int i=0;i<100;++i) {
        fast_time::TimedPoint p{V3D(3.,.01*i,1.),.3+.003*i};
        fast_time::TimePointLinearization model;
        if(!fast_time::linearizeTimePoint(tr,s,imu,p,.011,model))return 2;
        V3D exact;tr.point(p,s.td,exact);
        base_error=std::max(base_error,(model.world(s,0)-exact).norm());
        for(double d:{-.0025,-.001,0.,.001,.0025}) {
            tr.point(p,s.td+d,exact);
            approx_error=std::max(approx_error,(model.world(s,d)-exact).norm());
            V22D step=V22D::Zero();step.head<3>()=V3D(.01,-.02,.015);
            State updated=s;updated+=step;Eigen::Matrix<double,3,22> jac;
            model.world(updated,d,&jac);
            for(int k=0;k<22;++k) {
                V22D e=V22D::Zero();e[k]=1e-6;
                State a=updated,b=updated;a+=e;b+=(-e);
                V3D plus=model.world(a,d+(k==21?1e-6:0));
                V3D minus=model.world(b,d-(k==21?1e-6:0));
                jerror=std::max(jerror,((plus-minus)/2e-6-jac.col(k)).norm());
            }
        }
    }
    std::cout<<"base_error="<<base_error<<" jacobian_error="<<jerror<<" approximation_error_2p5ms="<<approx_error<<"\n";
    return base_error<1e-12 && jerror<1e-7 && approx_error<.0001 ? 0 : 3;
}
