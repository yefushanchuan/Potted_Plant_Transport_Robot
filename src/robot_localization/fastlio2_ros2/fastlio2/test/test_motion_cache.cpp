#include "map_builder/motion_linearization.h"
#include <iostream>
int main() {
 State anchor; anchor.time=.5;anchor.r_wi=fast_time::expSO3({.2,-.1,.3});
 V22D d=V22D::Zero(); d.segment<3>(0)<<.04,-.02,.03;
 d.segment<3>(6)<<-.03,.02,.01;d(21)=.003;d(12)=.05;
 State current=anchor;current+=d;
 const auto chart=fast_time::cachedChartJacobian(current-anchor);
 double error=0;
 for(int k=0;k<22;++k){V22D e=V22D::Zero();e(k)=1e-6;State a=current,b=current;a+=e;b+=(-e).eval();
  error=std::max(error,(((a-anchor)-(b-anchor))/2e-6-chart.col(k)).norm());}
 if(error>1e-7)return 1;
 if(fast_time::refreshMotionLinearization(V22D::Zero()))return 2;
 V22D large=V22D::Zero();large(21)=.006;if(!fast_time::refreshMotionLinearization(large))return 3;
 std::vector<IMUData> imu;for(int k=0;k<=1000;++k){double t=k*.001;
 imu.emplace_back(V3D(.3*sin(t),.2*cos(t),9.81),V3D(.2,.1,.3),t);}
 fast_time::TimedPoint point{V3D(3,-1,.5),.47};
 auto eval=[&](const State& s,Eigen::Matrix<double,3,22>*j){fast_time::Trajectory tr;Eigen::Vector3d w;
 if(!tr.build(s,imu,.4,.6,.002)||!tr.point(point,s.td,w,j))throw std::runtime_error("coverage");return w;};
 Eigen::Matrix<double,3,22> jac;auto w0=eval(anchor,&jac);
 V22D step=V22D::Zero();step.segment<3>(0)<<.002,-.001,.003;step(12)=.01;step(15)=.001;step(18)=.01;step(21)=.001;
 auto remainder=[&](double scale){State s=anchor;s+=(scale*step).eval();return (eval(s,nullptr)-w0-jac*(s-anchor)).norm();};
 double e1=remainder(1),e2=remainder(.5);
 if(!(e2<.35*e1 && e1<1e-3))return 4;
 // Cached model derivatives must match finite differences away from its anchor.
 Eigen::Matrix<double,3,22> hc=jac*chart;
 for(int k=0;k<22;++k){V22D e=V22D::Zero();e(k)=1e-6;State a=current,b=current;a+=e;b+=(-e).eval();
 const Eigen::Vector3d fd=jac*((a-anchor)-(b-anchor))/2e-6;if((fd-hc.col(k)).norm()>1e-6)return 5;}
 std::cout<<"chart_error="<<error<<" remainder="<<e1<<" half_step_remainder="<<e2<<"\n";
 return 0;
}
