#include "map_builder/ieskf.h"
#include <iostream>

static void configure(IESKF &kf, double target) {
    kf.time_options.load(YAML::Load("initial_td_std: 0.02\nmax_td_step: 0.005"));
    kf.P().setIdentity(); kf.P()(21,21)=.02*.02;
    kf.time_excited=true;
    kf.setStopFunction([](const V22D& dx){return dx.norm()<1e-8;});
    kf.setLossFunction([target](State& x, SharedState& o){
        o.H.setZero();o.b.setZero();o.H(21,21)=1e8;
        o.b(21)=1e8*(x.td-target);o.count=10;o.res=std::abs(x.td-target);o.valid=true;
    });
}
int main() {
    IESKF bounded;configure(bounded,.02);
    if(!bounded.update() || std::abs(bounded.x().td-.005)>1e-12 ||
       !bounded.report.td_limited || std::abs(bounded.P()(21,21)-.0004)>1e-12)return 1;
    IESKF frozen;configure(frozen,.02);frozen.time_excited=false;
    if(!frozen.update() || frozen.x().td!=0 || frozen.report.td_updated ||
       std::abs(frozen.P()(21,21)-.0004)>1e-12)return 2;
    IESKF invalid;configure(invalid,.02);int calls=0;
    invalid.setLossFunction([&](State&,SharedState& o){
        o.H.setIdentity();o.b.setOnes();o.count=10;o.valid=(++calls==1);
    });
    auto prior=invalid.x();auto cov=invalid.P();
    if(invalid.update() || (invalid.x()-prior).norm()>1e-12 || (invalid.P()-cov).norm()>1e-12)return 3;
    TimeOffsetOptions options;
    if(std::abs(options.requiredFuture(.05)-.055)>1e-12 || options.requiredFuture(-.05)!=0)return 4;
    try{options.load(YAML::Load("max_td_step: -1"));return 5;}catch(const std::invalid_argument&){}
    std::cout<<"timing bounds, covariance preservation, rollback and future coverage passed\n";
    return 0;
}
