#pragma once
#include "timed_motion.h"
namespace fast_time {
// Local coordinates are relative to the cached anchor. State updates use right
// rotation perturbations, so transport their derivatives into that chart.
inline Matrix cachedChartJacobian(const V22D &delta) {
    Matrix j=Matrix::Identity();
    j.block<3,3>(0,0)=JrInv(delta.segment<3>(0));
    j.block<3,3>(6,6)=JrInv(delta.segment<3>(6));
    return j;
}
inline bool refreshMotionLinearization(const V22D &d) {
    // Conservative geometric trust region, independent of injected test shifts.
    return d.segment<3>(0).norm()>.05 || d.segment<3>(3).norm()>.10 ||
           d.segment<3>(6).norm()>.05 || d.segment<3>(9).norm()>.05 ||
           d.segment<3>(12).norm()>.20 || d.segment<3>(15).norm()>.02 ||
           d.segment<3>(18).norm()>.20 || std::abs(d(21))>.005;
}
}
