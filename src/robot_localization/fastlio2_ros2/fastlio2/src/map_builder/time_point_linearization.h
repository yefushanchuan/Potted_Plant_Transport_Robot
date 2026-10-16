#pragma once
#include "timed_motion.h"
namespace fast_time {
// A deskewed point and its time derivative in the reference LiDAR frame.
// Only time is linearized here. Pose/extrinsics use the current SO(3) state.
struct TimePointLinearization {
    V3D point = V3D::Zero();
    V3D rate = V3D::Zero();
    V3D world(const State& s, double delta_td,
              Eigen::Matrix<double,3,22>* jac=nullptr) const {
        const V3D p = point + rate * delta_td;
        const V3D q = s.r_il * p + s.t_il;
        if (jac) {
            jac->setZero();
            jac->block<3,3>(0,0) = -s.r_wi * skew(q);
            jac->block<3,3>(0,3).setIdentity();
            jac->block<3,3>(0,6) = -s.r_wi * s.r_il * skew(p);
            jac->block<3,3>(0,9) = s.r_wi;
            jac->col(21) = s.r_wi * s.r_il * rate;
        }
        return s.r_wi*q+s.t_wi;
    }
};
inline bool linearizeTimePoint(const Trajectory& trajectory, const State& anchor,
                              const std::vector<IMUData>& imu, const TimedPoint& p,
                              double gap, TimePointLinearization& out) {
    State sample; IMUData measurement;
    const double t=p.time+anchor.td;
    if (!trajectory.evaluate(t,sample) || !interpolate(imu,t,gap,measurement)) return false;
    const V3D q=anchor.r_il*p.point+anchor.t_il;
    const V3D world=sample.r_wi*q+sample.t_wi;
    const V3D rate=sample.v+sample.r_wi*((measurement.gyro-sample.bg).cross(q));
    out.point=anchor.r_il.transpose()*(anchor.r_wi.transpose()*(world-anchor.t_wi)-anchor.t_il);
    out.rate=anchor.r_il.transpose()*anchor.r_wi.transpose()*rate;
    return out.point.allFinite() && out.rate.allFinite();
}
} // namespace fast_time
