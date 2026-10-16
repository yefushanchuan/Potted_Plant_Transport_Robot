#pragma once
#include <Eigen/Eigen>
#include <algorithm>
#include <vector>
#include <cmath>
#include "ieskf.h"
namespace fast_time {
using Matrix = Eigen::Matrix<double, 22, 22>;
using NativeState = State;
using Imu = IMUData;
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d h;
    h << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return h;
}
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d &v) {
    double a = v.norm();
    if (a < 1e-10)
        return Eigen::Matrix3d::Identity() + skew(v) + .5 * skew(v) * skew(v);
    return Eigen::AngleAxisd(a, v / a).toRotationMatrix();
}
inline Eigen::Matrix3d rightJacobian(const Eigen::Vector3d &v) {
    double a = v.norm();
    auto h = skew(v);
    if (a < 1e-5)
        return Eigen::Matrix3d::Identity() - .5 * h + h * h / 6.;
    return Eigen::Matrix3d::Identity() - (1 - std::cos(a)) / (a * a) * h +
           (a - std::sin(a)) / (a * a * a) * h * h;
}
struct TimedPoint {
    Eigen::Vector3d point;
    double time;
};
class Trajectory {
    struct Knot {
        NativeState state;
        Matrix phi = Matrix::Identity();
    };
    std::vector<Knot> knots_;
    const std::vector<Imu> *imu_ = nullptr;
    double max_gap_ = 0;

  public:
    bool build(const NativeState &, const std::vector<Imu> &, double, double, double);
    bool evaluate(double, NativeState &, Matrix * = nullptr) const;
    bool point(const TimedPoint &, double, Eigen::Vector3d &,
               Eigen::Matrix<double, 3, 22> * = nullptr) const;
};
inline Matrix integrate(NativeState &s, const Eigen::Vector3d &measured_acc,
                        const Eigen::Vector3d &gyro, double dt) {
    Matrix f;
    const Eigen::Vector3d w = gyro - s.bg;
    const Eigen::Vector3d acc = measured_acc - s.ba;
    const Eigen::Matrix3d eh = expSO3(w * dt * 0.5), rm = s.r_wi * eh;
    const Eigen::Matrix3d da_theta = -rm * skew(acc) * eh.transpose();
    const Eigen::Matrix3d da_bg = rm * skew(acc) * rightJacobian(w * dt * 0.5) * (dt * 0.5);
    f.setIdentity();
    f.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    f.block<3, 3>(3, 0) = da_theta * (0.5 * dt * dt);
    f.block<3, 3>(3, 15) = da_bg * (0.5 * dt * dt);
    f.block<3, 3>(3, 18) = -rm * (0.5 * dt * dt);
    f.block<3, 3>(12, 0) = da_theta * dt;
    f.block<3, 3>(12, 15) = da_bg * dt;
    f.block<3, 3>(12, 18) = -rm * dt;
    f.block<3, 3>(0, 0) = expSO3(-w * dt);
    f.block<3, 3>(0, 15) = -rightJacobian(w * dt) * dt;
    const Eigen::Vector3d aw = rm * acc + s.g;
    s.t_wi += s.v * dt + aw * (0.5 * dt * dt);
    s.v += aw * dt;
    s.r_wi = Eigen::Quaterniond(s.r_wi * expSO3(w * dt)).normalized().toRotationMatrix();
    s.time += dt;
    return f;
}
inline bool interpolate(const std::vector<Imu> &s, double t, double gap, Imu &out) {
    if (s.empty() || !std::isfinite(t) || t < s.front().time || t > s.back().time)
        return false;
    auto b =
        std::lower_bound(s.begin(), s.end(), t, [](const Imu &a, double v) { return a.time < v; });
    if (b == s.end())
        return false;
    if (b->time == t) {
        out = *b;
        return out.acc.allFinite() && out.gyro.allFinite();
    }
    if (b == s.begin())
        return false;
    const auto &a = *(b - 1);
    const double dt = b->time - a.time;
    if (!(dt > 0) || dt > gap)
        return false;
    const double w = (t - a.time) / dt;
    out.time = t;
    out.acc = (1 - w) * a.acc + w * b->acc;
    out.gyro = (1 - w) * a.gyro + w * b->gyro;
    return out.acc.allFinite() && out.gyro.allFinite();
}
inline bool motionStep(NativeState &s, const std::vector<Imu> &samples, double target, double gap,
                       Matrix &f) {
    Imu a, b;
    if (!interpolate(samples, s.time, gap, a) || !interpolate(samples, target, gap, b))
        return false;
    const double dt = target - s.time;
    if (std::abs(dt) > gap + 1e-9)
        return false;
    f = integrate(s, .5 * (a.acc + b.acc), .5 * (a.gyro + b.gyro), dt);
    s.time = target;
    return true;
}
inline bool Trajectory::build(const NativeState &anchor, const std::vector<Imu> &imu, double begin,
                              double end, double gap) {
    knots_.clear();
    imu_ = &imu;
    max_gap_ = gap;
    if (begin > end || imu.empty() || !std::isfinite(begin) || !std::isfinite(end))
        return false;
    begin = std::min(begin, anchor.time);
    end = std::max(end, anchor.time);
    if (begin < imu.front().time || end > imu.back().time)
        return false;
    // Reject duplicate/backward timestamps and missing IMU coverage before any integration.
    for (size_t i = 1; i < imu.size(); ++i) {
        if (!(imu[i].time > imu[i - 1].time))
            return false;
        if (imu[i].time > begin && imu[i - 1].time < end && imu[i].time - imu[i - 1].time > gap)
            return false;
    }
    Knot k;
    k.state = anchor;
    knots_.push_back(k);
    std::vector<double> targets;
    for (auto it = imu.rbegin(); it != imu.rend(); ++it)
        if (it->time < anchor.time && it->time > begin)
            targets.push_back(it->time);
    if (begin < anchor.time)
        targets.push_back(begin);
    for (double t : targets) {
        Matrix f;
        if (!motionStep(k.state, imu, t, gap, f))
            return false;
        k.phi = (f * k.phi).eval();
        knots_.push_back(k);
    }
    std::reverse(knots_.begin(), knots_.end());
    k = Knot();
    k.state = anchor;
    targets.clear();
    for (const auto &i : imu)
        if (i.time > anchor.time && i.time < end)
            targets.push_back(i.time);
    if (end > anchor.time)
        targets.push_back(end);
    for (double t : targets) {
        Matrix f;
        if (!motionStep(k.state, imu, t, gap, f))
            return false;
        k.phi = (f * k.phi).eval();
        knots_.push_back(k);
    }
    return true;
}
inline bool Trajectory::evaluate(double t, NativeState &state, Matrix *sensitivity) const {
    if (knots_.empty() || !std::isfinite(t) || t < knots_.front().state.time ||
        t > knots_.back().state.time)
        return false;
    auto it = std::upper_bound(knots_.begin(), knots_.end(), t,
                               [](double v, const Knot &k) { return v < k.state.time; });
    if (it == knots_.begin())
        return false;
    --it;
    state = it->state;
    Matrix f;
    if (!motionStep(state, *imu_, t, max_gap_, f))
        return false;
    if (sensitivity)
        *sensitivity = f * it->phi;
    return true;
}
inline bool Trajectory::point(const TimedPoint &p, double td, Eigen::Vector3d &world,
                              Eigen::Matrix<double, 3, 22> *jac) const {
    NativeState s;
    const double t = p.time + td;
    if (!p.point.allFinite() || knots_.empty() || !std::isfinite(t) ||
        t < knots_.front().state.time || t > knots_.back().state.time)
        return false;
    auto it = std::upper_bound(knots_.begin(), knots_.end(), t,
                               [](double v, const Knot &k) { return v < k.state.time; });
    if (it == knots_.begin())
        return false;
    --it;
    s = it->state;
    Matrix f;
    if (!motionStep(s, *imu_, t, max_gap_, f))
        return false;
    const Eigen::Vector3d q = s.r_il * p.point + s.t_il;
    world = s.r_wi * q + s.t_wi;
    if (jac) {
        Eigen::Matrix<double, 3, 22> h = Eigen::Matrix<double, 3, 22>::Zero();
        h.block<3, 3>(0, 3).setIdentity();
        h.block<3, 3>(0, 0) = -s.r_wi * skew(q);
        h.block<3, 3>(0, 6) = -s.r_wi * s.r_il * skew(p.point);
        h.block<3, 3>(0, 9) = s.r_wi;
        // Chain rule with a narrow intermediate: (3x22 * 22x22) * 22x22.
        // This retains every state column while avoiding per-point f * phi (22x22).
        // Both f and the knot sensitivity are evaluated from this iteration's state.
        const Eigen::Matrix<double, 3, 22> local = h * f;
        jac->noalias() = local * it->phi;
        Imu sample;
        if (!interpolate(*imu_, p.time + td, max_gap_, sample))
            return false;
        // Endpoint derivative; the fixed IMU anchor does NOT move with td.
        jac->col(21) = s.v + s.r_wi * ((sample.gyro - s.bg).cross(q));
    }
    return true;
}
} // namespace fast_time

inline bool deskewNative(const State &anchor, const std::vector<IMUData> &imus,
                         const CloudType &raw, double start, CloudType &output, double gap) {
    if (raw.empty())
        return false;
    double begin = start + raw.front().curvature * .001 + anchor.td, end = begin;
    for (const auto &p : raw) {
        begin = std::min(begin, start + p.curvature * .001 + anchor.td);
        end = std::max(end, start + p.curvature * .001 + anchor.td);
    }
    fast_time::Trajectory tr;
    if (!tr.build(anchor, imus, begin, end, gap))
        return false;
    output = raw;
    for (size_t i = 0; i < raw.size(); ++i) {
        const auto &p = raw[i];
        Eigen::Vector3d world;
        if (!tr.point({Eigen::Vector3d(p.x, p.y, p.z), start + p.curvature * .001}, anchor.td,
                      world))
            return false;
        const Eigen::Vector3d local =
            anchor.r_il.transpose() *
            (anchor.r_wi.transpose() * (world - anchor.t_wi) - anchor.t_il);
        output[i].x = local.x();
        output[i].y = local.y();
        output[i].z = local.z();
    }
    return true;
}
