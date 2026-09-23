#include "agrobot_time_sync/time_offset_motion.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace agrobot_time {
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}
Eigen::Matrix3d expSO3(const Eigen::Vector3d& v) {
  const double a = v.norm();
  if (a < 1e-10) return Eigen::Matrix3d::Identity() + skew(v) + 0.5 * skew(v) * skew(v);
  return Eigen::AngleAxisd(a, v / a).toRotationMatrix();
}
Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& v) {
  const double a = v.norm();
  const Eigen::Matrix3d h = skew(v);
  if (a < 1e-5) return Eigen::Matrix3d::Identity() - 0.5*h + h*h/6.;
  return Eigen::Matrix3d::Identity() - (1-std::cos(a))/(a*a)*h +
         (a-std::sin(a))/(a*a*a)*h*h;
}
Vector19 difference(const State& s, const State& ref) {
  Vector19 e;
  const Eigen::AngleAxisd aa(ref.R.transpose()*s.R);
  e << s.p-ref.p, s.v-ref.v, aa.angle()*aa.axis(), s.bg-ref.bg,
       s.ba-ref.ba, s.gravity-ref.gravity, s.td-ref.td;
  return e;
}
void inject(State& s, const Vector19& d) {
  s.p += d.segment<3>(0); s.v += d.segment<3>(3);
  s.R = Eigen::Quaterniond(s.R*expSO3(d.segment<3>(6))).normalized().toRotationMatrix();
  s.bg += d.segment<3>(9); s.ba += d.segment<3>(12);
  s.gravity += d.segment<3>(15); s.td += d(18);
}
Matrix19 priorJacobian(const Vector19& e) {
  Matrix19 j = Matrix19::Identity();
  j.block<3,3>(6,6) = rightJacobian(e.segment<3>(6)).inverse();
  return j;
}
bool interpolate(const std::vector<Imu>& s, double t, double gap, Imu& out) {
  if (s.empty() || !std::isfinite(t) || t < s.front().time || t > s.back().time) return false;
  auto b = std::lower_bound(s.begin(), s.end(), t,
                           [](const Imu& a, double v) { return a.time < v; });
  if (b == s.end()) return false;
  if (b->time == t) { out = *b; return out.acc.allFinite() && out.gyro.allFinite(); }
  if (b == s.begin()) return false;
  const auto& a = *(b-1);
  const double dt = b->time-a.time;
  if (!(dt > 0) || dt > gap) return false;
  const double w = (t-a.time)/dt;
  out = {t, (1-w)*a.acc+w*b->acc, (1-w)*a.gyro+w*b->gyro};
  return out.acc.allFinite() && out.gyro.allFinite();
}
bool motionStep(State& s, const std::vector<Imu>& samples, double target,
                double gap, Matrix19& f) {
  Imu a, b;
  if (!interpolate(samples, s.time, gap, a) || !interpolate(samples, target, gap, b)) return false;
  const double dt = target-s.time;
  if (std::abs(dt) > gap+1e-9) return false;
  const Eigen::Vector3d w = 0.5*(a.gyro+b.gyro)-s.bg;
  const Eigen::Vector3d acc = 0.5*(a.acc+b.acc)-s.ba;
  const Eigen::Matrix3d eh = expSO3(w*dt*0.5), rm = s.R*eh;
  const Eigen::Matrix3d da_theta = -rm*skew(acc)*eh.transpose();
  const Eigen::Matrix3d da_bg = rm*skew(acc)*rightJacobian(w*dt*0.5)*(dt*0.5);
  f.setIdentity();
  f.block<3,3>(0,3) = Eigen::Matrix3d::Identity()*dt;
  f.block<3,3>(0,6) = da_theta*(0.5*dt*dt);
  f.block<3,3>(0,9) = da_bg*(0.5*dt*dt);
  f.block<3,3>(0,12) = -rm*(0.5*dt*dt);
  f.block<3,3>(0,15) = Eigen::Matrix3d::Identity()*(0.5*dt*dt);
  f.block<3,3>(3,6) = da_theta*dt;
  f.block<3,3>(3,9) = da_bg*dt;
  f.block<3,3>(3,12) = -rm*dt;
  f.block<3,3>(3,15) = Eigen::Matrix3d::Identity()*dt;
  f.block<3,3>(6,6) = expSO3(-w*dt);
  f.block<3,3>(6,9) = -rightJacobian(w*dt)*dt;
  const Eigen::Vector3d aw = rm*acc+s.gravity;
  s.p += s.v*dt + aw*(0.5*dt*dt); s.v += aw*dt;
  s.R = Eigen::Quaterniond(s.R*expSO3(w*dt)).normalized().toRotationMatrix();
  s.time = target;
  return s.p.allFinite() && s.v.allFinite();
}
bool Trajectory::build(const State& anchor, const std::vector<Imu>& imu,
                       double begin, double end, double gap) {
  knots_.clear(); imu_ = &imu; max_gap_ = gap;
  if (begin > end || imu.empty() || !std::isfinite(begin) || !std::isfinite(end)) return false;
  begin = std::min(begin, anchor.time); end = std::max(end, anchor.time);
  if (begin < imu.front().time || end > imu.back().time) return false;
  // Reject duplicate/backward timestamps and missing IMU coverage before any integration.
  for (size_t i=1; i<imu.size(); ++i) {
    if (!(imu[i].time > imu[i-1].time)) return false;
    if (imu[i].time > begin && imu[i-1].time < end && imu[i].time-imu[i-1].time > gap) return false;
  }
  Knot k; k.state = anchor; knots_.push_back(k);
  std::vector<double> targets;
  for (auto it=imu.rbegin(); it!=imu.rend(); ++it)
    if (it->time < anchor.time && it->time > begin) targets.push_back(it->time);
  if (begin < anchor.time) targets.push_back(begin);
  for (double t: targets) {
    Matrix19 f;
    if (!motionStep(k.state, imu, t, gap, f)) return false;
    k.phi = (f*k.phi).eval(); knots_.push_back(k);
  }
  std::reverse(knots_.begin(), knots_.end());
  k = Knot(); k.state = anchor;
  targets.clear();
  for (const auto& i: imu) if (i.time > anchor.time && i.time < end) targets.push_back(i.time);
  if (end > anchor.time) targets.push_back(end);
  for (double t: targets) {
    Matrix19 f;
    if (!motionStep(k.state, imu, t, gap, f)) return false;
    k.phi = (f*k.phi).eval(); knots_.push_back(k);
  }
  return true;
}
bool Trajectory::evaluate(double t, State& state, Matrix19* sensitivity) const {
  if (knots_.empty() || !std::isfinite(t) || t < knots_.front().state.time || t > knots_.back().state.time) return false;
  auto it = std::upper_bound(knots_.begin(), knots_.end(), t,
                            [](double v, const Knot& k) { return v < k.state.time; });
  if (it == knots_.begin()) return false;
  --it; state = it->state;
  Matrix19 f;
  if (!motionStep(state, *imu_, t, max_gap_, f)) return false;
  if (sensitivity) *sensitivity = f*it->phi;
  return true;
}
bool Trajectory::point(const TimedPoint& p, double td, Eigen::Vector3d& world,
                       Eigen::Matrix<double,3,19>* jac) const {
  State s; Matrix19 phi;
  if (!p.point.allFinite() || !evaluate(p.time+td, s, jac ? &phi : nullptr)) return false;
  world = s.R*p.point+s.p;
  if (jac) {
    Eigen::Matrix<double,3,19> h = Eigen::Matrix<double,3,19>::Zero();
    h.block<3,3>(0,0).setIdentity(); h.block<3,3>(0,6) = -s.R*skew(p.point);
    *jac = h*phi;
    Imu sample;
    if (!interpolate(*imu_, p.time+td, max_gap_, sample)) return false;
    // Endpoint derivative; the fixed IMU anchor does NOT move with td.
    jac->col(18) = s.v+s.R*((sample.gyro-s.bg).cross(p.point));
  }
  return true;
}
} // namespace agrobot_time
