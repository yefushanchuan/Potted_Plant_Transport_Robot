#include "agrobot_time_sync/time_offset_motion.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace agrobot_time {
namespace {
struct Linearization {
  Matrix19 info = Matrix19::Zero();
  Vector19 gradient = Vector19::Zero();
  int count = 0;
  double square_error = 0;
};
bool linearize(const State& s, const std::vector<TimedPoint>& points,
               const std::vector<Imu>& imu, const Matcher& matcher,
               const Options& o, Linearization& out) {
  if (points.empty()) return false;
  double begin=points.front().time, end=begin;
  for (const auto& p:points) { begin=std::min(begin,p.time); end=std::max(end,p.time); }
  Trajectory trajectory;
  if (!trajectory.build(s,imu,begin+s.td,end+s.td,o.max_imu_gap)) return false;
  out = Linearization();
  for (const auto& p: points) {
    Eigen::Vector3d world; Eigen::Matrix<double,3,19> jp;
    if (!trajectory.point(p,s.td,world,&jp)) return false;
    Plane plane;
    if (!matcher(world,plane) || !plane.normal.allFinite() || !std::isfinite(plane.offset)) continue;
    const double norm=plane.normal.norm();
    if (norm < 1e-6) continue;
    plane.normal/=norm; plane.offset/=norm;
    const double residual=plane.normal.dot(world)+plane.offset;
    if (std::abs(residual)>o.max_residual) continue;
    const double weight=std::min(1.,o.huber/std::max(std::abs(residual),1e-12))/(o.point_std*o.point_std);
    const Row19 h=plane.normal.transpose()*jp;
    out.info.noalias()+=weight*h.transpose()*h;
    out.gradient.noalias()+=weight*h.transpose()*residual;
    out.square_error+=residual*residual; ++out.count;
  }
  return out.info.allFinite() && out.gradient.allFinite();
}
bool quality(const Linearization& l, size_t points, const Options& o) {
  return l.count>=o.min_points && double(l.count)/points>=o.min_fitness &&
         std::sqrt(l.square_error/std::max(l.count,1))<=o.max_rms;
}
double timeInformation(const Matrix19& b) {
  // Eliminate all six instantaneous pose directions: a nonzero time column
  // alone is NOT evidence of observability (e.g. constant velocity).
  Eigen::Matrix<double,6,6> a;
  Eigen::Matrix<double,6,1> c;
  const int idx[6]={0,1,2,6,7,8};
  for(int i=0;i<6;++i) {
    c(i)=b(idx[i],18);
    for(int j=0;j<6;++j) a(i,j)=b(idx[i],idx[j]);
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eig(a);
  if(eig.info()!=Eigen::Success) return 0;
  Eigen::Matrix<double,6,1> inv=eig.eigenvalues().unaryExpr([](double x){return x>1e-8?1/x:0.;});
  return std::max(0.,b(18,18)-c.dot(eig.eigenvectors()*inv.asDiagonal()*eig.eigenvectors().transpose()*c));
}
}

void Options::validate() const {
  for(double x:{initial_td_std,max_abs_td,max_td_step,max_imu_gap,max_scan_duration,
                accel_noise,gyro_noise,point_std,huber,max_residual,max_rms,
                excitation_window,min_excitation_span})
    if(!std::isfinite(x)||x<=0) throw std::invalid_argument("time_offset: invalid positive parameter");
  for(double x:{td_noise,accel_bias_noise,gyro_bias_noise,min_rotation_eigenvalue,
                min_translation_eigenvalue,min_gyro_std,min_accel_std,min_td_information})
    if(!std::isfinite(x)||x<0) throw std::invalid_argument("time_offset: invalid nonnegative parameter");
  if(!std::isfinite(initial_td)||std::abs(initial_td)>max_abs_td||max_td_step>max_abs_td||
     !std::isfinite(min_fitness)||min_fitness<=0||min_fitness>1||iterations<1||iterations>20||
     min_points<6||max_points<min_points||min_excitation_span>excitation_window)
    throw std::invalid_argument("time_offset: inconsistent parameters");
}
TimeOffsetFilter::TimeOffsetFilter(const Options& options):options_(options) {options_.validate();}
void TimeOffsetFilter::reset(const State& s, bool reset_td) {
  const double old_td=state_.td, old_var=covariance_(18,18);
  const bool retain=initialized_&&!reset_td;
  state_=s; state_.td=retain?old_td:options_.initial_td;
  covariance_.setZero();
  covariance_.diagonal().head<3>().setConstant(0.01);
  covariance_.diagonal().segment<3>(3).setConstant(0.1);
  covariance_.diagonal().segment<3>(6).setConstant(0.0025);
  covariance_.diagonal().segment<3>(9).setConstant(0.0001);
  covariance_.diagonal().segment<3>(12).setConstant(0.01);
  covariance_.diagonal().segment<3>(15).setConstant(0.001);
  covariance_(18,18)=retain?old_var:options_.initial_td_std*options_.initial_td_std;
  initialized_=true; excitation_.clear(); excitation_last_time_=-1;
}
bool TimeOffsetFilter::predictTo(double t,const std::vector<Imu>& samples) {
  if(!initialized_||!std::isfinite(t)||t<state_.time||t-state_.time>1.0) return false;
  Trajectory check;
  if(!check.build(state_,samples,state_.time,t,options_.max_imu_gap)) return false;
  State next=state_; Matrix19 cov=covariance_;
  std::vector<double> targets;
  for(const auto& i:samples) if(i.time>state_.time&&i.time<t) targets.push_back(i.time);
  targets.push_back(t);
  for(double end:targets) {
    const double dt=end-next.time;
    if(dt<=0) continue;
    Matrix19 f;
    if(!motionStep(next,samples,end,options_.max_imu_gap,f)) return false;
    Matrix19 q=Matrix19::Zero();
    const double av=options_.accel_noise*options_.accel_noise;
    q.block<3,3>(0,0).diagonal().setConstant(av*dt*dt*dt/3.);
    q.block<3,3>(0,3).diagonal().setConstant(av*dt*dt/2.);
    q.block<3,3>(3,0)=q.block<3,3>(0,3);
    q.block<3,3>(3,3).diagonal().setConstant(av*dt);
    q.block<3,3>(6,6).diagonal().setConstant(options_.gyro_noise*options_.gyro_noise*dt);
    q.block<3,3>(9,9).diagonal().setConstant(options_.gyro_bias_noise*options_.gyro_bias_noise*dt);
    q.block<3,3>(12,12).diagonal().setConstant(options_.accel_bias_noise*options_.accel_bias_noise*dt);
    q(18,18)=options_.td_noise*options_.td_noise*dt;
    cov=(f*cov*f.transpose()+q).eval();
  }
  if(!cov.allFinite()) return false;
  state_=next; covariance_=0.5*(cov+cov.transpose()); return true;
}
void TimeOffsetFilter::updateExcitation(const std::vector<Imu>& samples,Report& r) {
  for(const auto& i:samples) if(i.time>excitation_last_time_&&i.time<=state_.time) {
    excitation_.push_back(i); excitation_last_time_=i.time;
  }
  while(!excitation_.empty()&&excitation_.front().time<state_.time-options_.excitation_window) excitation_.pop_front();
  if(excitation_.size()<2) return;
  Eigen::Vector3d a=Eigen::Vector3d::Zero(),w=a;
  for(const auto& i:excitation_) {a+=i.acc;w+=i.gyro;}
  a/=excitation_.size();w/=excitation_.size();
  double va=0,vw=0;
  for(const auto& i:excitation_) {va+=(i.acc-a).squaredNorm();vw+=(i.gyro-w).squaredNorm();}
  r.accel_std=std::sqrt(va/excitation_.size());r.gyro_std=std::sqrt(vw/excitation_.size());
}
Report TimeOffsetFilter::update(const std::vector<TimedPoint>& points,
                                const std::vector<Imu>& samples,const Matcher& match) {
  Report report; report.td=state_.td; report.td_std=std::sqrt(covariance_(18,18));
  if(!initialized_||points.empty()) return report;
  updateExcitation(samples,report);
  const State prior=state_;
  Eigen::LDLT<Matrix19> psolve(covariance_);
  if(psolve.info()!=Eigen::Success||!psolve.isPositive()) {report.reason="invalid_covariance";return report;}
  const Matrix19 pinv=psolve.solve(Matrix19::Identity());
  Linearization l;
  if(!linearize(prior,points,samples,match,options_,l)) {report.reason="imu_coverage";return report;}
  report.matches=l.count;report.fitness=double(l.count)/points.size();
  report.rms=std::sqrt(l.square_error/std::max(1,l.count));
  if(!quality(l,points.size(),options_)) {report.reason="poor_match";return report;}
  report.td_information=timeInformation(l.info);
  const bool excited=excitation_.size()>1 &&
    excitation_.back().time-excitation_.front().time>=options_.min_excitation_span &&
    (report.gyro_std>=options_.min_gyro_std||report.accel_std>=options_.min_accel_std);
  const double translation_eigen=l.info.block<3,3>(0,0).selfadjointView<Eigen::Lower>().eigenvalues().minCoeff();
  const double rotation_eigen=l.info.block<3,3>(6,6).selfadjointView<Eigen::Lower>().eigenvalues().minCoeff();
  const bool observable=translation_eigen>=options_.min_translation_eigenvalue &&
    rotation_eigen>=options_.min_rotation_eigenvalue && report.td_information>=options_.min_td_information;
  const bool estimate=excited&&observable;
  const double td_lower=std::max(-options_.max_abs_td,prior.td-options_.max_td_step);
  const double td_upper=std::min(options_.max_abs_td,prior.td+options_.max_td_step);
  State current=prior;
  for(int iteration=0;iteration<options_.iterations;++iteration) {
    if(iteration && !linearize(current,points,samples,match,options_,l)) {report.reason="imu_coverage";return report;}
    const Vector19 e=difference(current,prior);
    const Matrix19 j=priorJacobian(e);
    const Matrix19 normal=j.transpose()*pinv*j+l.info;
    Eigen::LDLT<Matrix19> solver(normal);
    if(solver.info()!=Eigen::Success||!solver.isPositive()) {report.reason="singular_update";return report;}
    const Vector19 rhs=-j.transpose()*pinv*e-l.gradient;
    Vector19 delta=solver.solve(rhs);
    // Schmidt update: keep td mean fixed when unobservable. Its uncertainty
    // and cross-correlations remain in the gain and Joseph covariance below.
    if(!estimate) delta(18)=0;
    if(!delta.allFinite()) {report.reason="nonfinite_update";return report;}
    const double bounded_td=std::clamp(current.td+delta(18),td_lower,td_upper);
    if(estimate && bounded_td!=current.td+delta(18)) {
      // Solve the box-constrained quadratic at the active td boundary. The
      // other 18 increments must be recomputed; simply clipping delta(td)
      // would retain a pose correction calculated for a different time.
      delta(18)=bounded_td-current.td;
      const Eigen::Matrix<double,18,18> pose_normal=normal.topLeftCorner<18,18>();
      Eigen::LDLT<Eigen::Matrix<double,18,18>> conditional(pose_normal);
      if(conditional.info()!=Eigen::Success||!conditional.isPositive()) {
        report.reason="singular_bounded_update";return report;
      }
      delta.head<18>()=conditional.solve(rhs.head<18>()-normal.block<18,1>(0,18)*delta(18));
      if(!delta.allFinite()) {report.reason="nonfinite_update";return report;}
      report.td_limited=true;
    }
    State next=current; inject(next,delta);
    // One bound for the ENTIRE scan, not one independent bound per iteration.
    next.td=std::clamp(next.td,td_lower,td_upper);
    if((next.p-prior.p).norm()>1.0||difference(next,prior).segment<3>(6).norm()>0.5) {
      report.reason="pose_update_limit";return report;
    }
    current=next;report.iterations=iteration+1;
    if(delta.head<18>().norm()<1e-5&&std::abs(delta(18))<1e-6) break;
  }
  if(!linearize(current,points,samples,match,options_,l)||!quality(l,points.size(),options_)) {
    report.reason="poor_final_match";return report;
  }
  // One covariance update per scan, expressed in the FINAL tangent space.
  const Matrix19 j=priorJacobian(difference(current,prior));
  const Matrix19 ji=j.inverse();
  const Matrix19 plocal=ji*covariance_*ji.transpose();
  Eigen::LDLT<Matrix19> solver(j.transpose()*pinv*j+l.info);
  if(solver.info()!=Eigen::Success||!solver.isPositive()) {report.reason="invalid_posterior";return report;}
  Matrix19 gain_factor=solver.solve(Matrix19::Identity());
  const bool at_boundary=estimate &&
    (std::abs(current.td-td_lower)<1e-10||std::abs(current.td-td_upper)<1e-10);
  if(at_boundary) {
    // A boundary is a numerical constraint, NOT a measured time offset.
    // Use the local active-set gain for the other states, with no td gain.
    // Joseph form retains td variance and propagates its uncertainty through
    // the full measurement Jacobian/cross-covariance into the other states.
    const Matrix19 normal=j.transpose()*pinv*j+l.info;
    const Eigen::Matrix<double,18,18> active=normal.topLeftCorner<18,18>();
    Eigen::LDLT<Eigen::Matrix<double,18,18>> active_solver(active);
    if(active_solver.info()!=Eigen::Success||!active_solver.isPositive()) {
      report.reason="invalid_bounded_posterior";return report;
    }
    gain_factor.setZero();
    gain_factor.topLeftCorner<18,18>()=active_solver.solve(Eigen::Matrix<double,18,18>::Identity());
  } else if(!estimate) gain_factor.row(18).setZero();
  const Matrix19 a=Matrix19::Identity()-gain_factor*l.info;
  Matrix19 post=a*plocal*a.transpose()+gain_factor*l.info*gain_factor.transpose();
  post=(0.5*(post+post.transpose())).eval();
  if(!post.allFinite()||Eigen::LLT<Matrix19>(post).info()!=Eigen::Success) {
    report.reason="invalid_posterior";return report;
  }
  state_=current;covariance_=post;
  report.accepted=true;report.td_updated=estimate;
  report.reason=estimate?(at_boundary?"tracking_td_limited":"tracking"):
    (excited?"unobservable_geometry":"insufficient_excitation");
  report.matches=l.count;report.fitness=double(l.count)/points.size();
  report.rms=std::sqrt(l.square_error/l.count);
  report.td=state_.td;report.td_std=std::sqrt(covariance_(18,18));
  return report;
}
bool TimeOffsetFilter::poseAt(double lidar_time,const std::vector<Imu>& samples,State& out) const {
  Trajectory tr;
  const double t=lidar_time+state_.td;
  return initialized_&&tr.build(state_,samples,t,t,options_.max_imu_gap)&&tr.evaluate(t,out);
}
bool TimeOffsetFilter::deskew(const std::vector<TimedPoint>& points,double lidar_end,
                              const std::vector<Imu>& samples,std::vector<Eigen::Vector3d>& out) const {
  if(points.empty()) return false;
  double begin=lidar_end,end=lidar_end;
  for(const auto& p:points) {begin=std::min(begin,p.time);end=std::max(end,p.time);}
  Trajectory tr; State ref;
  if(!tr.build(state_,samples,begin+state_.td,end+state_.td,options_.max_imu_gap)||
     !tr.evaluate(lidar_end+state_.td,ref)) return false;
  out.clear();out.reserve(points.size());
  for(const auto& p:points) {
    Eigen::Vector3d world;
    if(!tr.point(p,state_.td,world)) return false;
    out.push_back(ref.R.transpose()*(world-ref.p));
  }
  return true;
}
} // namespace agrobot_time
