#include "agrobot_time_sync/time_offset_filter.h"
#include "agrobot_time_sync/time_offset_motion.h"
#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
#include <cmath>

using namespace agrobot_time;
namespace {
State truth(double t) {
  State s; s.time=t;
  s.p={0.5*std::sin(1.3*t),0.3*std::cos(0.9*t),0};
  s.v={0.65*std::cos(1.3*t),-0.27*std::sin(0.9*t),0};
  s.R=expSO3(Eigen::Vector3d(0,0,0.5*std::sin(1.7*t)));
  return s;
}
std::vector<Imu> measurements(double start,double end) {
  std::vector<Imu> out;
  const int n=int(std::round((end-start)/0.005));
  for(int i=0;i<=n;++i) {
    double t=start+i*0.005;
    const auto s=truth(t);
    Eigen::Vector3d aw(-0.845*std::sin(1.3*t),-0.243*std::cos(0.9*t),0);
    out.push_back({t,s.R.transpose()*(aw-s.gravity),Eigen::Vector3d(0,0,0.85*std::cos(1.7*t))});
  }
  return out;
}
std::vector<TimedPoint> scan(double start,double td,int count=240) {
  std::vector<TimedPoint> out;
  for(int i=0;i<count;++i) {
    const double t=start+0.1*i/(count-1);
    Eigen::Vector3d q(2*std::sin(i*2.31),2*std::cos(i*1.73),std::sin(i*0.91));
    const int axis=i%3; q(axis)=(i%2?1:-1)*(axis==2?2.:5.);
    State s=truth(t+td);
    out.push_back({s.R.transpose()*(q-s.p),t});
  }
  return out;
}
bool room(const Eigen::Vector3d& p,Plane& plane) {
  double best=1e9;
  for(int axis=0;axis<3;++axis) for(double sign:{-1.,1.}) {
    const double d=-sign*(axis==2?2.:5.);
    const double e=std::abs(p(axis)+d);
    if(e<best) {best=e;plane.normal=Eigen::Vector3d::Unit(axis);plane.offset=d;}
  }
  return best<0.5;
}
Options testOptions() {
  Options o; o.min_fitness=0.9;o.max_rms=0.15;o.max_td_step=0.015;
  o.point_std=0.025;o.gyro_noise=0.001;o.accel_noise=0.01;
  o.min_td_information=0.05;o.iterations=5;
  return o;
}
}

TEST(TimeOffset, InterpolatesBoundariesWithoutExtrapolation) {
  std::vector<Imu> s={{1,{0,0,0},{1,0,0}},{1.01,{2,0,0},{3,0,0}}};
  Imu out;
  ASSERT_TRUE(interpolate(s,1.005,0.02,out)); EXPECT_NEAR(out.acc.x(),1,1e-10);
  EXPECT_TRUE(interpolate(s,1,0.02,out)); EXPECT_TRUE(interpolate(s,1.01,0.02,out));
  EXPECT_FALSE(interpolate(s,0.999,0.02,out)); EXPECT_FALSE(interpolate(s,1.02,0.02,out));
  EXPECT_FALSE(interpolate(s,1.005,0.002,out));
}
TEST(TimeOffset, RejectsGapsAndDuplicateTimesTransactionally) {
  TimeOffsetFilter f; State s; s.time=1;f.reset(s);
  std::vector<Imu> imu={{1,{0,0,9.8},{0,0,0}},{1.1,{0,0,9.8},{0,0,0}}};
  const auto before=f.covariance();
  EXPECT_FALSE(f.predictTo(1.1,imu)); EXPECT_EQ(f.state().time,1);
  EXPECT_EQ((before-f.covariance()).norm(),0);
  imu[1].time=1;EXPECT_FALSE(f.predictTo(1,imu));
}
TEST(TimeOffset, AllNineteenPointJacobiansAgreeWithNumericalPerturbations) {
  auto imu=measurements(0,0.5);State anchor=truth(0.3);anchor.td=0.012;
  const TimedPoint point{{3,-1,0.5},0.21};
  Trajectory tr;ASSERT_TRUE(tr.build(anchor,imu,0.1,0.4,0.02));
  Eigen::Vector3d world;Eigen::Matrix<double,3,19> analytic;
  ASSERT_TRUE(tr.point(point,anchor.td,world,&analytic));
  for(int j=0;j<19;++j) {
    const double eps=1e-6; Vector19 d=Vector19::Zero();d(j)=eps;
    State plus=anchor,minus=anchor;inject(plus,d);inject(minus,-d);
    Trajectory tp,tm;
    ASSERT_TRUE(tp.build(plus,imu,0.1,0.4,0.02));ASSERT_TRUE(tm.build(minus,imu,0.1,0.4,0.02));
    Eigen::Vector3d wp,wm;ASSERT_TRUE(tp.point(point,plus.td,wp));ASSERT_TRUE(tm.point(point,minus.td,wm));
    EXPECT_LT(((wp-wm)/(2*eps)-analytic.col(j)).norm(),j==18?5e-4:1e-5) << "column " << j;
  }
}
TEST(TimeOffset, StationaryTimeStateDoesNotBecomeOverconfident) {
  auto o=testOptions();TimeOffsetFilter f(o);State s;s.time=0;s.p={0,0.3,0};f.reset(s);
  std::vector<Imu> imu;
  for(int i=0;i<=240;++i) imu.push_back({i*0.005,{0,0,9.8},{0,0,0}});
  auto points=scan(0.9,0);
  // Generate measurements of a genuinely stationary sensor.
  for(auto& p:points) {auto original=truth(p.time);p.point=original.R*p.point+original.p-s.p;}
  ASSERT_TRUE(f.predictTo(1,imu));double variance=f.covariance()(18,18);
  auto r=f.update(points,imu,room);
  ASSERT_TRUE(r.accepted)<<r.reason;EXPECT_FALSE(r.td_updated);
  EXPECT_DOUBLE_EQ(f.state().td,0);EXPECT_NEAR(f.covariance()(18,18),variance,1e-12);
}
TEST(TimeOffset, ConstantMotionIsNotTimeExcitation) {
  auto o=testOptions();TimeOffsetFilter f(o);State s;s.time=0;s.v={0.2,0,0};f.reset(s);
  std::vector<Imu> imu;for(int i=0;i<=240;++i)imu.push_back({i*0.005,{0,0,9.8},{0,0,0}});
  auto points=scan(0.9,0);
  for(auto& p:points){auto original=truth(p.time);p.point=original.R*p.point+original.p-s.v*p.time;}
  ASSERT_TRUE(f.predictTo(1,imu));auto r=f.update(points,imu,room);
  ASSERT_TRUE(r.accepted)<<r.reason;EXPECT_FALSE(r.td_updated);EXPECT_LT(r.td_information,1e-5);
}
TEST(TimeOffset, RecoversPositiveAndNegativeOffsets) {
  const auto imu=measurements(-0.2,12.5);
  for(double offset:{0.02,-0.02}) {
    TimeOffsetFilter f(testOptions());f.reset(truth(0));int updates=0,accepted=0;
    for(int frame=1;frame<=120;++frame) {
      double end=frame*0.1;
      ASSERT_TRUE(f.predictTo(end+f.state().td,imu));
      auto r=f.update(scan(end-0.1,offset),imu,room);
      updates+=r.td_updated;accepted+=r.accepted;
    }
    EXPECT_GT(updates,70);EXPECT_GT(accepted,110);
    EXPECT_NEAR(f.state().td,offset,0.003) << "offset="<<offset;
    EXPECT_GT(f.covariance().selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),0);
  }
}
TEST(TimeOffset, TracksSlowlyChangingOffset) {
  auto o=testOptions();o.td_noise=0.001;
  TimeOffsetFilter f(o);f.reset(truth(0));const auto imu=measurements(-0.2,20.5);
  for(int frame=1;frame<=200;++frame) {
    const double end=frame*0.1,offset=0.005+0.0005*end;
    ASSERT_TRUE(f.predictTo(end+f.state().td,imu));
    f.update(scan(end-0.1,offset),imu,room);
  }
  EXPECT_NEAR(f.state().td,0.015,0.003);
}
TEST(TimeOffset, BoundedJointUpdatesConvergeWithoutDroppingLargeOffsetScans) {
  const auto imu=measurements(-0.3,15.5);
  for(double offset:{0.06,-0.06}) {
    auto o=testOptions();o.max_td_step=0.003;o.initial_td_std=0.06;
    TimeOffsetFilter f(o);f.reset(truth(0));int bounded=0,accepted=0;
    for(int frame=1;frame<=150;++frame) {
      const double end=frame*0.1;
      ASSERT_TRUE(f.predictTo(end+f.state().td,imu));
      const double previous=f.state().td,variance=f.covariance()(18,18);
      const auto r=f.update(scan(end-0.1,offset),imu,room);
      accepted+=r.accepted;bounded+=r.td_limited;
      EXPECT_NE(r.reason,"td_update_limit");
      EXPECT_LE(std::abs(f.state().td-previous),o.max_td_step+1e-12);
      if(r.reason=="tracking_td_limited") {
        EXPECT_TRUE(r.accepted);
        EXPECT_NEAR(f.covariance()(18,18),variance,1e-12);
      }
      EXPECT_GT(f.covariance().selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),0);
    }
    EXPECT_GT(bounded,5);EXPECT_GE(accepted,145);
    EXPECT_NEAR(f.state().td,offset,0.003);
    State end_pose;ASSERT_TRUE(f.poseAt(15.,imu,end_pose));
    EXPECT_LT((end_pose.p-truth(15.+offset).p).norm(),0.03);
  }
}
TEST(TimeOffset, AbsoluteTimeBoundaryDoesNotBecomeAPerfectTimeMeasurement) {
  auto o=testOptions();o.max_abs_td=0.01;o.max_td_step=0.003;
  TimeOffsetFilter f(o);f.reset(truth(0));auto imu=measurements(-0.2,5.5);
  int at_boundary=0;
  for(int frame=1;frame<=50;++frame) {
    const double end=frame*0.1;
    ASSERT_TRUE(f.predictTo(end+f.state().td,imu));
    const double before=f.covariance()(18,18);
    const auto r=f.update(scan(end-0.1,0.04),imu,room);
    EXPECT_LE(std::abs(f.state().td),o.max_abs_td+1e-12);
    if(r.accepted&&std::abs(f.state().td-o.max_abs_td)<1e-10) {
      ++at_boundary;EXPECT_NEAR(f.covariance()(18,18),before,1e-12);
    }
  }
  EXPECT_GT(at_boundary,10);
}
TEST(TimeOffset, BadMatchingDoesNotChangeStateOrCovariance) {
  TimeOffsetFilter f(testOptions());f.reset(truth(0));auto imu=measurements(-0.2,1);
  ASSERT_TRUE(f.predictTo(0.5,imu));const auto p=f.covariance();const auto s=f.state();
  auto r=f.update(scan(0.4,0.02),imu,[](const Eigen::Vector3d&,Plane&){return false;});
  EXPECT_FALSE(r.accepted);EXPECT_EQ(r.reason,"poor_match");
  EXPECT_EQ((p-f.covariance()).norm(),0);EXPECT_EQ(difference(f.state(),s).norm(),0);
}
TEST(TimeOffset, RelocalizationRetainsCalibrationButClearsCrossCovariance) {
  auto o=testOptions();o.initial_td=0.015;TimeOffsetFilter f(o);f.reset(truth(0));
  const double variance=f.covariance()(18,18);State jump=truth(2);jump.p.x()+=3;
  f.reset(jump,false);EXPECT_DOUBLE_EQ(f.state().td,0.015);
  EXPECT_DOUBLE_EQ(f.covariance()(18,18),variance);
  EXPECT_EQ(f.covariance().row(18).head<18>().norm(),0);
  EXPECT_EQ(f.state().p.x(),jump.p.x());
}
TEST(TimeOffset, DeskewUsesUpdatedTimeAndPreservesInput) {
  auto o=testOptions();o.initial_td=0.02;TimeOffsetFilter f(o);f.reset(truth(0.42));
  auto imu=measurements(0,0.8);auto points=scan(0.3,0.02);const auto original=points;
  std::vector<Eigen::Vector3d> deskewed;ASSERT_TRUE(f.deskew(points,0.4,imu,deskewed));
  for(size_t i=0;i<points.size();++i) {
    const auto at=truth(points[i].time+0.02),ref=truth(0.42);
    const Eigen::Vector3d expected=ref.R.transpose()*(at.R*points[i].point+at.p-ref.p);
    EXPECT_LT((deskewed[i]-expected).norm(),1e-4);
    EXPECT_EQ((points[i].point-original[i].point).norm(),0);
  }
}
TEST(TimeOffset, InvalidConfigurationIsRejected) {
  Options o;o.td_noise=-1;EXPECT_THROW(TimeOffsetFilter f(o),std::invalid_argument);
  o=Options();o.initial_td=10;EXPECT_THROW(TimeOffsetFilter f(o),std::invalid_argument);
}
