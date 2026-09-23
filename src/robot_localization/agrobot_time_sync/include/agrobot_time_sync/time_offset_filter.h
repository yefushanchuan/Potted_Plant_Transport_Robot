#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace agrobot_time {

// Right SO(3) error; order: p, v, theta, bg, ba, gravity, td.
// All times are seconds. t_imu_ros = t_lidar + td. IMU headers are unchanged.
using Matrix19 = Eigen::Matrix<double, 19, 19>;
using Vector19 = Eigen::Matrix<double, 19, 1>;
using Row19 = Eigen::Matrix<double, 1, 19>;
struct Imu {
  double time = 0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};
struct State {
  double time = 0, td = 0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero(), ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.8);
};
struct TimedPoint {
  Eigen::Vector3d point = Eigen::Vector3d::Zero(); // base frame at acquisition
  double time = 0;                              // LiDAR clock, NOT reception
};
struct Plane { Eigen::Vector3d normal; double offset = 0; };
using Matcher = std::function<bool(const Eigen::Vector3d&, Plane&)>;

struct Options {
  double initial_td = 0, initial_td_std = 0.02;
  double td_noise = 0.0005; // random-walk density: seconds / sqrt(second)
  double max_abs_td = 0.1, max_td_step = 0.005;
  double max_imu_gap = 0.05, max_scan_duration = 0.25;
  double accel_noise = 0.15, gyro_noise = 0.01;
  double accel_bias_noise = 0.001, gyro_bias_noise = 0.0001;
  double point_std = 0.08, huber = 0.15, max_residual = 0.5;
  double min_fitness = 0.6, max_rms = 0.2;
  double min_rotation_eigenvalue = 0.1, min_translation_eigenvalue = 0.1;
  double excitation_window = 1.0, min_excitation_span = 0.5;
  double min_gyro_std = 0.03, min_accel_std = 0.15;
  double min_td_information = 1.0;
  int min_points = 30, max_points = 1200, iterations = 4;
  void validate() const;
};
struct Report {
  bool accepted = false, td_updated = false;
  bool td_limited = false;
  std::string reason = "not_initialized";
  int matches = 0, iterations = 0;
  double fitness = 0, rms = 0, td = 0, td_std = 0;
  double gyro_std = 0, accel_std = 0, td_information = 0;
};

Eigen::Matrix3d skew(const Eigen::Vector3d& v);
Eigen::Matrix3d expSO3(const Eigen::Vector3d& v);
bool interpolate(const std::vector<Imu>& samples, double t, double max_gap, Imu& out);

// Deterministic motion and its full state sensitivity from a FIXED IMU-time
// anchor. Backward evaluation is for observations only, never covariance.
class Trajectory {
 public:
  bool build(const State& anchor, const std::vector<Imu>& imu,
             double begin, double end, double max_gap);
  bool evaluate(double t, State& state, Matrix19* sensitivity = nullptr) const;
  bool point(const TimedPoint& point, double td, Eigen::Vector3d& world,
             Eigen::Matrix<double, 3, 19>* jacobian = nullptr) const;
 private:
  struct Knot { State state; Matrix19 phi = Matrix19::Identity(); };
  std::vector<Knot> knots_;
  const std::vector<Imu>* imu_ = nullptr;
  double max_gap_ = 0;
};

class TimeOffsetFilter {
 public:
  explicit TimeOffsetFilter(const Options& options = Options());
  void reset(const State& state, bool reset_time_offset = true);
  bool initialized() const { return initialized_; }
  const State& state() const { return state_; }
  const Matrix19& covariance() const { return covariance_; }
  const Options& options() const { return options_; }
  bool predictTo(double t, const std::vector<Imu>& samples);
  Report update(const std::vector<TimedPoint>& points, const std::vector<Imu>& samples,
                const Matcher& match);
  bool poseAt(double lidar_time, const std::vector<Imu>& samples, State& output) const;
  bool deskew(const std::vector<TimedPoint>& points, double lidar_end,
              const std::vector<Imu>& samples, std::vector<Eigen::Vector3d>& output) const;
 private:
  Options options_;
  State state_;
  Matrix19 covariance_ = Matrix19::Identity();
  bool initialized_ = false;
  std::deque<Imu> excitation_;
  double excitation_last_time_ = -1;
  void updateExcitation(const std::vector<Imu>& samples, Report& report);
};
} // namespace agrobot_time
