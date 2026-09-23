#pragma once
#include "time_offset_filter.h"
namespace agrobot_time {
Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& v);
bool motionStep(State& state, const std::vector<Imu>& imu, double target,
                double max_gap, Matrix19& transition);
Vector19 difference(const State& state, const State& reference);
void inject(State& state, const Vector19& delta);
Matrix19 priorJacobian(const Vector19& difference);
} // namespace agrobot_time
