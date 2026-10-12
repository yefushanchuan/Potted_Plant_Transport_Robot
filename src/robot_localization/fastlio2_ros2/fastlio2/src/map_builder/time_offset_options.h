#pragma once
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <stdexcept>
// Seconds; positive td means t_imu_ros = t_lidar + td.
struct TimeOffsetOptions {
    double initial_td = 0, initial_td_std = .02, td_noise = .001;
    double max_abs_td = .1, max_td_step = .005, max_imu_gap = .05;
    double min_td_information = 1., min_gyro_std = .03, min_accel_std = .5;
    // The scan-wide bound guarantees no iteration needs samples past this horizon.
    double requiredFuture(double td) const {
        return std::max(0., std::min(max_abs_td, td + max_td_step));
    }
    void load(const YAML::Node &n) {
        if (n) {
#define READ(name)                                                                                 \
    if (n[#name])                                                                                  \
        name = n[#name].as<decltype(name)>();
            READ(initial_td) READ(initial_td_std) READ(td_noise) READ(max_abs_td) READ(max_td_step)
                READ(max_imu_gap) READ(min_td_information) READ(min_gyro_std) READ(min_accel_std)
#undef READ
        }
        for (double v : {initial_td_std, max_abs_td, max_td_step, max_imu_gap})
            if (!std::isfinite(v) || v <= 0)
                throw std::invalid_argument("invalid time_offset positive parameter");
        for (double v : {td_noise, min_td_information, min_gyro_std, min_accel_std})
            if (!std::isfinite(v) || v < 0)
                throw std::invalid_argument("invalid time_offset nonnegative parameter");
        if (!std::isfinite(initial_td) || std::abs(initial_td) > max_abs_td ||
            max_td_step > max_abs_td)
            throw std::invalid_argument("invalid time_offset bounds");
    }
};
struct TimeOffsetReport {
    bool accepted = false, td_updated = false, td_limited = false;
    int matches = 0, iterations = 0;
    double fitness = 0, rms = 0, td_information = 0;
    std::string reason = "initializing";
    std::string json(double lidar_time, double td, double variance) const {
        std::ostringstream s;
        s << std::setprecision(16) << std::boolalpha;
        s << "{\"lidar_time_sec\":" << lidar_time << ",\"td_ms\":" << td * 1000
          << ",\"td_std_ms\":" << std::sqrt(std::max(0., variance)) * 1000
          << ",\"accepted\":" << accepted << ",\"td_updated\":" << td_updated
          << ",\"td_limited\":" << td_limited << ",\"matches\":" << matches
          << ",\"fitness\":" << fitness << ",\"rms\":" << rms
          << ",\"td_information\":" << td_information << ",\"iterations\":" << iterations
          << ",\"reason\":\"" << reason << "\"}";
        return s.str();
    }
};
