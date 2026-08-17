#include "motion/trapezoidal_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion {

TrapezoidalProfile::TrapezoidalProfile(
    double distance, double max_velocity, double max_acceleration)
    : direction_(distance < 0.0 ? -1.0 : 1.0),
      distance_(std::abs(distance)),
      acceleration_(max_acceleration) {
    if (max_velocity <= 0.0 || max_acceleration <= 0.0) {
        throw std::invalid_argument("Profile limits must be positive");
    }
    if (distance_ == 0.0) {
        return;
    }

    const double nominal_acceleration_time = max_velocity / max_acceleration;
    const double nominal_acceleration_distance =
        0.5 * max_acceleration * nominal_acceleration_time * nominal_acceleration_time;

    if (2.0 * nominal_acceleration_distance >= distance_) {
        acceleration_time_ = std::sqrt(distance_ / max_acceleration);
        peak_velocity_ = max_acceleration * acceleration_time_;
        cruise_time_ = 0.0;
    } else {
        acceleration_time_ = nominal_acceleration_time;
        peak_velocity_ = max_velocity;
        cruise_time_ =
            (distance_ - 2.0 * nominal_acceleration_distance) / max_velocity;
    }
    acceleration_distance_ =
        0.5 * acceleration_ * acceleration_time_ * acceleration_time_;
    total_time_ = 2.0 * acceleration_time_ + cruise_time_;
}

MotionState TrapezoidalProfile::sample(double time) const {
    if (time <= 0.0 || distance_ == 0.0) {
        return {};
    }
    if (time >= total_time_) {
        return {direction_ * distance_, 0.0, 0.0};
    }

    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    if (time < acceleration_time_) {
        acceleration = acceleration_;
        velocity = acceleration_ * time;
        position = 0.5 * acceleration_ * time * time;
    } else if (time < acceleration_time_ + cruise_time_) {
        const double cruise_elapsed = time - acceleration_time_;
        velocity = peak_velocity_;
        position = acceleration_distance_ + peak_velocity_ * cruise_elapsed;
    } else {
        const double deceleration_elapsed =
            time - acceleration_time_ - cruise_time_;
        acceleration = -acceleration_;
        velocity = peak_velocity_ - acceleration_ * deceleration_elapsed;
        position = acceleration_distance_ + peak_velocity_ * cruise_time_ +
                   peak_velocity_ * deceleration_elapsed -
                   0.5 * acceleration_ * deceleration_elapsed * deceleration_elapsed;
    }
    return {direction_ * position, direction_ * velocity, direction_ * acceleration};
}

}  // namespace motion

