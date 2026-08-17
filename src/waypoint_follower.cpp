#include "motion/waypoint_follower.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion {

WaypointFollower::WaypointFollower(WaypointFollowerConfig config) : config_(config) {
    if (config_.position_gain <= 0.0 || config_.heading_gain <= 0.0 ||
        config_.final_heading_gain <= 0.0 || config_.max_linear_speed <= 0.0 ||
        config_.max_angular_speed <= 0.0 || config_.position_tolerance <= 0.0 ||
        config_.heading_tolerance <= 0.0) {
        throw std::invalid_argument("Waypoint follower configuration must be positive");
    }
}

Twist2d WaypointFollower::compute(const Pose2d& pose, const Pose2d& target) const {
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double distance = std::hypot(dx, dy);

    if (distance <= config_.position_tolerance) {
        const double heading_error = wrap_angle(target.heading - pose.heading);
        return {
            0.0,
            std::clamp(config_.final_heading_gain * heading_error,
                       -config_.max_angular_speed,
                       config_.max_angular_speed),
        };
    }

    const double path_heading = std::atan2(dy, dx);
    const double heading_error = wrap_angle(path_heading - pose.heading);
    const double final_heading_error = wrap_angle(target.heading - pose.heading);
    const double alignment = std::max(0.0, std::cos(heading_error));
    const double linear = std::clamp(
        config_.position_gain * distance * alignment, 0.0, config_.max_linear_speed);
    const double angular = std::clamp(
        config_.heading_gain * heading_error +
            0.15 * config_.final_heading_gain * final_heading_error,
        -config_.max_angular_speed,
        config_.max_angular_speed);
    return {linear, angular};
}

bool WaypointFollower::reached(const Pose2d& pose, const Pose2d& target) const {
    return std::hypot(target.x - pose.x, target.y - pose.y) <=
               config_.position_tolerance &&
           std::abs(wrap_angle(target.heading - pose.heading)) <=
               config_.heading_tolerance;
}

}  // namespace motion
