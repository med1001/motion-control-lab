#include "motion/differential_drive.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace motion {

double wrap_angle(double angle) {
    return std::remainder(angle, 2.0 * std::numbers::pi);
}

DifferentialDrive::DifferentialDrive(double wheel_radius, double track_width)
    : wheel_radius_(wheel_radius), track_width_(track_width) {
    if (wheel_radius_ <= 0.0 || track_width_ <= 0.0) {
        throw std::invalid_argument("Drive geometry must be positive");
    }
}

WheelSpeeds DifferentialDrive::inverse_kinematics(const Twist2d& twist) const {
    return {
        (twist.linear - 0.5 * track_width_ * twist.angular) / wheel_radius_,
        (twist.linear + 0.5 * track_width_ * twist.angular) / wheel_radius_,
    };
}

Twist2d DifferentialDrive::forward_kinematics(
    const WheelSpeeds& wheel_speeds) const {
    return {
        wheel_radius_ * (wheel_speeds.left + wheel_speeds.right) * 0.5,
        wheel_radius_ * (wheel_speeds.right - wheel_speeds.left) / track_width_,
    };
}

Pose2d DifferentialDrive::integrate(
    const Pose2d& pose, double left_distance, double right_distance) const {
    const double center_distance = 0.5 * (left_distance + right_distance);
    const double heading_change = (right_distance - left_distance) / track_width_;
    const double midpoint_heading = pose.heading + 0.5 * heading_change;
    return {
        pose.x + center_distance * std::cos(midpoint_heading),
        pose.y + center_distance * std::sin(midpoint_heading),
        wrap_angle(pose.heading + heading_change),
    };
}

EncoderOdometry::EncoderOdometry(
    DifferentialDrive drive, double ticks_per_revolution)
    : drive_(drive),
      distance_per_tick_(2.0 * std::numbers::pi * drive.wheel_radius() /
                         ticks_per_revolution) {
    if (ticks_per_revolution <= 0.0) {
        throw std::invalid_argument("Encoder resolution must be positive");
    }
}

void EncoderOdometry::reset(
    Pose2d pose, long long left_ticks, long long right_ticks) {
    pose_ = pose;
    previous_left_ticks_ = left_ticks;
    previous_right_ticks_ = right_ticks;
}

Pose2d EncoderOdometry::update(long long left_ticks, long long right_ticks) {
    const auto left_delta_ticks = left_ticks - previous_left_ticks_;
    const auto right_delta_ticks = right_ticks - previous_right_ticks_;
    previous_left_ticks_ = left_ticks;
    previous_right_ticks_ = right_ticks;
    pose_ = drive_.integrate(
        pose_,
        static_cast<double>(left_delta_ticks) * distance_per_tick_,
        static_cast<double>(right_delta_ticks) * distance_per_tick_);
    return pose_;
}

}  // namespace motion

