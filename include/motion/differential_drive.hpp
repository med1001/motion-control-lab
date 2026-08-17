#pragma once

namespace motion {

struct Pose2d {
    double x{0.0};
    double y{0.0};
    double heading{0.0};
};

struct Twist2d {
    double linear{0.0};
    double angular{0.0};
};

struct WheelSpeeds {
    double left{0.0};
    double right{0.0};
};

class DifferentialDrive {
public:
    DifferentialDrive(double wheel_radius, double track_width);

    [[nodiscard]] WheelSpeeds inverse_kinematics(const Twist2d& twist) const;
    [[nodiscard]] Twist2d forward_kinematics(const WheelSpeeds& wheel_speeds) const;
    [[nodiscard]] Pose2d integrate(
        const Pose2d& pose, double left_distance, double right_distance) const;
    [[nodiscard]] double wheel_radius() const noexcept { return wheel_radius_; }
    [[nodiscard]] double track_width() const noexcept { return track_width_; }

private:
    double wheel_radius_;
    double track_width_;
};

class EncoderOdometry {
public:
    EncoderOdometry(DifferentialDrive drive, double ticks_per_revolution);

    void reset(Pose2d pose = {}, long long left_ticks = 0, long long right_ticks = 0);
    [[nodiscard]] Pose2d update(long long left_ticks, long long right_ticks);
    [[nodiscard]] const Pose2d& pose() const noexcept { return pose_; }

private:
    DifferentialDrive drive_;
    double distance_per_tick_;
    long long previous_left_ticks_{0};
    long long previous_right_ticks_{0};
    Pose2d pose_;
};

[[nodiscard]] double wrap_angle(double angle);

}  // namespace motion

