#pragma once

namespace motion {

struct MotionState {
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

class TrapezoidalProfile {
public:
    TrapezoidalProfile(double distance, double max_velocity, double max_acceleration);

    [[nodiscard]] MotionState sample(double time) const;
    [[nodiscard]] double duration() const noexcept { return total_time_; }

private:
    double direction_{1.0};
    double distance_{0.0};
    double acceleration_{0.0};
    double peak_velocity_{0.0};
    double acceleration_time_{0.0};
    double cruise_time_{0.0};
    double total_time_{0.0};
    double acceleration_distance_{0.0};
};

}  // namespace motion

