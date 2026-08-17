#pragma once

#include "motion/differential_drive.hpp"

namespace motion {

struct WaypointFollowerConfig {
    double position_gain{1.2};
    double heading_gain{3.0};
    double final_heading_gain{2.0};
    double max_linear_speed{0.8};
    double max_angular_speed{2.0};
    double position_tolerance{0.03};
    double heading_tolerance{0.05};
};

class WaypointFollower {
public:
    explicit WaypointFollower(WaypointFollowerConfig config = {});

    [[nodiscard]] Twist2d compute(const Pose2d& pose, const Pose2d& target) const;
    [[nodiscard]] bool reached(const Pose2d& pose, const Pose2d& target) const;

private:
    WaypointFollowerConfig config_;
};

}  // namespace motion

