#include "motion/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion {

SafetySupervisor::SafetySupervisor(SafetyLimits limits) : limits_(limits) {
    if (limits_.command_timeout <= 0.0 || limits_.max_current <= 0.0 ||
        limits_.max_wheel_speed <= 0.0) {
        throw std::invalid_argument("Safety limits must be positive");
    }
}

FaultCode SafetySupervisor::evaluate(
    double now, double last_command_time, const MotorFeedback& feedback) {
    if (fault_ != FaultCode::none) {
        return fault_;
    }
    const double values[] = {
        feedback.left_velocity,
        feedback.right_velocity,
        feedback.left_current,
        feedback.right_current,
    };
    for (double value : values) {
        if (!std::isfinite(value)) {
            fault_ = FaultCode::invalid_feedback;
            return fault_;
        }
    }
    if (now - last_command_time > limits_.command_timeout) {
        fault_ = FaultCode::command_timeout;
    } else if (std::max(std::abs(feedback.left_current),
                        std::abs(feedback.right_current)) > limits_.max_current) {
        fault_ = FaultCode::over_current;
    } else if (std::max(std::abs(feedback.left_velocity),
                        std::abs(feedback.right_velocity)) >
               limits_.max_wheel_speed) {
        fault_ = FaultCode::over_speed;
    }
    return fault_;
}

std::string_view to_string(FaultCode fault) noexcept {
    switch (fault) {
        case FaultCode::none:
            return "none";
        case FaultCode::command_timeout:
            return "command_timeout";
        case FaultCode::over_current:
            return "over_current";
        case FaultCode::over_speed:
            return "over_speed";
        case FaultCode::invalid_feedback:
            return "invalid_feedback";
    }
    return "unknown";
}

}  // namespace motion

