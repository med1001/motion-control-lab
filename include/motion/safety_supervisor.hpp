#pragma once

#include "motion/drivetrain.hpp"

#include <string_view>

namespace motion {

enum class FaultCode {
    none,
    command_timeout,
    over_current,
    over_speed,
    invalid_feedback,
};

struct SafetyLimits {
    double command_timeout{0.25};
    double max_current{40.0};
    double max_wheel_speed{80.0};
};

class SafetySupervisor {
public:
    explicit SafetySupervisor(SafetyLimits limits = {});

    [[nodiscard]] FaultCode evaluate(
        double now, double last_command_time, const MotorFeedback& feedback);
    void reset() noexcept { fault_ = FaultCode::none; }
    [[nodiscard]] FaultCode fault() const noexcept { return fault_; }
    [[nodiscard]] bool healthy() const noexcept { return fault_ == FaultCode::none; }

private:
    SafetyLimits limits_;
    FaultCode fault_{FaultCode::none};
};

[[nodiscard]] std::string_view to_string(FaultCode fault) noexcept;

}  // namespace motion

