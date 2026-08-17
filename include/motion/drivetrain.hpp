#pragma once

#include "motion/dc_motor.hpp"
#include "motion/pid_controller.hpp"

#include <cstdint>

namespace motion {

struct MotorCommand {
    std::uint64_t sequence{0};
    double left_velocity{0.0};
    double right_velocity{0.0};
    bool enable{false};
};

struct MotorFeedback {
    std::uint64_t sequence{0};
    double timestamp{0.0};
    double left_velocity{0.0};
    double right_velocity{0.0};
    double left_current{0.0};
    double right_current{0.0};
    double left_position{0.0};
    double right_position{0.0};
    double left_voltage{0.0};
    double right_voltage{0.0};
};

class MotorInterface {
public:
    virtual ~MotorInterface() = default;
    virtual void write(const MotorCommand& command) = 0;
    [[nodiscard]] virtual MotorFeedback read() const = 0;
};

class SimulatedDrivetrain final : public MotorInterface {
public:
    SimulatedDrivetrain(
        DcMotorParameters motor_parameters,
        PidConfig left_controller,
        PidConfig right_controller);

    void write(const MotorCommand& command) override;
    [[nodiscard]] MotorFeedback read() const override { return feedback_; }
    [[nodiscard]] MotorFeedback step(
        double timestamp, double dt, double left_load, double right_load);
    void emergency_stop();

private:
    DcMotor left_motor_;
    DcMotor right_motor_;
    PidController left_controller_;
    PidController right_controller_;
    MotorCommand command_;
    MotorFeedback feedback_;
};

}  // namespace motion

