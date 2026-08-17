#include "motion/drivetrain.hpp"

namespace motion {

SimulatedDrivetrain::SimulatedDrivetrain(
    DcMotorParameters motor_parameters,
    PidConfig left_controller,
    PidConfig right_controller)
    : left_motor_(motor_parameters),
      right_motor_(motor_parameters),
      left_controller_(left_controller),
      right_controller_(right_controller) {}

void SimulatedDrivetrain::write(const MotorCommand& command) {
    command_ = command;
}

MotorFeedback SimulatedDrivetrain::step(
    double timestamp, double dt, double left_load, double right_load) {
    double left_voltage = 0.0;
    double right_voltage = 0.0;
    if (command_.enable) {
        left_voltage = left_controller_
                           .update(command_.left_velocity,
                                   left_motor_.state().angular_velocity,
                                   dt)
                           .command;
        right_voltage = right_controller_
                            .update(command_.right_velocity,
                                    right_motor_.state().angular_velocity,
                                    dt)
                            .command;
    } else {
        left_controller_.reset();
        right_controller_.reset();
    }

    const MotorState left = left_motor_.step(left_voltage, left_load, dt);
    const MotorState right = right_motor_.step(right_voltage, right_load, dt);
    feedback_ = {
        command_.sequence,
        timestamp,
        left.angular_velocity,
        right.angular_velocity,
        left.current,
        right.current,
        left.angular_position,
        right.angular_position,
        left_voltage,
        right_voltage,
    };
    return feedback_;
}

void SimulatedDrivetrain::emergency_stop() {
    command_.enable = false;
    command_.left_velocity = 0.0;
    command_.right_velocity = 0.0;
    left_controller_.reset();
    right_controller_.reset();
}

}  // namespace motion

