#include "motion/dc_motor.hpp"

#include <stdexcept>

namespace motion {
namespace {

MotorState add_scaled(const MotorState& lhs, const MotorState& rhs, double scale) {
    return {
        lhs.current + rhs.current * scale,
        lhs.angular_velocity + rhs.angular_velocity * scale,
        lhs.angular_position + rhs.angular_position * scale,
    };
}

}  // namespace

DcMotor::DcMotor(DcMotorParameters parameters) : parameters_(parameters) {
    if (parameters_.resistance <= 0.0 || parameters_.inductance <= 0.0 ||
        parameters_.torque_constant <= 0.0 ||
        parameters_.back_emf_constant <= 0.0 || parameters_.inertia <= 0.0 ||
        parameters_.viscous_friction < 0.0) {
        throw std::invalid_argument("Invalid DC motor parameters");
    }
}

void DcMotor::reset(MotorState state) {
    state_ = state;
}

MotorState DcMotor::derivative(
    const MotorState& state, double voltage, double load_torque) const {
    const double current_rate =
        (voltage - parameters_.resistance * state.current -
         parameters_.back_emf_constant * state.angular_velocity) /
        parameters_.inductance;
    const double velocity_rate =
        (parameters_.torque_constant * state.current -
         parameters_.viscous_friction * state.angular_velocity - load_torque) /
        parameters_.inertia;
    return {current_rate, velocity_rate, state.angular_velocity};
}

MotorState DcMotor::step(double voltage, double load_torque, double dt) {
    if (dt <= 0.0) {
        throw std::invalid_argument("Motor integration step must be positive");
    }

    const MotorState k1 = derivative(state_, voltage, load_torque);
    const MotorState k2 = derivative(add_scaled(state_, k1, dt * 0.5), voltage, load_torque);
    const MotorState k3 = derivative(add_scaled(state_, k2, dt * 0.5), voltage, load_torque);
    const MotorState k4 = derivative(add_scaled(state_, k3, dt), voltage, load_torque);

    state_.current += dt * (k1.current + 2.0 * k2.current + 2.0 * k3.current +
                            k4.current) /
                      6.0;
    state_.angular_velocity +=
        dt * (k1.angular_velocity + 2.0 * k2.angular_velocity +
              2.0 * k3.angular_velocity + k4.angular_velocity) /
        6.0;
    state_.angular_position +=
        dt * (k1.angular_position + 2.0 * k2.angular_position +
              2.0 * k3.angular_position + k4.angular_position) /
        6.0;
    return state_;
}

}  // namespace motion
