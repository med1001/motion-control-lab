#pragma once

namespace motion {

struct DcMotorParameters {
    double resistance{1.0};
    double inductance{0.5};
    double torque_constant{0.1};
    double back_emf_constant{0.1};
    double inertia{0.01};
    double viscous_friction{0.01};
};

struct MotorState {
    double current{0.0};
    double angular_velocity{0.0};
    double angular_position{0.0};
};

class DcMotor {
public:
    explicit DcMotor(DcMotorParameters parameters = {});

    [[nodiscard]] const MotorState& state() const noexcept { return state_; }
    [[nodiscard]] const DcMotorParameters& parameters() const noexcept {
        return parameters_;
    }
    void reset(MotorState state = {});
    [[nodiscard]] MotorState step(double voltage, double load_torque, double dt);

private:
    [[nodiscard]] MotorState derivative(
        const MotorState& state, double voltage, double load_torque) const;

    DcMotorParameters parameters_;
    MotorState state_;
};

}  // namespace motion

