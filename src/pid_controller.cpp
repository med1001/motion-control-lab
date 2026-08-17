#include "motion/pid_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace motion {

PidController::PidController(PidConfig config) : config_(config) {
    if (config_.kp < 0.0 || config_.ki < 0.0 || config_.kd < 0.0) {
        throw std::invalid_argument("PID gains must be non-negative");
    }
    if (config_.output_min >= config_.output_max) {
        throw std::invalid_argument("PID output limits must be ordered");
    }
    if (config_.derivative_filter_time < 0.0) {
        throw std::invalid_argument("Derivative filter time must be non-negative");
    }
}

PidOutput PidController::update(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) {
        throw std::invalid_argument("PID sample time must be positive");
    }

    const double error = setpoint - measurement;
    const double proportional = config_.kp * error;

    double measurement_derivative = 0.0;
    if (previous_measurement_.has_value()) {
        measurement_derivative = (measurement - *previous_measurement_) / dt;
    }
    previous_measurement_ = measurement;

    if (config_.derivative_filter_time == 0.0) {
        filtered_measurement_derivative_ = measurement_derivative;
    } else {
        const double alpha = config_.derivative_filter_time /
                             (config_.derivative_filter_time + dt);
        filtered_measurement_derivative_ =
            alpha * filtered_measurement_derivative_ +
            (1.0 - alpha) * measurement_derivative;
    }
    const double derivative = -config_.kd * filtered_measurement_derivative_;

    const double candidate_integral_error = integral_error_ + error * dt;
    const double candidate_integral = config_.ki * candidate_integral_error;
    const double unconstrained = proportional + candidate_integral + derivative;
    const double constrained =
        std::clamp(unconstrained, config_.output_min, config_.output_max);

    const bool saturated_high = unconstrained > config_.output_max;
    const bool saturated_low = unconstrained < config_.output_min;
    const bool drives_back = (saturated_high && error < 0.0) ||
                             (saturated_low && error > 0.0);
    if ((!saturated_high && !saturated_low) || drives_back) {
        integral_error_ = candidate_integral_error;
    }

    const double integral = config_.ki * integral_error_;
    const double command = std::clamp(
        proportional + integral + derivative, config_.output_min, config_.output_max);
    return {command,
            error,
            proportional,
            integral,
            derivative,
            constrained != unconstrained};
}

void PidController::reset() {
    integral_error_ = 0.0;
    filtered_measurement_derivative_ = 0.0;
    previous_measurement_.reset();
}

}  // namespace motion

