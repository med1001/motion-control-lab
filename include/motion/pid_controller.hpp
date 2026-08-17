#pragma once

#include <optional>
#include <utility>

namespace motion {

struct PidConfig {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double output_min{-1.0};
    double output_max{1.0};
    double derivative_filter_time{0.0};
};

struct PidOutput {
    double command{0.0};
    double error{0.0};
    double proportional{0.0};
    double integral{0.0};
    double derivative{0.0};
    bool saturated{false};
};

class PidController {
public:
    explicit PidController(PidConfig config);

    [[nodiscard]] PidOutput update(double setpoint, double measurement, double dt);
    void reset();
    [[nodiscard]] const PidConfig& config() const noexcept { return config_; }

private:
    PidConfig config_;
    double integral_error_{0.0};
    double filtered_measurement_derivative_{0.0};
    std::optional<double> previous_measurement_;
};

}  // namespace motion

