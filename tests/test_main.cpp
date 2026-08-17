#include "motion/dc_motor.hpp"
#include "motion/differential_drive.hpp"
#include "motion/drivetrain.hpp"
#include "motion/pid_controller.hpp"
#include "motion/safety_supervisor.hpp"
#include "motion/trapezoidal_profile.hpp"
#include "motion/waypoint_follower.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void check_near(double actual, double expected, double tolerance, const std::string& message) {
    check(std::abs(actual - expected) <= tolerance,
          message + " actual=" + std::to_string(actual) +
              " expected=" + std::to_string(expected));
}

void test_pid_controller() {
    motion::PidController controller({1.0, 1.0, 0.1, -12.0, 12.0, 0.02});
    for (int index = 0; index < 500; ++index) {
        const auto output = controller.update(100.0, 0.0, 0.01);
        check(output.command <= 12.0, "PID respects positive output limit");
    }
    const auto recovered = controller.update(0.0, 0.0, 0.01);
    check_near(recovered.command, 0.0, 1e-12, "Anti-windup recovers from saturation");
}

void test_motor_model() {
    motion::DcMotor motor;
    for (int index = 0; index < 1000; ++index) {
        (void)motor.step(12.0, 0.0, 0.001);
    }
    check(motor.state().angular_velocity > 0.0, "Positive voltage accelerates motor");
    check(motor.state().current > 0.0, "Positive voltage produces current");
}

void test_trapezoidal_profile() {
    const motion::TrapezoidalProfile profile(2.0, 1.0, 2.0);
    const auto start = profile.sample(0.0);
    const auto middle = profile.sample(profile.duration() * 0.5);
    const auto end = profile.sample(profile.duration());
    check_near(start.position, 0.0, 1e-12, "Profile starts at zero");
    check(middle.velocity > 0.0, "Profile has positive velocity mid-motion");
    check_near(end.position, 2.0, 1e-12, "Profile reaches target distance");
    check_near(end.velocity, 0.0, 1e-12, "Profile stops at target");
}

void test_kinematics_and_odometry() {
    const motion::DifferentialDrive drive(0.08, 0.42);
    const motion::Twist2d requested{0.5, 0.8};
    const auto wheels = drive.inverse_kinematics(requested);
    const auto recovered = drive.forward_kinematics(wheels);
    check_near(recovered.linear, requested.linear, 1e-12, "Linear kinematics round trip");
    check_near(recovered.angular, requested.angular, 1e-12, "Angular kinematics round trip");

    const auto straight = drive.integrate({}, 1.0, 1.0);
    check_near(straight.x, 1.0, 1e-12, "Equal wheel travel moves straight");
    check_near(straight.y, 0.0, 1e-12, "Straight motion has no lateral drift");

    motion::EncoderOdometry odometry(drive, 2048.0);
    const auto estimate = odometry.update(2048, 2048);
    check_near(estimate.x, 2.0 * std::numbers::pi * 0.08, 1e-9,
               "One encoder revolution maps to wheel circumference");
}

void test_waypoint_follower() {
    const motion::WaypointFollower follower;
    const motion::Pose2d target{1.0, 0.0, 0.0};
    const auto command = follower.compute({}, target);
    check(command.linear > 0.0, "Waypoint follower commands forward motion");
    check_near(command.angular, 0.0, 1e-12, "Aligned waypoint needs no rotation");
    check(follower.reached(target, target), "Target pose is detected as reached");
}

void test_safety_supervisor() {
    motion::SafetySupervisor supervisor({0.1, 10.0, 20.0});
    motion::MotorFeedback feedback;
    check(supervisor.evaluate(0.05, 0.0, feedback) == motion::FaultCode::none,
          "Fresh command remains healthy");
    check(supervisor.evaluate(0.2, 0.0, feedback) ==
              motion::FaultCode::command_timeout,
          "Stale command trips watchdog");
    check(!supervisor.healthy(), "Safety faults latch");

    supervisor.reset();
    feedback.left_current = std::numeric_limits<double>::quiet_NaN();
    check(supervisor.evaluate(0.0, 0.0, feedback) ==
              motion::FaultCode::invalid_feedback,
          "Non-finite feedback is rejected");
}

void test_closed_loop_drivetrain() {
    const motion::DcMotorParameters parameters{0.8, 0.02, 0.12, 0.12, 0.003, 0.003};
    const motion::PidConfig pid{0.9, 3.5, 0.012, -24.0, 24.0, 0.015};
    motion::SimulatedDrivetrain drivetrain(parameters, pid, pid);
    drivetrain.write({1, 10.0, 10.0, true});
    motion::MotorFeedback feedback;
    for (int index = 0; index < 2000; ++index) {
        feedback = drivetrain.step(static_cast<double>(index) * 0.001, 0.001, 0.0, 0.0);
    }
    check_near(feedback.left_velocity, 10.0, 0.5, "Left axis tracks velocity command");
    check_near(feedback.right_velocity, 10.0, 0.5, "Right axis tracks velocity command");
}

}  // namespace

int main() {
    test_pid_controller();
    test_motor_model();
    test_trapezoidal_profile();
    test_kinematics_and_odometry();
    test_waypoint_follower();
    test_safety_supervisor();
    test_closed_loop_drivetrain();

    if (failures == 0) {
        std::cout << "All motion-control tests passed\n";
        return 0;
    }
    std::cerr << failures << " test assertion(s) failed\n";
    return 1;
}
