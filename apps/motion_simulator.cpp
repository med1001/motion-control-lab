#include "motion/differential_drive.hpp"
#include "motion/drivetrain.hpp"
#include "motion/safety_supervisor.hpp"
#include "motion/waypoint_follower.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 0.002;
constexpr double kDuration = 30.0;
constexpr double kWheelRadius = 0.08;
constexpr double kTrackWidth = 0.42;
constexpr double kEncoderTicksPerRevolution = 2048.0;

long long encoder_ticks(double angular_position) {
    return std::llround(angular_position * kEncoderTicksPerRevolution /
                        (2.0 * std::numbers::pi));
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::filesystem::path output = "out/telemetry.csv";
        bool inject_timeout = false;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--inject-timeout") {
                inject_timeout = true;
            } else {
                output = argument;
            }
        }
        if (output.has_parent_path()) {
            std::filesystem::create_directories(output.parent_path());
        }

        const motion::DcMotorParameters motor_parameters{
            0.8, 0.02, 0.12, 0.12, 0.003, 0.003};
        const motion::PidConfig velocity_pid{0.9, 3.5, 0.012, -24.0, 24.0, 0.015};
        motion::SimulatedDrivetrain drivetrain(
            motor_parameters, velocity_pid, velocity_pid);
        const motion::DifferentialDrive drive(kWheelRadius, kTrackWidth);
        motion::EncoderOdometry odometry(drive, kEncoderTicksPerRevolution);
        const motion::WaypointFollower follower;
        motion::SafetySupervisor safety({0.20, 45.0, 90.0});

        const std::vector<motion::Pose2d> waypoints{
            {1.0, 0.0, 0.0},
            {1.0, 1.0, std::numbers::pi / 2.0},
            {0.0, 1.0, std::numbers::pi},
            {0.0, 0.0, -std::numbers::pi / 2.0},
        };

        std::ofstream csv(output);
        if (!csv) {
            throw std::runtime_error("Unable to open telemetry output");
        }
        csv << "time,target_index,target_x,target_y,true_x,true_y,true_heading,"
               "odom_x,odom_y,odom_heading,left_setpoint,right_setpoint,"
               "left_speed,right_speed,left_voltage,right_voltage,left_current,"
               "right_current,right_load,fault\n";

        motion::Pose2d true_pose;
        motion::Pose2d estimated_pose;
        motion::MotorFeedback feedback;
        std::size_t target_index = 0;
        std::uint64_t sequence = 0;
        double previous_left_position = 0.0;
        double previous_right_position = 0.0;
        double last_command_time = 0.0;

        for (double time = 0.0; time <= kDuration; time += kDt) {
            const motion::Pose2d target = waypoints[target_index];
            if (follower.reached(estimated_pose, target)) {
                target_index = (target_index + 1) % waypoints.size();
            }
            const motion::Twist2d body_command =
                follower.compute(estimated_pose, waypoints[target_index]);
            const motion::WheelSpeeds wheel_command =
                drive.inverse_kinematics(body_command);

            const bool command_dropout =
                inject_timeout && time >= 18.0 && time < 18.35;
            if (!command_dropout && safety.healthy()) {
                drivetrain.write(
                    {++sequence, wheel_command.left, wheel_command.right, true});
                last_command_time = time;
            }

            const double right_load = time >= 8.0 && time < 10.0 ? 0.12 : 0.0;
            feedback = drivetrain.step(time, kDt, 0.0, right_load);
            if (safety.evaluate(time, last_command_time, feedback) !=
                motion::FaultCode::none) {
                drivetrain.emergency_stop();
            }

            const double left_delta =
                (feedback.left_position - previous_left_position) * kWheelRadius;
            const double right_delta =
                (feedback.right_position - previous_right_position) * kWheelRadius;
            previous_left_position = feedback.left_position;
            previous_right_position = feedback.right_position;
            true_pose = drive.integrate(true_pose, left_delta, right_delta);
            estimated_pose = odometry.update(
                encoder_ticks(feedback.left_position),
                encoder_ticks(feedback.right_position));

            if (static_cast<std::uint64_t>(time / kDt) % 10U == 0U) {
                csv << time << ',' << target_index << ','
                    << waypoints[target_index].x << ',' << waypoints[target_index].y
                    << ',' << true_pose.x << ',' << true_pose.y << ','
                    << true_pose.heading << ',' << estimated_pose.x << ','
                    << estimated_pose.y << ',' << estimated_pose.heading << ','
                    << wheel_command.left << ',' << wheel_command.right << ','
                    << feedback.left_velocity << ',' << feedback.right_velocity << ','
                    << feedback.left_voltage << ',' << feedback.right_voltage << ','
                    << feedback.left_current << ',' << feedback.right_current << ','
                    << right_load << ',' << motion::to_string(safety.fault()) << '\n';
            }
        }

        std::cout << "Telemetry: " << output.string() << '\n'
                  << "Final odometry: x=" << estimated_pose.x
                  << " y=" << estimated_pose.y
                  << " heading=" << estimated_pose.heading << '\n'
                  << "Safety status: " << motion::to_string(safety.fault()) << '\n';
        return safety.healthy() || inject_timeout ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "motion_simulator: " << error.what() << '\n';
        return 1;
    }
}
