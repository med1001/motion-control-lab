# Parameter configuration guide

This document explains where to configure **Motion Control Lab**, what each
parameter means, and which units to use. The reference scenario values are
currently defined directly in `apps/motion_simulator.cpp`; the project does not
yet use an external configuration file.

## Overview

| Category | Type or location | Purpose |
|---|---|---|
| Simulation | Constants in `apps/motion_simulator.cpp` | Sample period, duration, and encoder resolution |
| Geometry | `DifferentialDrive` | Wheel radius and track width |
| Motor | `DcMotorParameters` | Electrical and mechanical plant model |
| Control | `PidConfig` | Wheel-speed feedback control |
| Navigation | `WaypointFollowerConfig` | Waypoint tracking behavior |
| Safety | `SafetyLimits` | Watchdog, current, and speed limits |
| 1D trajectory | `TrapezoidalProfile` | Velocity and acceleration limits |
| Scenario | `apps/motion_simulator.cpp` | Initial pose, waypoints, and disturbances |

## Where to set parameters

To customize only the reference scenario, set values in
`apps/motion_simulator.cpp` before the main loop. Avoid changing default values
in `include/motion/*.hpp` unless the library-wide defaults should change for
every user.

Prefer named initializers so each value remains associated with its field:

```cpp
const motion::DcMotorParameters motor_parameters{
    .resistance = 0.8,
    .inductance = 0.02,
    .torque_constant = 0.12,
    .back_emf_constant = 0.12,
    .inertia = 0.003,
    .viscous_friction = 0.003,
};

const motion::PidConfig velocity_pid{
    .kp = 0.9,
    .ki = 3.5,
    .kd = 0.012,
    .output_min = -24.0,
    .output_max = 24.0,
    .derivative_filter_time = 0.015,
};

const motion::WaypointFollower follower({
    .position_gain = 1.2,
    .heading_gain = 3.0,
    .final_heading_gain = 2.0,
    .max_linear_speed = 0.8,
    .max_angular_speed = 2.0,
    .position_tolerance = 0.03,
    .heading_tolerance = 0.05,
});

motion::SafetySupervisor safety({
    .command_timeout = 0.20,
    .max_current = 45.0,
    .max_wheel_speed = 90.0,
});
```

When positional aggregate initialization is used instead, values must follow
the exact field order in the corresponding header.

## Simulation parameters

These constants are defined near the top of `apps/motion_simulator.cpp`.

| Parameter | Reference value | Unit | Definition and requirement |
|---|---:|---|---|
| `kDt` | `0.002` | s | Control-loop period. `0.002 s` corresponds to `500 Hz`. Must be strictly positive. |
| `kDuration` | `30.0` | s | Total duration of the scenario. |
| `kWheelRadius` | `0.08` | m | Effective radius of each wheel. Must be strictly positive. |
| `kTrackWidth` | `0.42` | m | Lateral distance between the left and right wheel centers. Must be strictly positive. |
| `kEncoderTicksPerRevolution` | `2048.0` | ticks/rev | Encoder counts for one wheel revolution. Must be strictly positive. |

Use the effective rolling radius rather than only the nominal wheel dimension.
A radius error directly creates a distance scale error. A track-width error
primarily creates a heading error during turns.

## DC motor parameters

`DcMotorParameters` is declared in `include/motion/dc_motor.hpp`. The reference
simulator constructs its parameter set in `apps/motion_simulator.cpp`.

| Field | Library default | Simulator reference | SI unit | Definition and requirement |
|---|---:|---:|---|---|
| `resistance` | `1.0` | `0.8` | ohm (Ω) | Armature resistance. Must be strictly positive. |
| `inductance` | `0.5` | `0.02` | henry (H) | Armature inductance. Must be strictly positive. |
| `torque_constant` | `0.1` | `0.12` | N·m/A | Motor torque constant. Must be strictly positive. |
| `back_emf_constant` | `0.1` | `0.12` | V·s/rad | Back-EMF constant. Must be strictly positive. |
| `inertia` | `0.01` | `0.003` | kg·m² | Equivalent inertia referred to the modeled shaft. Must be strictly positive. |
| `viscous_friction` | `0.01` | `0.003` | N·m·s/rad | Viscous-friction coefficient. Must be non-negative. |

The model calculates current in amperes, angular velocity in rad/s, and angular
position in radians. Parameters must be referred to the same mechanical shaft
used by the wheel-radius model. When a gearbox is present, either include its
ratio in the equivalent parameters or add an explicit transmission model.

## Wheel-speed PID parameters

`PidConfig` is declared in `include/motion/pid_controller.hpp`. The reference
simulator uses the same configuration for both wheels, although
`SimulatedDrivetrain` accepts separate left and right configurations.

| Field | Reference value | Indicative unit | Definition and requirement |
|---|---:|---|---|
| `kp` | `0.9` | V/(rad/s) | Proportional gain. Must be non-negative. |
| `ki` | `3.5` | V/rad | Integral gain. Must be non-negative. |
| `kd` | `0.012` | V·s²/rad | Derivative-on-measurement gain. Must be non-negative. |
| `output_min` | `-24.0` | V | Minimum allowed command voltage. Must be less than `output_max`. |
| `output_max` | `24.0` | V | Maximum allowed command voltage. Must be greater than `output_min`. |
| `derivative_filter_time` | `0.015` | s | Derivative low-pass filter time constant. Must be non-negative; `0` disables filtering. |

The derivative term operates on the measurement, preventing a derivative kick
when the setpoint changes abruptly. Conditional anti-windup stops the integral
term from accumulating further into saturation.

To tune the two sides independently:

```cpp
const motion::PidConfig left_pid{/* left-side parameters */};
const motion::PidConfig right_pid{/* right-side parameters */};
motion::SimulatedDrivetrain drivetrain(
    motor_parameters, left_pid, right_pid);
```

## Waypoint follower parameters

`WaypointFollowerConfig` is declared in
`include/motion/waypoint_follower.hpp`. A default-constructed
`WaypointFollower` uses the following values.

| Field | Default | Unit | Definition and requirement |
|---|---:|---|---|
| `position_gain` | `1.2` | s⁻¹ | Converts distance to the waypoint into linear speed. Must be strictly positive. |
| `heading_gain` | `3.0` | s⁻¹ | Corrects the heading toward the waypoint during translation. Must be strictly positive. |
| `final_heading_gain` | `2.0` | s⁻¹ | Corrects the requested final heading. Must be strictly positive. |
| `max_linear_speed` | `0.8` | m/s | Chassis linear-speed limit. Must be strictly positive. |
| `max_angular_speed` | `2.0` | rad/s | Chassis angular-speed limit. Must be strictly positive. |
| `position_tolerance` | `0.03` | m | Maximum distance for considering the target position reached. Must be strictly positive. |
| `heading_tolerance` | `0.05` | rad | Maximum error for considering the target heading reached. Must be strictly positive. |

A smaller tolerance requests greater accuracy, but it may prevent advancement
to the next waypoint if the system oscillates or the encoder resolution is too
coarse.

## Waypoints and initial pose

Waypoints are defined in `apps/motion_simulator.cpp`:

```cpp
const std::vector<motion::Pose2d> waypoints{
    {1.0, 0.0, 0.0},
    {1.0, 1.0, std::numbers::pi / 2.0},
    {0.0, 1.0, std::numbers::pi},
    {0.0, 0.0, -std::numbers::pi / 2.0},
};
```

Each pose contains `{x, y, heading}`:

| Field | Unit | Definition |
|---|---|---|
| `x` | m | Position along the global horizontal axis. |
| `y` | m | Position along the global vertical axis. |
| `heading` | rad | Planar orientation. `0` points along `+x`; positive angles turn counterclockwise. |

Common conversions are `π/2 = 90°`, `π = 180°`, and `-π/2 = -90°`.

The true and estimated poses currently start at `{0, 0, 0}`:

```cpp
motion::Pose2d true_pose;
motion::Pose2d estimated_pose;
```

To use another initial pose, initialize both poses and align the odometry:

```cpp
motion::Pose2d true_pose{0.5, 0.5, std::numbers::pi / 2.0};
motion::Pose2d estimated_pose = true_pose;
odometry.reset(estimated_pose);
```

The waypoint list must not be empty because the simulation always accesses
`waypoints[target_index]`.

## Safety parameters

`SafetyLimits` is declared in `include/motion/safety_supervisor.hpp`. The
reference scenario uses `{0.20, 45.0, 90.0}`.

| Field | Library default | Simulator reference | Unit | Definition and requirement |
|---|---:|---:|---|---|
| `command_timeout` | `0.25` | `0.20` | s | Maximum age of the latest command. Must be strictly positive. |
| `max_current` | `40.0` | `45.0` | A | Maximum absolute current for either wheel. Must be strictly positive. |
| `max_wheel_speed` | `80.0` | `90.0` | rad/s | Maximum absolute speed for either wheel. Must be strictly positive. |

A limit violation or non-finite feedback value latches a fault. The drivetrain
is then disabled and both PID controllers are reset. Returning to a healthy
state requires an explicit call to `SafetySupervisor::reset()`.

Keep the timeout comfortably above `kDt` and include real transport delays when
a hardware adapter replaces the simulation.

## Trapezoidal motion profile

`TrapezoidalProfile`, declared in
`include/motion/trapezoidal_profile.hpp`, takes three arguments:

```cpp
motion::TrapezoidalProfile profile(
    distance,
    max_velocity,
    max_acceleration);
```

| Argument | Unit | Definition and requirement |
|---|---|---|
| `distance` | m or rad | Signed requested displacement. A negative value reverses the motion direction. |
| `max_velocity` | m/s or rad/s | Maximum absolute velocity. Must be strictly positive. |
| `max_acceleration` | m/s² or rad/s² | Maximum absolute acceleration. Must be strictly positive. |

Units depend on the modeled axis but must remain consistent across all three
arguments. A short distance automatically produces a triangular profile. A
longer distance produces a trapezoidal profile with a constant-speed phase.

The profile is available in the library and covered by tests, but it is not
currently connected to the `motion_simulator` scenario.

## Disturbance and fault injection

The reference scenario applies a load to the right wheel from 8 s to 10 s:

```cpp
const double right_load = time >= 8.0 && time < 10.0 ? 0.12 : 0.0;
```

`right_load` is a resisting torque in N·m. The arguments to
`drivetrain.step()` are the timestamp, sample period, left load, and right load:

```cpp
drivetrain.step(time, kDt, 0.0, right_load);
```

The `--inject-timeout` command-line option stops refreshing commands from 18 s
to 18.35 s. These bounds are defined in `apps/motion_simulator.cpp` and can be
changed to construct another fault scenario.

## Output and telemetry

The simulator writes `out/telemetry.csv` by default. Pass another path as the
first argument when a different destination is required:

```bash
./build/motion_simulator out/my_run.csv
./build/motion_simulator out/fault.csv --inject-timeout
```

The loop writes one row every ten iterations. With `kDt = 0.002 s`, telemetry
is therefore exported at `50 Hz`. Change the `10U` divisor in the export
condition to adjust this rate without changing the control-loop frequency.

## Runtime states and commands

The following types represent dynamic runtime data rather than fixed
configuration values:

- `MotorCommand`: sequence number, left/right speed commands, and motor-enable
  state;
- `MotorFeedback`: timestamp, speeds, currents, positions, and voltages;
- `MotorState`: internal current, angular velocity, and angular position;
- `Pose2d`, `Twist2d`, and `WheelSpeeds`: robot pose, chassis command, and wheel
  speeds.

In a physical system, the controller, encoders, and hardware driver must update
these values each cycle. The `sequence` and `timestamp` fields support detection
of stale or missing data.

## Recommended procedure after changing parameters

After each parameter change:

1. rebuild the project;
2. run the test suite;
3. run a nominal simulation;
4. inspect telemetry, saturation, and safety margins;
5. run the timeout fault scenario.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/motion_simulator out/telemetry.csv
./build/motion_simulator out/fault_telemetry.csv --inject-timeout
python tools/plot_telemetry.py out/telemetry.csv --output docs/motion_control.png
```

The reference values describe a deterministic software model. Do not apply
them to physical hardware before validating units, identifying the motor,
tuning the PID controllers, and independently verifying all safety limits.
