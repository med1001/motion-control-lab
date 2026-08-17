# Architecture and design notes

## Control timing

The reference simulator uses a fixed 2 ms step. At every step it:

1. evaluates the current waypoint against encoder odometry;
2. generates a bounded body-frame linear/angular command;
3. applies differential-drive inverse kinematics;
4. updates the wheel-velocity PID controllers;
5. integrates the electrical and mechanical motor states;
6. checks feedback and command freshness through the safety supervisor;
7. quantizes wheel positions into encoder ticks;
8. updates odometry and exports decimated telemetry.

The update order is explicit to make the one-sample delays visible and
repeatable.

## Controller behavior

The wheel controllers use derivative-on-measurement rather than
derivative-on-error. A change in velocity setpoint therefore does not create a
derivative kick. The derivative estimate is low-pass filtered.

Conditional integration prevents the integral accumulator from winding up
while the voltage command is saturated, unless the current error would drive
the controller back out of saturation.

## Geometry and estimation

Body velocity is mapped to wheel angular velocity using wheel radius `r` and
track width `L`:

```text
omega_left  = (v - L * yaw_rate / 2) / r
omega_right = (v + L * yaw_rate / 2) / r
```

Odometry consumes integer encoder ticks. Wheel travel is integrated at the
midpoint heading, which reduces curvature error compared with a first-order
heading update.

## Safety state

Faults are latched and require an explicit reset. The supervisor checks:

- age of the most recent command;
- left/right current limits;
- left/right speed limits;
- non-finite feedback values.

Once a fault is detected, the drivetrain disables its command and resets both
PID controllers. The fault-injection test deliberately stops refreshing the
command timestamp and verifies the resulting watchdog state.

## Hardware adaptation

The command and feedback structures include sequence and timestamp fields to
support a real transport adapter. Hardware-specific framing, scaling, retries,
bus-off recovery, and clock synchronization belong in that adapter and are not
simulated by the current implementation.

