#pragma once

#include <cmath>

namespace lusi::drivetrain2027 {

struct DriveCommand {
    double front_left_velocity = 0.0;
    double rear_left_velocity = 0.0;
    double front_right_velocity = 0.0;
    double rear_right_velocity = 0.0;
    double front_left_steering = 0.0;
    double front_right_steering = 0.0;
};

struct Geometry {
    // Provisional dimensions; verify against final mechanical CAD.
    double wheel_radius_m = 0.108;
    double track_width_m = 0.5013;
};

inline DriveCommand autonomousSkidSteer(
    double linear_x_mps,
    double angular_z_radps,
    const Geometry& geometry = {})
{
    DriveCommand command{};

    if (!std::isfinite(linear_x_mps) ||
        !std::isfinite(angular_z_radps) ||
        !std::isfinite(geometry.wheel_radius_m) ||
        !std::isfinite(geometry.track_width_m) ||
        geometry.wheel_radius_m <= 0.0 ||
        geometry.track_width_m <= 0.0) {
        return command;
    }

    const double half_track = geometry.track_width_m / 2.0;

    const double left_velocity =
        (linear_x_mps - angular_z_radps * half_track) /
        geometry.wheel_radius_m;

    const double right_velocity =
        (linear_x_mps + angular_z_radps * half_track) /
        geometry.wheel_radius_m;

    command.front_left_velocity = left_velocity;
    command.rear_left_velocity = left_velocity;
    command.front_right_velocity = right_velocity;
    command.rear_right_velocity = right_velocity;

    // Autonomous mode keeps the front steering actuators centered.
    command.front_left_steering = 0.0;
    command.front_right_steering = 0.0;

    return command;
}

}  // namespace lusi::drivetrain2027
