# Rover Drivetrain Reference

> **These values are estimates and may change.** They were measured or estimated from the
> current working CAD (Onshape), which is still being modified. Before relying on a value in
> code or simulation, check with the DCS (Driveline, Chassis, Suspension) lead that it is
> still current, and update this file when it changes.

Last updated: November 2026.

## Layout

- 4 wheels, all driven. Front-left and front-right steer; rear wheels are fixed.
- Suspension: two-arm rocker (one arm per side, no bogie). The two arms are coupled by a
  gear differential, so one arm rotating up drives the other down.
- **Driving mode:** front steering is position-controlled within ±100° (software limit, no
  hard stops; the limit protects the wiring that crosses the steering joint).
- **Autonomy mode:** front steering is locked at 0° and the rover skid-steers.

## Geometry

| Parameter | Value | SI |
|---|---|---|
| Wheel radius | 4.25 in | 0.1080 m |
| Wheel width | 4.25 in | 0.1080 m |
| Wheelbase (front to rear axle, center to center) | 29.962 in | 0.7610 m |
| Track width (left to right wheel centers) | 19.736 in | 0.5013 m |
| Chassis body (L × W × H) | 16 × 14 × 8 in | 0.406 × 0.356 × 0.203 m |
| Overall rover (L × W × H) | 38.46 × 23.99 × 20.74 in | 0.977 × 0.609 × 0.527 m |
| Steering axis offset from wheel center | 0 (wheel centered under the steering fork) | 0 m |
| Steering motor to steering shaft, center to center | 4.344 in | 0.1103 m |
| Rocker pivot height above ground | ~17.0 in | 0.432 m |
| Rocker pivot to front axle (horizontal) | ~15.5 in | 0.394 m |
| Rocker pivot to rear axle (horizontal) | ~14.5 in | 0.368 m |
| Rocker travel | ~±15° | ±0.26 rad |

## Mass

| Parameter | Value |
|---|---|
| Full rover (estimate) | ~40 kg (URC limit is 50 kg per mission) |
| Structure only, from CAD (no electronics or modules) | 9.5 kg |
| Center of mass height above ground | ~0.175 m (structure-only CAD; will rise once electronics and modules are placed) |

## Motors and controllers (CAN bus 0, 24V battery)

| CAN ID | Wheel | Function | Motor | Controller |
|---|---|---|---|---|
| 1 | Left front | Drive | NEO V1.1 (REV-21-1650) | SPARK MAX |
| 2 | Left rear | Drive | NEO V1.1 | SPARK MAX |
| 3 | Right front | Drive | NEO V1.1 | SPARK MAX |
| 4 | Right rear | Drive | NEO V1.1 | SPARK MAX |
| 5 | Left front | Steer | NEO V1.1 | SPARK MAX |
| 6 | Right front | Steer | NEO V1.1 | SPARK MAX |

The arm uses CAN IDs 51-57 on the same bus.

## Gearing and performance

| Parameter | Value |
|---|---|
| Drive gearbox | MAXPlanetary 3:1 + 4:1 + 5:1 = 60:1, 1/2 in hex output direct to the wheel |
| Drive wheel free speed | ~19.8 rad/s (~2.1 m/s) at 24V; 9.91 rad/s (1.07 m/s) at 12V |
| Drive wheel stall torque | ~156 N·m (ideal, no losses) |
| Steering gearing | 60:1 MAXPlanetary, then a 26T to 40T spur stage, ~92.3:1 total |
| Steering range | ±100° from straight-ahead |

## Encoders

- **Drive:** NEO built-in Hall encoder, 42 counts per motor revolution = 2,520 counts per
  wheel revolution (relative only).
- **Steering:** REV Through Bore Encoder (absolute) on the 40T idler gear, which turns 1:1
  with the steering shaft, so it reads the true wheel angle.

## Suggested simulation joint structure

Each side (right side mirrored):

- `chassis` → `left_rocker`: revolute, side-to-side axis
- `right_rocker` mimics `left_rocker` with multiplier −1 (models the differential)
- `left_rocker` → `front_left_steer`: revolute, vertical axis, ±100° limits
- `front_left_steer` → `front_left_wheel`: continuous
- `left_rocker` → `rear_left_wheel`: continuous

## Where the software differs (as of November 2026)

- `main` still drives 6 wheels with tank (skid) steering. 4-wheel work exists on the
  unmerged branches `tina/steering` (CAN map and motor directions) and `new-driveline`
  (ros2_control skeleton).
- `DriveTrainManager` uses a 4.5 in wheel radius (should be 4.25 in) and treats
  `roverWidth` as the full track in the differential formula, so it turns about twice as hard as intended.
