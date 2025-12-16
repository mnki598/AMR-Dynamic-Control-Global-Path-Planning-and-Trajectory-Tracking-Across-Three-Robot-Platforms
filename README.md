# Robot-dynamics-path-planning-control

Dynamic control, global path planning, and trajectory tracking for skid-steer, unicycle, and mecanum robots.

## Mecanum Wheel Robot Trajectory Tracking (Lyapunov-Based PD Control) Overview

This repository demonstrates trajectory tracking for a mecanum-wheel mobile robot using a Lyapunov-based PD control approach. A reference trajectory is recorded from real motion data and stored in CSV format. To enable stable nonlinear control, the recorded data are filtered and interpolated to obtain smooth, time-continuous reference signals.

The proposed framework achieves accurate and stable trajectory tracking while respecting realistic velocity and acceleration limits.

System Model

The robot is a four-wheel mecanum platform capable of omnidirectional motion. The body-frame velocity

𝜈 =[𝑣𝑥,𝑣𝑦,𝜔𝑧]^𝑇
is mapped to wheel speeds via a mecanum Jacobian. The robot pose

𝜂 = [x,y,ψ]^𝑇

evolves according to planar kinematics with rotation matrix 𝑅(𝜓). Physical parameters such as mass, inertia, wheel radius, and geometry are explicitly included. Viscous damping is modeled in both wheel and body dynamics to reflect real hardware behavior. 

## Trajectory Recording and Preprocessing

The reference trajectory is stored as a CSV file containing time, position, velocity, and yaw. Since recorded data are noisy and discretely sampled, the following preprocessing steps are applied:

Zero-phase Butterworth low-pass filtering to remove high-frequency noise without phase delay

Cubic spline interpolation with natural boundary conditions to obtain smooth, continuous-time reference signals

This preprocessing ensures well-behaved derivatives suitable for Lyapunov-based control.

## Control Strategy

A nonlinear PD controller is designed in the body frame using Lyapunov stability theory. The controller combines:

proportional feedback on pose errors,

derivative feedback on velocity errors,

velocity feedforward from the reference trajectory.

Acceleration feedforward is intentionally omitted due to noise sensitivity in real data. Input saturation enforces realistic actuator and velocity limits.

## Simulation and Results

Simulations are performed with a 2 ms time step and realistic physical constraints. The robot successfully tracks the recorded trajectory with smooth motion, bounded velocities, and small tracking errors. The code automatically generates:

desired vs. actual trajectories,

tracking error plots,

velocity tracking plots,

trajectory animations.

## Key Features

Lyapunov-based PD control with stability guarantees

Robust tracking of recorded (real) trajectories

Noise-aware preprocessing (filtering + spline interpolation)

Physically realistic damping and constraints

Reproducible simulations with clear visualization

## Usage

Replace the CSV file with any recorded trajectory of the same format and run the simulation script. All parameters and gains are explicitly defined for easy modification and extension.

## Extensions

This framework can be extended to real-time experiments, adaptive or finite-time control laws, and multi-robot or cooperative mecanum systems.
