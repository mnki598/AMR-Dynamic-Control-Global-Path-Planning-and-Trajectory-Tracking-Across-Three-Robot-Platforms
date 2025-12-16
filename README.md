# Robot-dynamics-path-planning-control

Dynamic control, global path planning, and trajectory tracking for skid-steer, unicycle, and mecanum robots.

# Mecanum Wheel Robot Trajectory Tracking (Lyapunov-Based PD Control) Overview

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



# A* Path Planning and Unicycle Tracking Simulation

This repository presents a complete motion planning and control pipeline for a unicycle-type mobile robot operating in a 2D environment with static obstacles. The framework integrates grid-based A* path planning, collision-free path smoothing, and closed-loop trajectory tracking using a pure pursuit controller.

## 1. Problem Description

The objective is to autonomously navigate a unicycle robot from a given start pose to a goal pose while avoiding obstacles. The environment contains both polygonal and circular obstacles, and the robot is modeled with nonholonomic unicycle kinematics. The solution must ensure:

Collision-free motion

Smooth, trackable trajectories

Feasible control inputs under actuator limits

## 2. Environment Modeling

The workspace is discretized into a 2D occupancy grid, where each cell represents a fixed area in the real world. Obstacles are rasterized onto the grid and inflated by a specified radius to account for the robot’s physical size and safety margin. This inflation guarantees that any path planned through free cells is collision-free in continuous space.

## 3. Global Path Planning (A*)

A grid-based A* algorithm is used to compute a shortest collision-free path from start to goal. Key features include:

## 4-connected or 8-connected neighborhood (optional diagonal motion)

Euclidean-distance heuristic for efficiency

Obstacle-aware expansion using the inflated occupancy grid

The output of this stage is a discrete sequence of grid cells, which is then converted into world coordinates.

## 4. Path Reconstruction and Smoothing

Since raw A* paths are piecewise and not directly suitable for control, a two-step smoothing process is applied:

Collision-free shortcutting
Random waypoint pairs along the path are tested for direct visibility. If a straight-line segment is collision-free, intermediate points are removed.

Continuous interpolation
The shortened path is re-parameterized by arc length and interpolated using cubic splines to generate a dense, smooth reference trajectory with position and heading information.

The final planned trajectory is saved as planned_path.csv for reuse.

## 5. Robot Model

The robot follows standard unicycle kinematics:

𝑥˙=𝑣cos(𝜃) ,   𝑦˙= 𝑣sin(𝜃) ,  𝜃˙ = 𝜔 

where 𝑣 and 𝜔  are the linear and angular velocities, subject to saturation limits.

## 6. Trajectory Tracking Control

Trajectory tracking is achieved using a Pure Pursuit controller:

A lookahead point is selected along the reference path

The curvature required to reach this point is computed

Linear velocity is kept nominal, while angular velocity is adjusted based on curvature

This controller is well-suited for nonholonomic systems and provides smooth convergence to the path. The simulation starts with a perturbed initial pose to demonstrate robustness.

## 7. Performance Evaluation

During simulation, the following metrics are recorded:

Robot trajectory versus reference path

Cross-track error

Heading error

Root Mean Square Error (RMSE) for tracking performance

Results are visualized in real time and saved to astar_unicycle_sim_results_final.mat.

## 8. Key Contributions

End-to-end planning and control pipeline in a single script

Obstacle inflation without external toolboxes

Collision-free smoothing suitable for nonholonomic tracking

Fully reproducible MATLAB simulation

This framework serves as a baseline for mobile robot navigation, and can be extended to differential-drive robots, dynamic obstacles, or higher-level planners (e.g., MPC or sampling-based methods).
