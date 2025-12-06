% File: skid_steering_dynamics.m
% Description: Simplified skid-steer dynamics and PD tracking controller
% Author: Mayank Pandey
% Date: 2025
% Note: Public demonstration version. Full implementation private.


clear all; clc; close all;

NONHOLONOMIC = true;   % true -> skid-steer 
USE_ACCEL_FF = true;   % true -> include zeta_dot_traj (accel feedforward) in tau

%% Simulation parameters
dt = 0.1;
ts = 40;
t = 0:dt:ts;
a = 0.05;

%% Initial conditions
eta0 = [0;0;0];     % [x;y;psi]
zeta0 = [0;0;0];    % [u;v;r]

eta(:,1)  = eta0;
zeta(:,1) = zeta0;

%% Robot physical parameters
m  = 10;
Iz = 0.1;
xbc = 0; ybc = 0;

%% Wheel configuration (Differential Drive)  (unchanged from your script)
l = 0.3;
d = 0.2;

Gamma = 1/4*[a,a,a,a ;
                   0,0,0,0;
                   -a/d,-a/d,a/d,a/d];

% (Gamma_inv not used further)
Gamma_inv = [1/a,-d/a ;
                   1/a,-d/a;
                   1/a, d/a;
                   1/a,d/a];


% REFERENCE TRAJECTORY (circle and point stabilisation)

R = 2;                 % radius [m]
omega_ref = 0.2;       % angular rate
xd = R*cos(omega_ref*t);
yd = R*sin(omega_ref*t);
psid = wrapToPi(omega_ref*t + pi/2);
% xd = 2+0*t;
% yd = 3+0*t;
% psid = 0.5 + 0*t;

%% Controller gains
% Outer-loop gains (Kp) used as diag gains: [kp_x, kp_y, kp_psi]
kp_x = 1.2;
kp_y = 1.2;
kp_psi = 1.2;
Kp = diag([kp_x kp_y kp_psi]);

% Inner-loop velocity tracking gains
Kd = diag([2 2 1]);           % velocity tracking (inner loop)
 
% Additional tuning if NONHOLONOMIC true:
if NONHOLONOMIC
    % lateral-to-yaw gain
    k_y = 0.8;   % how strongly lateral error affects yaw command
else
    k_y = 0;
end


% MAIN LOOP

% Preallocate
tau = zeros(3,length(t));
kappa = zeros(4,length(t));
zeta_dot = zeros(3,length(t));

for i = 1:length(t)

    %% Current states
    u = zeta(1,i);
    v = zeta(2,i);
    r = zeta(3,i);
    psi = eta(3,i);

   
    % Desired trajectory derivatives (analytic for circle)
    
    % eta_d = [xd; yd; psid]
    eta_dot_des = [-R*omega_ref*sin(omega_ref*t(i));
                     R*omega_ref*cos(omega_ref*t(i));
                     omega_ref];

    eta_ddot_des = [-R*omega_ref^2*cos(omega_ref*t(i));
                    -R*omega_ref^2*sin(omega_ref*t(i));
                     0];


    % 1) Trajectory tracking — Desired body velocities

    e_pos = [xd(i) - eta(1,i);
             yd(i) - eta(2,i);
             wrapToPi(psid(i) - psi)];

    % Rotation matrix J(psi) and its transpose J^T = Rpsi (body from world)
    J = [cos(psi), -sin(psi), 0;
         sin(psi),  cos(psi), 0;
         0,         0,        1];
    Rpsi = J';  % equals [cos, sin; -sin, cos; 0 0 1] as you used

    e_body = Rpsi * e_pos;    % convert error to body frame

    % -----------------------------------------------------------------
    % Kinematic feedforward: desired body velocity of the reference
    % zeta_traj = J(psi)^T * eta_dot_des
    zeta_traj = Rpsi * eta_dot_des;

   
    if USE_ACCEL_FF
        r_use = zeta_traj(3); % use desired yaw rate for feedforward (better)
    else
        r_use = r;
    end
    S_r = [0, -r_use, 0;
           r_use, 0, 0;
           0, 0, 0];

    zeta_dot_traj = Rpsi * eta_ddot_des - S_r * zeta_traj;  % approx d/dt(zeta_traj)

    % Outer law: feedforward + proportional correction in body frame
    zeta_des = zeta_traj + Kp * e_body;

    % If non-holonomic: enforce v_des = 0 and use lateral error for yaw
    if NONHOLONOMIC
        u_des = zeta_des(1);
        v_des = 0;
        r_des = zeta_traj(3) + kp_psi * e_body(3) + k_y * e_body(2);
        zeta_des = [u_des; v_des; r_des];
        % Recompute zeta_dot_traj entry for yaw if desired (optional)
        % zeta_dot_traj(2) = 0; % lateral accel feedforward zero
    end


    %           2) Dynamic model computations

    % Inertia matrix D
    D = [m,0,-ybc*m;
         0,m,xbc*m;
        -ybc*m,xbc*m,Iz+m*(xbc^2+ybc^2)];

    % Nonlinear velocity terms n_v
    n_v = [-m*r*(v+xbc*r);
            m*r*(u-ybc*r);
            m*r*(xbc*u+ybc*v)];

  
    %           3) Dynamic control τ
    %           include acceleration feedforward if enabled

    if USE_ACCEL_FF
        tau(:,i) = D * ( zeta_dot_traj + Kd*(zeta_des - zeta(:,i)) ) + n_v;
    else
        tau(:,i) = D * ( Kd*(zeta_des - zeta(:,i)) ) + n_v;
    end

 
    kappa(:,i) = Gamma \ tau(:,i);   % wheel inputs F1..F4

    %% ---------------------------------------------------------------
    %           4) Propagate dynamics
    %% ---------------------------------------------------------------
    zeta_dot(:,i) = D \ (tau(:,i) - n_v);
    zeta(:,i+1) = zeta(:,i) + dt*zeta_dot(:,i);

    % Kinematic update (midpoint-ish)
    J_eta = [cos(psi),-sin(psi),0;
             sin(psi), cos(psi),0;
             0,0,1];

    eta(:,i+1) = eta(:,i) + dt*J_eta*(zeta(:,i) + dt/2*zeta_dot(:,i));
end

%% --------------------------------------------------------------------
%                   Animation
% ---------------------------------------------------------------------
lbot = 0.6; wbot = 0.4;
mr_co = [-lbot/2,lbot/2,lbot/2,-lbot/2,-lbot/2;
         -wbot/2,-wbot/2,wbot/2,wbot/2,-wbot/2];

figure; hold on; grid on; axis equal;
for i = 1:length(t)+1
    psi = eta(3,i);
    Rpsi2 = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    v_pos = Rpsi2 * mr_co;
    fill(v_pos(1,:)+eta(1,i), v_pos(2,:)+eta(2,i),'g');
    hold on;
    plot(eta(1,1:i), eta(2,1:i),'b','LineWidth',2);
    plot(xd, yd,'r--','LineWidth',1.5)
    axis([-3 3 -3 3])
    xlabel("x [m]"); ylabel("y [m]");
    pause(0.05); hold off;
end

%% Plots
figure
subplot(2,1,1)
plot(t, eta(1,1:end-1),'r', t, eta(2,1:end-1),'b', t, eta(3,1:end-1),'k');
legend('x','y','psi'); grid on; title('Pose');

subplot(2,1,2)
plot(t, zeta(1,1:end-1),'r', t, zeta(2,1:end-1),'b', t, zeta(3,1:end-1),'k');
legend('u','v','r'); grid on; title('Body velocities');

figure
plot(t, tau(1,:), 'r', t, tau(2,:), 'b', t, tau(3,:), 'k'); grid on;
legend('\tau_x','\tau_y','\tau_\psi'); title('Body Wrench (tau)');

figure
plot(t, kappa(1,:), t, kappa(2,:), t, kappa(3,:), t, kappa(4,:));
legend('F1','F2','F3','F4'); title('Wheel inputs');

%% Compute errors for plotting
e_world = zeros(3, length(t));
e_body_all = zeros(3, length(t));

for i = 1:length(t)
    psi = eta(3,i);
    Rpsi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    e_world(:,i) = [xd(i) - eta(1,i);
                    yd(i) - eta(2,i);
                    wrapToPi(psid(i) - psi)];
    e_body_all(:,i) = Rpsi' * e_world(:,i);
end

%% Plot position/orientation tracking errors (world frame)
figure;
subplot(2,1,1)
plot(t, e_world(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(t, e_world(2,:), 'b', 'LineWidth', 1.2);
plot(t, e_world(3,:), 'k', 'LineWidth', 1.2);
grid on; xlabel('Time [s]');
ylabel('Error');
legend('e_x','e_y','e_\psi');
title('Tracking error in world frame');

%% Plot body-frame errors (what controller uses)
subplot(2,1,2)
plot(t, e_body_all(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(t, e_body_all(2,:), 'b', 'LineWidth', 1.2);
plot(t, e_body_all(3,:), 'k', 'LineWidth', 1.2);
grid on; xlabel('Time [s]');
ylabel('Error (body frame)');
legend('e_{bx}','e_{by}','e_{b\psi}');
title('Tracking error in body frame');


disp('Simulation finished.');
