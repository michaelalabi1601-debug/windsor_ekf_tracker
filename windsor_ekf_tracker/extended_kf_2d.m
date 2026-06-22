% PROJECT: Windsor 2D EKF Tracker
% PHASE 2: 2D Kinematics & Noisy Sensor Simulation Space
clear; clc; close all;

% 1. Time and Simulation Setup
dt = 0.1;                % Time steps 
t_final = 15;            % Total time for the simulation 
t = 0: dt :t_final;      % Time vector
N = length(t);           % Number of steps 

% 2. 2D GROUND TRUTH (Real Physics)
% State vector structure: [X_position; Y_position; Heading_theta; Velocity]
x_true = zeros(4,N);
% Initial State: starts at (0,0), facing 45deg, moving at 5m/s
x_true(:,1) = [0; 0; deg2rad(45); 5];
% Control Inputs (Linear Acceleration (m/s^2); Turn rate (rad/s))
u = [0.1; deg2rad(5)]; % constant acc while turning 5 deg/s 

% 3. NOISY SENSOR SIMULATION (2D GPS TRACKER)
noise_sigma = 1.5; % measurement noise standard deviation = 1.5 m 
z_measured = zeros (2,N); % this tracks measures X and Y positions

% 4. SIMULATION LOOP (2D PHYSICS ENGINE)
for k = 2:N
    % Extract the values from previous step for Readability.
    theta_prev = x_true(3, k-1);      % Previous Heading angle
    v_prev = x_true(4, k-1);   % Previous Velocity.
    % Predicted Values using 2D linear Kinematics 
    x_true(1,k) = x_true(1, k-1) + v_prev * cos(theta_prev) * dt; 
    x_true(2,k) = x_true(2, k-1) + v_prev * sin(theta_prev) * dt;
    x_true(3,k) = theta_prev + u(2); % New heading angle = old one + 5 deg/s
    x_true(4,k) = v_prev + u(1); % New velocity = old velocity + constant acc of 0.1m/s^2
    % Noisy 2D Position Sensor 
    z_measured(:,k)= x_true(1:2,k) + noise_sigma * randn(2,1); % sensor reading = x & y positions + noise deviation * column vector of reandom numbers
end

% 5. VISUALIZATION OF THE 2D ARENA
figure('Name', 'Windsor 2D Tracker Environment', 'NumberTitle', 'off');
plot(x_true(1,:), x_true(2,:), 'g-', 'LineWidth', 2.5); hold on;
plot(z_measured(1,:), z_measured(2,:), 'r.', 'MarkerSize', 6);

xlabel('X Position (meters)');
ylabel('Y Position (meters)');
title('2D Tracking Arena: True Trajectory vs. Noisy GPS');
legend('True Path (Ground Truth)', 'Noisy GPS Position', 'Location', 'best');
grid on; axis equal;


