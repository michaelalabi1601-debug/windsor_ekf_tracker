% Clear workspace and command window
clear; clc; close all;

% 1. Setup time vectors and true trajectory (a constant velocity robot)
dt = 0.1;               % Time step (seconds)
t = 0:dt:10;            % Total tracking time of 10 seconds
true_velocity = 5;      % 5 meters per second
true_position = true_velocity * t; % Perfect physics track (Line)

% 2. Simulate heavy, random sensor noise (White Gaussian Noise)
noise_amplitude = 3.0; 
noisy_sensor_readings = true_position + noise_amplitude * randn(size(t));

% 3. Plot the visual difference instantly
figure;
plot(t, true_position, 'g-', 'LineWidth', 2); hold on;
plot(t, noisy_sensor_readings, 'r.', 'MarkerSize', 10);
xlabel('Time (seconds)');
ylabel('Robot Position (meters)');
title('The Sensor Estimation Challenge: Noisy Radar vs. Reality');
legend('True Trajectory (Physics)', 'Noisy Sensor Readings (Reality)');
grid on;
