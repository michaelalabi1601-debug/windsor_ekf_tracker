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

% // KALMAN FILTER INITIALIZATION //
% 1. Memory Array
N =length(t);
x_est = zeros(1,N); % creates a row of zeros for N units
P_mat = zeros(1,N); % creates a record for the filter to store uncertainty values
% 2. First Guess
x_est(1) = 0; % Telling the filter to assume the staring position as 0
P = 10.0; % Uncertainty at 10.0 is very High.
P_mat(1)=P;
% 3. Filter Hyperparameters
Q = 0.01; % Process Noise Variance (0.01 means the noise does not affect the math model too much)
R = noise_amplitude^2; % Measured Noise Variance: How untrustworthy the sensor is (3^2=9 :Very untrustworthy) 

%//KALMAN FILTER RECURSIVE LOOP //
for k = 2:N
    % 1. Predict Phase
    x_pred = x_est(k-1) + true_velocity*dt; % Predicted new position = Former position + Distance(V * T)
    P_pred = P + Q; % Predicted uncertainty = Former uncertainty + Process Noise.
    % 2. Update/Correction Phase
    K = P_pred / (P_pred + R); %Kalman Gain
    x_est(k)= x_pred + K * (noisy_sensor_readings(k) -x_pred); % Final estimate = predicted position + Kalman Gain * (diff btwn sensor and math model)
    P = (1-K)*P_pred; % After looking at sensor readings, the uncertainty is reduced.
    P_mat(k) = P; % Saving the current uncertainty value in the filter's record 
end

% MULTI-PANEL PERFORMANCE VISUALIZATION
figure('Name', 'Windsor 1D Kalman Filter Tracker', 'NumberTitle', 'off');

% Subplot 1: Trajectory Performance Space
subplot(2,1,1);
plot(t, true_position, 'g-', 'LineWidth', 2.5); hold on;
plot(t, noisy_sensor_readings, 'r.', 'MarkerSize', 8);
plot(t, x_est, 'b--', 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Robot Position (meters)');
title('1D Kalman Filter State Tracking Estimation');
legend('Ground Truth (Actual)', 'Noisy Radar (Input)', 'Kalman Estimate (Filtered Output)', 'Location', 'northwest');
grid on;

% Subplot 2: Covariance Convergence (Proof of Stability)
subplot(2,1,2);
plot(t, P_mat, 'm-', 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Error Variance (P)');
title('Error Covariance Convergence Profile');
legend('Estimation Uncertainty (P)');
grid on;