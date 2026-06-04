% Day 23: IMU + Magnetic Matching + Kalman Filter Fusion
clear; close all; clc;

% Load magnetic map (from Day 20)
load('magnetic_map.mat');   % 10x10 grid
map_size = size(map);
total_cells = map_size(1) * map_size(2);

% Simulation parameters
dt = 0.1;           % time step (seconds)
total_time = 50;    % simulate 50 seconds
time = 0:dt:total_time;
N = length(time);

% True vehicle movement (simple: constant velocity in x and y)
true_x = zeros(1,N);
true_y = zeros(1,N);
true_x(1) = 2;   % start at column 2
true_y(1) = 3;   % start at row 3
vx = 0.2;        % velocity (cells per second)
vy = 0.1;

for k = 2:N
    true_x(k) = true_x(k-1) + vx*dt;
    true_y(k) = true_y(k-1) + vy*dt;
    % Keep within map boundaries (1 to 10)
    true_x(k) = max(1, min(10, true_x(k)));
    true_y(k) = max(1, min(10, true_y(k)));
end

% Get true magnetic value at each true position
true_mag = zeros(1,N);
for k = 1:N
    row = round(true_y(k));
    col = round(true_x(k));
    true_mag(k) = map(row, col);
end

% --- Simulate IMU (drift) ---
% IMU measures velocity with noise, then integrate to position
imu_vel_x = vx + 0.5*randn(1,N);   % noisy velocity measurement
imu_vel_y = vy + 0.5*randn(1,N);
imu_pos_x = zeros(1,N);
imu_pos_y = zeros(1,N);
imu_pos_x(1) = true_x(1);
imu_pos_y(1) = true_y(1);
for k = 2:N
    imu_pos_x(k) = imu_pos_x(k-1) + imu_vel_x(k)*dt;
    imu_pos_y(k) = imu_pos_y(k-1) + imu_vel_y(k)*dt;
end

% --- Simulate Magnetic Matching (noisy, sometimes wrong) ---
mag_est_x = zeros(1,N);
mag_est_y = zeros(1,N);
% At each step, get noisy measurement of magnetic field
for k = 1:N
    % Noisy measurement of true magnetic value
    noisy_measurement = true_mag(k) + 5*randn;   % noise std = 5
    
    % Find closest cell in map
    [minDiff, idx] = min(abs(map(:) - noisy_measurement));
    [row_est, col_est] = ind2sub(map_size, idx);
    mag_est_x(k) = col_est;
    mag_est_y(k) = row_est;
end

% --- Kalman Filter Fusion ---
% Simple 1D Kalman for x and y separately (works for this simulation)
% Initialize
kf_x = zeros(1,N);   % estimated position x
kf_y = zeros(1,N);
P_x = 1;   % initial uncertainty
P_y = 1;
kf_x(1) = imu_pos_x(1);
kf_y(1) = imu_pos_y(1);

% Process noise (how much we trust IMU)
Q = 0.001;
% Measurement noise (how much we trust magnetic matching)
R_mag = 100.0;   % smaller = trust magnetic more

for k = 2:N
    % --- Prediction (using IMU delta) ---
    % Predict position based on IMU velocity
    pred_x = kf_x(k-1) + imu_vel_x(k)*dt;
    pred_y = kf_y(k-1) + imu_vel_y(k)*dt;
    P_pred_x = P_x + Q;
    P_pred_y = P_y + Q;
    
    % --- Update with magnetic measurement ---
    % Kalman gain
    K_x = P_pred_x / (P_pred_x + R_mag);
    K_y = P_pred_y / (P_pred_y + R_mag);
    % Correct estimate
    kf_x(k) = pred_x + K_x * (mag_est_x(k) - pred_x);
    kf_y(k) = pred_y + K_y * (mag_est_y(k) - pred_y);
    P_x = (1 - K_x) * P_pred_x;
    P_y = (1 - K_y) * P_pred_y;
end

% --- Compute Errors ---
imu_error = sqrt((imu_pos_x - true_x).^2 + (imu_pos_y - true_y).^2);
mag_error = sqrt((mag_est_x - true_x).^2 + (mag_est_y - true_y).^2);
kf_error = sqrt((kf_x - true_x).^2 + (kf_y - true_y).^2);

% --- Plot Results ---
figure;
subplot(2,2,1);
plot(time, true_x, 'g-', time, imu_pos_x, 'r--', time, mag_est_x, 'b:', time, kf_x, 'k-', 'LineWidth',1.5);
legend('True','IMU (drift)','Magnetic (jumpy)','KF (fused)');
title('X Position over Time');
xlabel('Time (s)'); ylabel('X cell');

subplot(2,2,2);
plot(time, true_y, 'g-', time, imu_pos_y, 'r--', time, mag_est_y, 'b:', time, kf_y, 'k-', 'LineWidth',1.5);
legend('True','IMU','Magnetic','KF');
title('Y Position over Time');
xlabel('Time (s)'); ylabel('Y cell');

subplot(2,2,3);
plot(time, imu_error, 'r-', time, mag_error, 'b-', time, kf_error, 'k-', 'LineWidth',1.5);
legend('IMU error','Magnetic error','KF error');
title('Position Error (cells)');
xlabel('Time (s)'); ylabel('Error (cells)');

subplot(2,2,4);
plot(imu_error, kf_error, 'ko');
xlabel('IMU error (cells)'); ylabel('KF error (cells)');
title('Kalman reduces error drastically');
grid on;

fprintf('Final errors (cells):\n');
fprintf('IMU error: %.2f\n', imu_error(end));
fprintf('Magnetic error: %.2f\n', mag_error(end));
fprintf('Kalman fused error: %.2f\n', kf_error(end));