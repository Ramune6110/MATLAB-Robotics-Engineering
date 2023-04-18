clear;
close all;
clc;

% Vehicle parameters
vehicle_length = 4; % meters
vehicle_width = 2; % meters
wheel_length = 0.5; % meters
wheel_width = 0.2; % meters
wheelbase = 2.5; % meters

% Simulation parameters
time_step = 0.1; % seconds
total_time = 10; % seconds
n_steps = total_time / time_step;

% Initialize vehicle state
x = 0;
y = 0;
theta = 0;

% Initialize steering input (replace this with your desired steering input)
steering_input = 30 * sin(2 * pi * 0.5 * (0:time_step:total_time - time_step)); % degrees

% Initialize vehicle states
x_trj = zeros(1, n_steps);
y_trj = zeros(1, n_steps);
theta_trj = zeros(1, n_steps);

% Simulate the bicycle model
for i = 2:n_steps
    % Update vehicle state
    V = 10; % m/s (constant speed)
    steering_angle = steering_input(i - 1);
    
    x_trj(i) = x_trj(i - 1) + V * cos(theta_trj(i - 1)) * time_step;
    y_trj(i) = y_trj(i - 1) + V * sin(theta_trj(i - 1)) * time_step;
    theta_trj(i) = theta_trj(i - 1) + (V / wheelbase) * tan(deg2rad(steering_angle)) * time_step;
end

% Initialize plot
fig = figure;
fig.Position = [0 0 1368 700];

for step = 1:n_steps
    % Update vehicle state
    V = 10; % m/s (constant speed)
    steering_angle = steering_input(step);
    
    x = x + V * cos(theta) * time_step;
    y = y + V * sin(theta) * time_step;
    theta = theta + (V / wheelbase) * tan(deg2rad(steering_angle)) * time_step;

    % Draw vehicle
    vehicle_corners = [
        -vehicle_length / 2, vehicle_length / 2, vehicle_length / 2, -vehicle_length / 2, -vehicle_length / 2;
        -vehicle_width / 2, -vehicle_width / 2, vehicle_width / 2, vehicle_width / 2, -vehicle_width / 2
    ];
    R_vehicle = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    vehicle_corners = R_vehicle * vehicle_corners + repmat([x; y], 1, 5);
    car_center = [x; y];
    plot(vehicle_corners(1, :), vehicle_corners(2, :), 'k-');
    plot(car_center(1), car_center(2), 'bo');
    % Plot vehicle trajectory
    plot(x_trj(1:step), y_trj(1:step), 'r--');
    
    % Draw wheels
    for i = 1:4
        wheel_corners = [
            -wheel_length / 2, wheel_length / 2, wheel_length / 2, -wheel_length / 2, -wheel_length / 2;
            -wheel_width / 2, -wheel_width / 2, wheel_width / 2, wheel_width / 2, -wheel_width / 2
        ];
        
        wheel_positions = calc_wheel_positions(car_center, theta, deg2rad(steering_angle), vehicle_length, vehicle_width);
        
        if i <= 2
            wheel_theta = theta + deg2rad(steering_angle);
        else
            wheel_theta = theta;
        end
        R_wheel = [cos(wheel_theta), -sin(wheel_theta); sin(wheel_theta), cos(wheel_theta)];
        
        wheel_corners = R_wheel * wheel_corners + repmat(wheel_positions(:, i), 1, 5);
        plot(wheel_corners(1, :), wheel_corners(2, :), 'k-'); hold on;
    end
    
    % 左前輪と右前輪の中心位置を結ぶ連結線を描画
    front_left_wheel_center = wheel_positions(:, 1);
    front_right_wheel_center = wheel_positions(:, 2);
    rear_left_wheel_center = wheel_positions(:, 3);
    rear_right_wheel_center = wheel_positions(:, 4);
    hold on;
    plot([front_left_wheel_center(1), front_right_wheel_center(1)], [front_left_wheel_center(2), front_right_wheel_center(2)], 'k-');
    plot([rear_left_wheel_center(1), rear_right_wheel_center(1)], [rear_left_wheel_center(2), rear_right_wheel_center(2)], 'k-');
    hold off;
    
    % 左前輪と右前輪の中心位置を結ぶ連結線を描画
    front_wheel_center = (front_left_wheel_center + front_right_wheel_center) ./  2;
    rear_wheel_center = (rear_left_wheel_center + rear_right_wheel_center) ./ 2;
    hold on;
    plot([front_wheel_center(1), rear_wheel_center(1)], [front_wheel_center(2), rear_wheel_center(2)], 'k-');
    hold off;
    
    % Refresh plot
    pause(time_step);
    if step ~= n_steps
        clf;
        axis equal;
        xlim([0, 80])
        ylim([0, 60])
        hold on;
        xlabel('X [m]');
        ylabel('Y [m]');
    end
end

function wheel_positions = calc_wheel_positions(car_center, theta, delta, L, W)
    % 操舵角から前輪の車輪角度を計算
    front_wheel_angle = [cos(theta + delta), sin(theta + delta)];
    rear_wheel_angle = [cos(theta), sin(theta)];

    % 前輪と後輪の位置を計算
    front_left_wheel = car_center + L/3 * [cos(theta); sin(theta)] + W/3 * [-sin(theta); cos(theta)];
    front_right_wheel = car_center + L/3 * [cos(theta); sin(theta)] - W/3 * [-sin(theta); cos(theta)];
    rear_left_wheel = car_center - L/3 * [cos(theta); sin(theta)] + W/3 * [-sin(theta); cos(theta)];
    rear_right_wheel = car_center - L/3 * [cos(theta); sin(theta)] - W/3 * [-sin(theta); cos(theta)];

    % 車輪の位置を格納
    wheel_positions = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel];
end
