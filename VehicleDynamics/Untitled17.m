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

figure;  % 新しいウィンドウを開く
axis equal;
grid on;
hold on;
handle_center = [0; 0];
handle_radius = 1;
h_handle_circle = draw_handle_circle(handle_center, handle_radius, deg2rad(steering_input(1)));
title('Steering Handle');
xlabel('x');
ylabel('y');
xlim([-2, 2]);
ylim([-2, 2]);
hold off

for step = 1:n_steps
    % Update vehicle state
    V = 10; % m/s (constant speed)
    steering_angle = steering_input(step);
    
    % ハンドルのアニメーション更新
    update_handle_plot(h_handle_circle, handle_center, handle_radius, deg2rad(steering_angle));
    
    % Refresh plot
    pause(time_step);
%     if step ~= n_steps
%         clf;
%         axis equal;
%         xlim([0, 80])
%         ylim([0, 60])
%         hold on;
%         xlabel('X [m]');
%         ylabel('Y [m]');
%     end
end

function h_handle_circle = draw_handle_circle(handle_center, radius, steering_angle)
    num_points = 100;
    circle_points = zeros(2, num_points);
    for i = 1:num_points
        angle = 2*pi*(i-1)/num_points;
        circle_points(:, i) = handle_center + radius * [cos(angle); sin(angle)];
    end
    handle_marker = handle_center + radius * [cos(steering_angle); sin(steering_angle)];
    h_handle_circle = plot(circle_points(2, :), circle_points(1, :), 'b', handle_marker(2), handle_marker(1), 'ro', 'MarkerSize', 10);
end

function update_handle_plot(h_handle_circle, handle_center, radius, steering_angle)
    num_points = 100;
    circle_points = zeros(2, num_points);
    for i = 1:num_points
        angle = 2*pi*(i-1)/num_points;
        circle_points(:, i) = handle_center + radius * [cos(angle); sin(angle)];
    end
    handle_marker = handle_center + radius * [cos(steering_angle); sin(steering_angle)];
    set(h_handle_circle(1), 'XData', circle_points(2, :), 'YData', circle_points(1, :));
    set(h_handle_circle(2), 'XData', handle_marker(2), 'YData', handle_marker(1));
% %     hold on;
%     plot([handle_marker(2), 0], [handle_marker(1), 0], 'k--');
% %     hold off;
    
    drawnow;
end
