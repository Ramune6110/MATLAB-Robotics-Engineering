% function [] = fit_clothoid_generation()
%     clear;
%     close all;
%     clc;
%     
%     points = [0, 0;
%               5, 3;
%               10, 8;
%               15, 15];
%           
%     global start_point;
%     global start_yaw;
%     start_point = points(1, :);
%     start_yaw = 0;
%     
%     global goal_point;
%     global goal_yaw;
%     goal_point = points(4, :);
%     goal_yaw = 0;
%     
%     
%     [fitted_clothoid, optimal_curvature, optimal_curvature_rate] = fit_clothoid(points)
%     
%     fitted_clothoid
%     
% %     clothoid_path = [];
% %     n_path_points = length(points(:, 1));
% %     clothoid_points = zeros(n_path_points, 2);
% %     L = 50;
% %     s_values = linspace(0, L, n_path_points);
% %     
% %     start_point = points(1, :);
% %     start_yaw = 0;
% %     
% %     curvature = optimal_curvature;
% %     curvature_rate = optimal_curvature_rate;
% %     
% %     for i = 1:n_path_points
% %         s = s_values(i);
% %         try
% %             x = start_point(1) + s * X(curvature_rate * s^2, curvature * s, start_yaw);
% %             y = start_point(2) + s * Y(curvature_rate * s^2, curvature * s, start_yaw);
% %             clothoid_points(i, :) = [x, y];
% %         catch exception
% %             fprintf('Skipping failed clothoid point: %s\n', exception.message);
% %         end
% %     end
% %     clothoid_path = clothoid_points
%     
%     figure(1);
%     for i = 1:length(points(:, 1))
%         plot(points(i, 2), points(i, 1), 'o'); hold on;
%     end
%     plot(clothoid_path(:, 2), clothoid_path(:, 1), 'b'); hold on;
%     
% %     fitted_clothoid
%     
% end

function [] = fit_clothoid_generation_ver2()
clear;
close all;
clc;

% Define arbitrary 4 points
% points = [
%     0, 0;
%     1, 2;
%     2, 3;
%     3, 3.5
% ];
    points = [0, 0;
              5, 3;
              10, 8;
              15, 15];

global start_point;
global start_yaw;
start_point = points(1, :);
start_yaw = 0;

global goal_point;
global goal_yaw;
goal_point = points(4, :);
goal_yaw = 0;
    
% Fit clothoid to the points
[optimal_curvature, optimal_curvature_rate, L] = fit_clothoid(points);

n_path_points = size(points, 1);
s_values = linspace(0, L, 4);
for i = 1:n_path_points
    s = s_values(i);
    try
        x = start_point(1) + s * X(optimal_curvature_rate * s^2, optimal_curvature * s, start_yaw);
        y = start_point(2) + s * Y(optimal_curvature_rate * s^2, optimal_curvature * s, start_yaw);
        clothoid_points(i, :) = [x, y];
    catch exception
        fprintf('Skipping failed clothoid point: %s\n', exception.message);
    end
end
clothoid_path = clothoid_points
    
% % Calculate points on the fitted clothoid
% t_values = linspace(0, 5, 4);
% clothoid_points = arrayfun(fitted_clothoid, t_values, 'UniformOutput', false);
% clothoid_points = cell2mat(clothoid_points');
% clothoid_x = clothoid_points(:, 1);
% clothoid_y = clothoid_points(:, 2);

% Plot original points and fitted clothoid
figure;
hold on;
plot(points(:, 2), points(:, 1), 'ro', 'MarkerSize', 8, 'DisplayName', 'Given Points');
% plot(clothoid_x, clothoid_y, 'b-', 'LineWidth', 2, 'DisplayName', 'Fitted Clothoid');
X = clothoid_path(:, 1);
Y = clothoid_path(:, 2);
%             clearpoints(lines);
%             addpoints(lines, x, y);
plot(X, Y, 'b'); hold on
xlabel('x');
ylabel('y');
title('Comparison of Given Points and Fitted Clothoid');
legend('show');
grid on;
hold off;
end

function [optimal_curvature, optimal_curvature_rate, L] = fit_clothoid(points)
% points: 4x2 matrix, each row contains [x, y] coordinates of a point

% Initial guess of parameters
init_curvature = 0;
init_curvature_rate = 0;
init_params = [init_curvature, init_curvature_rate];

% Optimization options
% options = optimoptions('fminunc','Display','iter','Algorithm','quasi-newton');
% options = optimoptions('fminunc','Algorithm','quasi-newton');
% % Optimize parameters
% [optimal_params, ~] = fminunc(@(params) clothoid_error(params, points), init_params, options);

global start_point;
global start_yaw;
global goal_point;
global goal_yaw;    

dx = goal_point(1) - start_point(1);
dy = goal_point(2) - start_point(2);
r = hypot(dx, dy);

phi = atan2(dy, dx);
phi1 = normalize_angle(start_yaw - phi);
phi2 = normalize_angle(goal_yaw - phi);
delta = phi2 - phi1;

A = solve_g_for_root(phi1, phi2, delta);
L = compute_path_length(r, phi1, delta, A);

% Optimization options
options = optimoptions('fminunc','Display','iter','Algorithm','quasi-newton');

% Optimize parameters
[optimal_params, ~] = fminunc(@(params) clothoid_error(params, points, L), init_params, options);

% Extract optimal curvature and curvature rate
optimal_curvature = optimal_params(1);
optimal_curvature_rate = optimal_params(2);

% Fitted clothoid
% fitted_clothoid = @(t) clothoid_function(t, optimal_curvature, optimal_curvature_rate);
end

function error = clothoid_error(params, points, L)
% params: [curvature, curvature_rate]
% points: 4x2 matrix, each row contains [x, y] coordinates of a point

% Define clothoid
% clothoid = @(t) clothoid_function(t, params(1), params(2));

% Find t values for each point
t_values = linspace(0, L, size(points, 1));

% t_values = linspace(0, 101, size(points, 1));

global start_point;
global start_yaw;
global goal_point;
global goal_yaw;

% Calculate error
error = 0;
for i = 1:size(points, 1)
    point = points(i, :);
    t = t_values(i);
    x = start_point(1) + t * X(params(2) * t^2, params(1) * t, start_yaw);
    y = start_point(2) + t * Y(params(2) * t^2, params(1) * t, start_yaw);
    clothoid_point = [x, y];
    error = error + norm(point - clothoid_point)^2;
end
end

function point = clothoid_function(t, curvature, curvature_rate)
% Calculate Fresnel integrals
global start_point;
global start_yaw;
    
x = start_point(1) + t * X(curvature_rate * t^2, curvature * t, start_yaw);
y = start_point(2) + t * Y(curvature_rate * t^2, curvature * t, start_yaw);

point = [x, y];
end

function result = X(a, b, c)
    result = integral(@(t) cos((a/2) .* t.^2 + b .* t + c), 0, 1);
end

function result = Y(a, b, c)
    result = integral(@(t) sin((a/2) .* t.^2 + b .* t + c), 0, 1);
end

function result = solve_g_for_root(theta1, theta2, delta)
    initial_guess = 3 * (theta1 + theta2);
    result = fsolve(@(A) Y(2*A, delta - A, theta1), initial_guess);
end

function result = compute_path_length(r, theta1, delta, A)
    result = r / X(2*A, delta - A, theta1);
end

function result = normalize_angle(angle_rad)
    result = mod(angle_rad + pi, 2 * pi) - pi;
end
