clear;
close all;
clc;

% Start and goal points and orientations
p1 = [0, 0];
theta1 = 0;
p2 = [10, 5];
theta2 = 0;

% Number of path points
n_path_points = 100;

% Compute the optimal curvature and curvature_rate
[curvature, curvature_rate] = compute_curvature_curvature_rate(p1, theta1, p2, theta2, n_path_points);

% Generate clothoid path
path_points = generate_clothoid_path(p1, theta1, p2, theta2, n_path_points, curvature, curvature_rate);

% Plot the clothoid path
figure;
plot(path_points(:, 1), path_points(:, 2), 'b-');
hold on;
plot(p1(1), p1(2), 'ro');
plot(p2(1), p2(2), 'ro');
xlabel('x');
ylabel('y');
title('Clothoid Path');
grid on;
axis equal;

% function [curvature, curvature_rate] = compute_curvature_curvature_rate(p1, theta1, p2, theta2)
%     % Define the objective function
%     function error = objective(x)
%         curvature = x(1);
%         curvature_rate = x(2);
%         path_points = generate_clothoid_path(p1, theta1, p2, theta2, 2, curvature, curvature_rate);
%         error = norm(path_points(end, :) - p2);
%     end
% 
%     % Initial guess
%     x0 = [0.1, 0.1];
% 
%     % Optimization options
%     options = optimoptions('fmincon', 'Display', 'off');
% 
%     % Find the optimal curvature and curvature_rate values
%     x_opt = fmincon(@objective, x0, [], [], [], [], [0, 0], [inf, inf], [], options);
% 
%     % Extract the optimal curvature and curvature_rate
%     curvature = x_opt(1);
%     curvature_rate = x_opt(2);
% end

function [curvature, curvature_rate] = compute_curvature_curvature_rate(p1, theta1, p2, theta2, num_points)
    % Define the objective function
    function error = objective(x)
        curvature = x(1);
        curvature_rate = x(2);
        path_points = generate_clothoid_path(p1, theta1, p2, theta2, 5, curvature, curvature_rate);
%         error = norm(path_points(end, :) - p2);
%         path_length = compute_path_length(p1, theta1, p2, theta2, curvature, curvature_rate, num_points); % Calculate path_length
%         error = norm(path_points(end, :) - p2) + 0.1 * path_length; % Add path_length term to the objective function
        error = norm(path_points(end, :) - p2);
    end

    % Initial guess
    x0 = [0.1, 0.1];

    % Optimization options
    options = optimset('Display', 'off');

    % Find the optimal curvature and curvature_rate values
    x_opt = fminsearch(@objective, x0, options);

    % Extract the optimal curvature and curvature_rate
    curvature = x_opt(1);
    curvature_rate = x_opt(2);
end

% function path_points = generate_clothoid_path(p1, theta1, p2, theta2, n_path_points, curvature, curvature_rate)
%     [dist, angle_diff] = compute_dist_angle_diff(p1, p2, theta1, theta2);
%     path_length = dist / curvature;
% 
%     path_points = zeros(n_path_points, 2);
%     for i = 1:n_path_points
%         s = (i - 1) / (n_path_points - 1) * path_length;
%         [x_offset, y_offset] = fresnel_integral(curvature_rate * s^2, curvature * s, theta1);
% %         x_offset
%         path_points(i, :) = p1 + [x_offset, y_offset] .* s;
%     end
% end

function [dist, angle_diff] = compute_dist_angle_diff(p1, p2, theta1, theta2)
    dx = p2(1) - p1(1);
    dy = p2(2) - p1(2);
    dist = sqrt(dx^2 + dy^2);
    phi = atan2(dy, dx);
    angle_diff = mod(theta2 - theta1 - phi + pi, 2*pi) - pi;
end

% function [x, y] = fresnel_integral(a, b, c)
%     x = integral(@(t) cos(a/2 * t.^2 + b .* t + c), 0, 1);
%     y = integral(@(t) sin(a/2 * t.^2 + b .* t + c), 0, 1);
% end

function path_points = generate_clothoid_path(p1, theta1, p2, theta2, num_points, curvature, curvature_rate)
    [dist, angle_diff] = compute_dist_angle_diff(p1, p2, theta1, theta2);
%     path_length = 1;
%     path_length = compute_path_length(p1, theta1, p2, theta2, curvature, curvature_rate, num_points);
    path_length = compute_path_length(p1, theta1, p2, theta2, curvature, curvature_rate);
    L = linspace(0, path_length, num_points);
    path_points = zeros(num_points, 2);
    
    for i = 1:num_points
        [x, y] = fresnel_integral(L(i), curvature_rate * L(i)^2, curvature * L(i), theta1);
        path_points(i, 1) = p1(1) + x;
        path_points(i, 2) = p1(2) + y;
    end
end

function [x, y] = fresnel_integral(t, a, b, c)
    % Calculate the Fresnel integral of the given function
    fun_x = @(t) cos((a/2) .* t.^2 + b .* t + c);
    fun_y = @(t) sin((a/2) .* t.^2 + b .* t + c);
    
    [x, err_x] = quad(fun_x, 0, t);
    [y, err_y] = quad(fun_y, 0, t);
end

function L = compute_path_length(p1, theta1, p2, theta2, curvature, curvature_rate)
    % Define the arc length function
    fun_length = @(t) sqrt(((-1) * sin((curvature_rate / 2) * t.^2 + curvature * t + theta1) .* (curvature_rate .* t + curvature)).^2 + ...
                            (cos((curvature_rate / 2) * t.^2 + curvature * t + theta1) .* (curvature_rate .* t + curvature)).^2);

    % Calculate the arc length
    [L, err_L] = quad(fun_length, 0, 1);
end

% function L = compute_path_length(p1, theta1, p2, theta2, curvature, curvature_rate, num_points)
%     path_points = generate_clothoid_path(p1, theta1, p2, theta2, num_points, curvature, curvature_rate);
%     L = 0;
%     for i = 2:num_points
%         L = L + norm(path_points(i, :) - path_points(i - 1, :));
%     end
% end
