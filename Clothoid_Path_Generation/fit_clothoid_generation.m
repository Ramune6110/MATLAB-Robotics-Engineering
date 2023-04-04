function [] = fit_clothoid_generation()
    clear;
    close all;
    clc;
    
    points = [0, 0;
              5, 3;
              10, 8;
              15, 15];
          
    [fitted_clothoid, optimal_curvature, optimal_curvature_rate] = fit_clothoid(points)
    
    fitted_clothoid
    
    clothoid_path = [];
    n_path_points = length(points(:, 1));
    clothoid_points = zeros(n_path_points, 2);
    L = 50;
    s_values = linspace(0, L, n_path_points);
    
    start_point = points(1, :);
    start_yaw = 0;
    
    curvature = optimal_curvature;
    curvature_rate = optimal_curvature_rate;
    
    for i = 1:n_path_points
        s = s_values(i);
        try
            x = start_point(1) + s * X(curvature_rate * s^2, curvature * s, start_yaw);
            y = start_point(2) + s * Y(curvature_rate * s^2, curvature * s, start_yaw);
            clothoid_points(i, :) = [x, y];
        catch exception
            fprintf('Skipping failed clothoid point: %s\n', exception.message);
        end
    end
    clothoid_path = clothoid_points
    
    figure(1);
    for i = 1:length(points(:, 1))
        plot(points(i, 2), points(i, 1), 'o'); hold on;
        plot(clothoid_path(i, 2), clothoid_path(i, 1), 'o'); hold on;
    end
    
%     fitted_clothoid
    
end

% function [] = fit_clothoid_generation()
% % Define arbitrary 4 points
% points = [
%     0, 0;
%     1, 2;
%     2, 3;
%     3, 3.5
% ];
% 
% % Fit clothoid to the points
% [fitted_clothoid, optimal_curvature, optimal_curvature_rate] = fit_clothoid(points);
% 
% % Calculate points on the fitted clothoid
% t_values = linspace(0, 1, 100);
% clothoid_points = arrayfun(fitted_clothoid, t_values, 'UniformOutput', false);
% clothoid_points = cell2mat(clothoid_points');
% clothoid_x = clothoid_points(:, 1);
% clothoid_y = clothoid_points(:, 2);
% 
% % Plot original points and fitted clothoid
% figure;
% hold on;
% plot(points(:, 1), points(:, 2), 'ro', 'MarkerSize', 8, 'DisplayName', 'Given Points');
% plot(clothoid_x, clothoid_y, 'b-', 'LineWidth', 2, 'DisplayName', 'Fitted Clothoid');
% xlabel('x');
% ylabel('y');
% title('Comparison of Given Points and Fitted Clothoid');
% legend('show');
% grid on;
% hold off;
% end

function result = X(a, b, c)
    result = integral(@(t) cos((a/2) .* t.^2 + b .* t + c), 0, 1);
end

function result = Y(a, b, c)
    result = integral(@(t) sin((a/2) .* t.^2 + b .* t + c), 0, 1);
end

function [fitted_clothoid, optimal_curvature, optimal_curvature_rate] = fit_clothoid(points)
% points: 4x2 matrix, each row contains [x, y] coordinates of a point

% Initial guess of parameters
init_curvature = 0;
init_curvature_rate = 0;
init_params = [init_curvature, init_curvature_rate];

% Optimization options
options = optimoptions('fminunc','Display','iter','Algorithm','quasi-newton');

% Optimize parameters
[optimal_params, ~] = fminunc(@(params) clothoid_error(params, points), init_params, options);

% Extract optimal curvature and curvature rate
optimal_curvature = optimal_params(1);
optimal_curvature_rate = optimal_params(2);

% Fitted clothoid
fitted_clothoid = @(t) clothoid_function(t, optimal_curvature, optimal_curvature_rate);
end

function error = clothoid_error(params, points)
% params: [curvature, curvature_rate]
% points: 4x2 matrix, each row contains [x, y] coordinates of a point

% Define clothoid
clothoid = @(t) clothoid_function(t, params(1), params(2));

% Find t values for each point
t_values = linspace(0, 1, size(points, 1))

% Calculate error
error = 0;
for i = 1:size(points, 1)
    point = points(i, :);
    clothoid_point = clothoid(t_values(i));
    error = error + norm(point - clothoid_point)^2;
end
end

function point = clothoid_function(t, curvature, curvature_rate)
% Calculate Fresnel integrals
% [x_fresnel, y_fresnel] = fresnel(t * sqrt(curvature_rate));
[x, y] = fresnel_integral(t, curvature, curvature_rate);

% % Calculate point on clothoid
% x = curvature_rate * x_fresnel;
% y = curvature_rate * y_fresnel;

point = [x, y];
end

function [x, y] = fresnel_integral(t, a, b)
    % Calculate the Fresnel integral of the given function
    fun_x = @(t) cos((a/2) .* t.^2 + b .* t);
    fun_y = @(t) sin((a/2) .* t.^2 + b .* t);
    
    [x, err_x] = quad(fun_x, 0, t);
    [y, err_y] = quad(fun_y, 0, t);
end