clear;
close all;
clc;

%% main
start_point = [0, 0];
start_orientation_list = 0.0;
goal_point = [10, 0];
goal_orientation_list = linspace(-pi, pi, 75);
num_path_points = 100;

clothoid_paths = generate_clothoid_paths(...
    start_point, start_orientation_list, ...
    goal_point, goal_orientation_list, ...
    num_path_points);

show_animation = true;

if show_animation
    draw_clothoids(start_point, goal_point, ...
        num_path_points, clothoid_paths, ...
        false);
end

%% function
function clothoids = generate_clothoid_paths(start_point, start_yaw_list, goal_point, goal_yaw_list, n_path_points)
    clothoids = {};
    for start_yaw = start_yaw_list
        for goal_yaw = goal_yaw_list
            clothoid = generate_clothoid_path(start_point, start_yaw, goal_point, goal_yaw, n_path_points);
            clothoids{end+1} = clothoid;
        end
    end
end

function clothoid_path = generate_clothoid_path(start_point, start_yaw, goal_point, goal_yaw, n_path_points)
    dx = goal_point(1) - start_point(1);
    dy = goal_point(2) - start_point(2);
    r = hypot(dx, dy);

    phi = atan2(dy, dx);
    phi1 = normalize_angle(start_yaw - phi);
    phi2 = normalize_angle(goal_yaw - phi);
    delta = phi2 - phi1;

    try
        A = solve_g_for_root(phi1, phi2, delta);

        L = compute_path_length(r, phi1, delta, A);
        curvature = compute_curvature(delta, A, L);
        curvature_rate = compute_curvature_rate(A, L);
    catch exception
        fprintf('Failed to generate clothoid points: %s\n', exception.message);
        clothoid_path = [];
        return;
    end

    points = zeros(n_path_points, 2);
    s_values = linspace(0, L, n_path_points);
    for i = 1:n_path_points
        s = s_values(i);
        try
            x = start_point(1) + s * X(curvature_rate * s^2, curvature * s, start_yaw);
            y = start_point(2) + s * Y(curvature_rate * s^2, curvature * s, start_yaw);
            points(i, :) = [x, y];
        catch exception
            fprintf('Skipping failed clothoid point: %s\n', exception.message);
        end
    end
    clothoid_path = points;
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

function result = compute_curvature(delta, A, L)
    result = (delta - A) / L;
end

function result = compute_curvature_rate(A, L)
    result = 2 * A / (L^2);
end

function result = normalize_angle(angle_rad)
    result = mod(angle_rad + pi, 2 * pi) - pi;
end

%% draw function
function [x_min, x_max, y_min, y_max] = get_axes_limits(clothoids)
    x_vals = [];
    y_vals = [];

    for i = 1:numel(clothoids)
        clothoid = clothoids{i};
        x_vals = [x_vals, clothoid(:, 1)'];
        y_vals = [y_vals, clothoid(:, 2)'];
    end

    x_min = min(x_vals);
    x_max = max(x_vals);
    y_min = min(y_vals);
    y_max = max(y_vals);

    x_offset = 0.1 * (x_max - x_min);
    y_offset = 0.1 * (y_max - y_min);

    x_min = x_min - x_offset;
    x_max = x_max + x_offset;
    y_min = y_min - y_offset;
    y_max = y_max + y_offset;
end

function draw_clothoids(start, goal, num_steps, clothoidal_paths, save_animation)
    figure('Position', [100, 100, 800, 800]);
    [x_min, x_max, y_min, y_max] = get_axes_limits(clothoidal_paths);
    axes = gca;
    axes.XLim = [x_min, x_max];
    axes.YLim = [y_min, y_max];

    plot(start(1), start(2), 'ro');
    plot(goal(1), goal(2), 'ro');

    lines = gobjects(1, numel(clothoidal_paths));
    for i = 1:numel(clothoidal_paths)
        lines(i) = animatedline('Color', 'b', 'LineWidth', 1);
    end

    for i = 1:num_steps
        for j = 1:numel(clothoidal_paths)
            clothoid_path = clothoidal_paths{j};
            x = clothoid_path(1:i, 1);
            y = clothoid_path(1:i, 2);
            clearpoints(lines(j));
            addpoints(lines(j), x, y);
        end
        pause(0.025);
    end

    if save_animation
        % Save the animation as a GIF file
        filename = 'clothoid.gif';
        delay_time = 0.025;
        save_animated_gif(filename, delay_time);
    end
end
