% CubicSpline2D_example.m
clear;
close all;
clc;

% Define x, y, and ds
x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0];
y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0];
ds = 0.1; % [m] distance of each interpolated points

% Compute the cubic spline
[rx, ry, ryaw, rk, s] = calc_spline_course(x, y, ds);

% Plot the cubic spline path
figure;
plot(x, y, 'xb', 'DisplayName', 'Data points');
hold on;
plot(rx, ry, '-r', 'DisplayName', 'Cubic spline path');
grid on;
axis equal;
xlabel('x[m]');
ylabel('y[m]');
legend;

% Plot yaw angle
figure;
plot(s, rad2deg(ryaw), '-r', 'DisplayName', 'yaw');
grid on;
legend;
xlabel('line length[m]');
ylabel('yaw angle[deg]');

% Plot curvature
figure;
plot(s, rk, '-r', 'DisplayName', 'curvature');
grid on;
legend;
xlabel('line length[m]');
ylabel('curvature [1/m]');

function [rx, ry, ryaw, rk, s] = calc_spline_course(x, y, ds)
    % Compute the cubic spline
    sp = CubicSpline2D(x, y);
    s = 0:ds:sp.s(end);
    rx = [];
    ry = [];
    ryaw = [];
    rk = [];

    for i_s = s
        [ix, iy] = sp.calc_position(i_s);
        rx = [rx, ix];
        ry = [ry, iy];
        ryaw = [ryaw, sp.calc_yaw(i_s)];
        rk = [rk, sp.calc_curvature(i_s)];
    end
end
