% Create a script file and type the following code
% Assume that CubicSpline1D class implementation has been saved in a file named 'CubicSpline1D.m'

% https://github.com/AtsushiSakai/pycubicspline/tree/acafae87f16e78dda0464241c88585434f153da7

clear;
close all;
clc;

% x = 0:4;
% y = [1.7, -6, 5, 6.5, 0.0];
% x = [-0.5, 0.0, 0.5, 1.0, 1.5];
% y = [3.2, 2.7, 6, 5, 6.5];

x = [-0.5, 0.0, 0.5, 1.0, 1.5];
y = [3.2, 2.7, 6, 5, 6.5];

% x = [1,2,3,4];
% y = [2.7,6,5,6.5];

sp = CubicSpline1D(x, y);

xi = linspace(-2.0, 4.0, 100);
yi = arrayfun(@(x) calc_position(sp, x), xi);

pp = spline(x,y);
yy = ppval(pp, xi);

figure;
plot(x, y, 'xb', 'DisplayName', 'Data points');
hold on;
plot(xi, yi, 'r', 'DisplayName', 'Cubic spline interpolation');
hold on;
%plot(xi, yy, 'g', 'DisplayName', 'Cubic spline interpolation');
hold on;
grid on;
legend;
hold off;