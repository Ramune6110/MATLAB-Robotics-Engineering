clear;
close all;
clc;

% Parameters
L = 2.5; % wheelbase [m]
v = 10; % constant vehicle speed [m/s]
dt = 0.01; % time step [s]
T = 10; % total time [s]
t = 0:dt:T; % time vector
omega = 2 * pi / T; % angular frequency of the sine wave

% Generate steering angle signal (sin wave)
delta = sin(omega * t);

% Initialize vehicle states
x = zeros(1, length(t));
y = zeros(1, length(t));
theta = zeros(1, length(t));

% Simulate the bicycle model
for i = 2:length(t)
    theta(i) = theta(i-1) + (v / L) * tan(delta(i-1)) * dt;
    x(i) = x(i-1) + v * cos(theta(i-1)) * dt;
    y(i) = y(i-1) + v * sin(theta(i-1)) * dt;
end

% Calculate traveled distance
traveled_distance = sum(sqrt(diff(x).^2 + diff(y).^2));
traveled_distance_log = cumsum(sqrt(diff(x).^2 + diff(y).^2));

% Plot x, y, theta vs time
figure(1);
subplot(411);
plot(t, delta);
xlabel('Time [s]');
ylabel('delta [rad]');

subplot(412);
plot(t, x);
xlabel('Time [s]');
ylabel('x [m]');

subplot(413);
plot(t, y);
xlabel('Time [s]');
ylabel('y [m]');

subplot(414);
plot(t, theta);
xlabel('Time [s]');
ylabel('\theta [rad]');

% Plot x, y, theta vs traveled distance
figure(2);
subplot(411);
plot(traveled_distance_log, delta(2:end));
xlabel('Traveled Distance [m]');
ylabel('delta [rad]');

subplot(412);
plot(traveled_distance_log, x(2:end));
xlabel('Traveled Distance [m]');
ylabel('x [m]');

subplot(413);
plot(traveled_distance_log, y(2:end));
xlabel('Traveled Distance [m]');
ylabel('y [m]');

subplot(414);
plot(traveled_distance_log, theta(2:end));
xlabel('Traveled Distance [m]');
ylabel('\theta [rad]');

% trajectory
figure(3);
plot(x, y, 'b');
grid on;
