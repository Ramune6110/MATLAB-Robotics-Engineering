% Parameters
L = 2.5; % wheelbase [m]
Lf = 1.25; % distance from front axle to CG [m]
Lr = L - Lf; % distance from rear axle to CG [m]
v = 10; % constant vehicle speed [m/s]
dt = 0.01; % time step [s]
T = 10; % total time [s]
t = 0:dt:T; % time vector
omega = 2 * pi / T; % angular frequency of the sine wave

% Generate steering angle signal (sin wave)
delta_f = sin(omega * t);

% Initialize vehicle states
x = zeros(1, length(t));
y = zeros(1, length(t));
theta = zeros(1, length(t));

% Simulate the four-wheel vehicle model
for i = 2:length(t)
    beta = atan((Lr / L) * tan(delta_f(i-1)));
    theta(i) = theta(i-1) + (v / L) * sin(beta) * dt;
    x(i) = x(i-1) + v * cos(theta(i-1)) * dt;
    y(i) = y(i-1) + v * sin(theta(i-1)) * dt;
end

% Plot the four-wheel vehicle motion animation
figure;
% for i = 1:length(t)
%     % Vehicle dimensions
%     vehicle_width = 1.8;
%     vehicle_length = 4.5;
%     
%     % Vehicle body
%     body = [x(i) + Lf * cos(theta(i)) - vehicle_length / 2 * cos(theta(i)), y(i) + Lf * sin(theta(i)) - vehicle_length / 2 * sin(theta(i));
%             x(i) + Lf * cos(theta(i)) + vehicle_length / 2 * cos(theta(i)), y(i) + Lf * sin(theta(i)) + vehicle_length / 2 * sin(theta(i))];
%     % Left and right wheels
%     left_wheel = [body(1,1) - vehicle_width / 2 * sin(theta(i)), body(1,2) + vehicle_width / 2 * cos(theta(i))];
%     right_wheel = [body(1,1) + vehicle_width / 2 * sin(theta(i)), body(1,2) - vehicle_width / 2 * cos(theta(i))];
%     
%     % Plot vehicle body
%     plot(body(:,1), body(:,2), 'k-', 'LineWidth', 2);
%     hold on;
%     
%     % Plot left and right wheels
%     plot([left_wheel(1), right_wheel(1)], [left_wheel(2), right_wheel(2)], 'k-', 'LineWidth', 2);
%     
%     % Plot vehicle trajectory
%     plot(x(1:i), y(1:i), 'r--');
%     
%     % Axis settings
%     axis equal;
%     xlim([min(x) - vehicle_length, max(x) + vehicle_length]);
%     ylim([min(y) - vehicle_length, max(y) + vehicle_length]);
%     xlabel('X [m]');
%     ylabel('Y [m]');
%     title(sprintf('Four-Wheel Vehicle Motion Animation (t=%.2fs)', t(i)));
%     
%     % Pause for animation
%     pause(0.01);
%     
%     % Clear current frame
%     if i < length(t)
%         clf;
%     end
% end

% Plot the four-wheel vehicle motion animation
figure;
for i = 1:length(t)
    % Vehicle dimensions
    vehicle_width = 1.8;
    vehicle_length = 4.5;
    
    % Corner positions
    p1 = [x(i) + Lf * cos(theta(i)) - vehicle_length / 2 * cos(theta(i)) - vehicle_width / 2 * sin(theta(i)), y(i) + Lf * sin(theta(i)) - vehicle_length / 2 * sin(theta(i)) + vehicle_width / 2 * cos(theta(i))];
    p2 = [x(i) + Lf * cos(theta(i)) - vehicle_length / 2 * cos(theta(i)) + vehicle_width / 2 * sin(theta(i)), y(i) + Lf * sin(theta(i)) - vehicle_length / 2 * sin(theta(i)) - vehicle_width / 2 * cos(theta(i))];
    p3 = [x(i) + Lf * cos(theta(i)) + vehicle_length / 2 * cos(theta(i)) + vehicle_width / 2 * sin(theta(i)), y(i) + Lf * sin(theta(i)) + vehicle_length / 2 * sin(theta(i)) - vehicle_width / 2 * cos(theta(i))];
    p4 = [x(i) + Lf * cos(theta(i)) + vehicle_length / 2 * cos(theta(i)) - vehicle_width / 2 * sin(theta(i)), y(i) + Lf * sin(theta(i)) + vehicle_length / 2 * sin(theta(i)) + vehicle_width / 2 * cos(theta(i))];
    
    % Plot vehicle body
    plot([p1(1), p2(1), p3(1), p4(1), p1(1)], [p1(2), p2(2), p3(2), p4(2), p1(2)], 'k-', 'LineWidth', 2);
    hold on;
    
    % Plot front wheels
    wheel_length = vehicle_width * 0.4;
    left_front_wheel = [p1(1) + Lf * wheel_length * cos(delta_f(i)), p1(2) + Lf * wheel_length * sin(delta_f(i))];
    right_front_wheel = [p2(1) + Lf * wheel_length * cos(delta_f(i)), p2(2) + Lf * wheel_length * sin(delta_f(i))];
    plot([p1(1), left_front_wheel(1)], [p1(2), left_front_wheel(2)], 'k-', 'LineWidth', 2);
    plot([p2(1), right_front_wheel(1)], [p2(2), right_front_wheel(2)], 'k-', 'LineWidth', 2);
    
    % Plot rear wheels
    plot([p4(1), p3(1)], [p4(2), p3(2)], 'k-', 'LineWidth', 2);
    
    % Plot vehicle trajectory
    plot(x(1:i), y(1:i), 'r--');
    
    % Axis settings
    axis equal;
    xlim([min(x) - vehicle_length, max(x) + vehicle_length]);
    ylim([min(y) - vehicle_length, max(y) + vehicle_length]);
    xlabel('X [m]');
    ylabel('Y [m]');
    title(sprintf('Four-Wheel Vehicle Motion Animation (t=%.2fs)', t(i)));
        % Pause for animation
    pause(0.01);
    
    % Clear current frame
    if i < length(t)
        clf;
    end
end


