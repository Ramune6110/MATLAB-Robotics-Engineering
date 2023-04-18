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

% Initialize plot
fig = figure;
fig.Position = [0 0 1368 700];
axis equal;
hold on;
xlabel('X [m]');
ylabel('Y [m]');

for step = 1:n_steps
    % Update vehicle state
    V = 10; % m/s (constant speed)
    steering_angle = steering_input(step);
    delta_x = V * cos(theta) * time_step;
    delta_y = V * sin(theta) * time_step;
    x = x + delta_x;
    y = y + delta_y;
    theta = theta + (V / wheelbase) * tan(deg2rad(steering_angle)) * time_step;

    % Draw vehicle
    vehicle_corners = [
        -vehicle_length / 2, vehicle_length / 2, vehicle_length / 2, -vehicle_length / 2, -vehicle_length / 2;
        -vehicle_width / 2, -vehicle_width / 2, vehicle_width / 2, vehicle_width / 2, -vehicle_width / 2
    ];
    R_vehicle = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    vehicle_corners = R_vehicle * vehicle_corners + repmat([x; y], 1, 5);
    plot(vehicle_corners(1, :), vehicle_corners(2, :), 'r-');
    
    % Draw wheels
    wheel_positions = [
        x + wheelbase / 2 * cos(theta), x - wheelbase / 2 * cos(theta), x - wheelbase / 2 * cos(theta), x + wheelbase / 2 * cos(theta);
        y + wheelbase / 2 * sin(theta), y - wheelbase / 2 * sin(theta), y - wheelbase / 2 * sin(theta) - vehicle_width / 2 * cos(theta - pi / 2), y + wheelbase / 2 * sin(theta) - vehicle_width / 2 * cos(theta - pi / 2)
    ];

    for i = 1:2
        wheel_corners = [
            -wheel_length / 2, wheel_length / 2, wheel_length / 2, -wheel_length / 2, -wheel_length / 2;
            -wheel_width / 2, -wheel_width / 2, wheel_width / 2, wheel_width / 2, -wheel_width / 2
        ];
        
        if i <= 2
            wheel_theta = theta + deg2rad(steering_angle);
        else
            wheel_theta = theta;
        end
        R_wheel = [cos(wheel_theta), -sin(wheel_theta); sin(wheel_theta), cos(wheel_theta)];
        
        wheel_corners = R_wheel * wheel_corners + repmat(wheel_positions(:, i), 1, 5);
        plot(wheel_corners(1, :), wheel_corners(2, :), 'k-');
    end
    
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
