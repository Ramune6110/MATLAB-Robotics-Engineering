clear;
close all;
clc;

%% Ipopt solver path
dir = pwd;
addpath(fullfile(dir,'Ipopt'))

%% Setup and Parameters
dt = 0.1; % sample time
nx = 3;   % Number of states
nu = 1;   % Number of input

P = 200 * eye(nx);                        % Termination cost weight matrix
Q = diag([1.0, 1.0, 1.0, 1.0]); % Weight matrix of state quantities
R = 0.1;                     % Weight matrix of input quantities
N = 10;                                   % Predictive Horizon

% Range of states and inputs
xmin = [-5; -5; -5; -5; -5; -5];
xmax = [6; 6; 5; 5; 5; 5];
umin = [-1; -5];
umax = [1; 5];

% target state value
x_target = [6.0; 1.0; 0.0; 0.0; 0.0; 0.0]; 

% Obstacle
obs.pos = [3.0; 0.0];
obs.own_vehicle_diameter = 0.5; % [m]
obs.obstacle_diameter    = 0.5; % [m]
obs.r = obs.own_vehicle_diameter + obs.obstacle_diameter;

%% Linear Car models (Lateral Vehicle Dynamics)
M = 1500;     % [kg]
I = 2500;     % [kgm^2]
lf = 1.1;     % [m]
lr = 1.6;     % [m]
l = lf + lr;  % [m]
Kf = 55000;   % [N/rad]
Kr = 60000;   % [N/rad]
Vx = 20;      % [m/s]

system.dt = dt;
system.nx = nx;
system.nu = nu;

A21 = -2 * (Kf + Kr) / (M * Vx);
A22 = -Vx - 2 * (Kf * lf - Kr * lr) / (M * Vx);
A42 = -2 * (lf * Kf - lr * Kr) / (I * Vx);
A44 = -2 * (lf^2 * Kf + lr^2 * Kr) / (I * Vx);

system.A = eye(nx) + [0.0, 1.0, 0.0, 0.0;
                      0.0, A21, 0.0, A22;
                      0.0, 0.0, 0.0, 1.0;
                      0.0, A42, 0.0, A44] .* dt;

B21 = 2 * Kf / M;
B41 = 2 * Kf * lf / I;
system.B = [0.0;
            B21;
            0.0;
            B41] .* dt;
            
system.xl = xmin;
system.xu = xmax;
system.ul = umin;
system.uu = umax;

system.target = x_target;

%% Load the trajectory
t = 0:dt:7; % Use 7 second for lane change
[x_ref, y_ref, psi_ref] = trajectory_generator(t, Vx);    

sim_length = length(t); % Number of control loop iterations

refSignals = zeros(length(x_ref(:, 2)) * 3, 1);

k = 1;
for i = 1:controlled_states:length(refSignals)
    refSignals(i)     = psi_ref(k, 2);
    refSignals(i + 1) = Y_ref(k, 2);
    k = k + 1;
end

%% Load the initial state
x0 = [y_ref(1, 2); 0.0; psi_ref(1, 2); 0.0]; % Initial state of Lateral Vehicle Dynamics

%% MPC parameters
params_mpc.Q = Q;
params_mpc.R = R;
params_mpc.P = P;
params_mpc.N = N;
params_mpc.p = p;

%% main loop
xTrue(:, 1) = x0;
uk(:, 1) = [0; 0];
time = 3.0;
time_curr = 0.0;
current_step = 1;
solvetime = zeros(1, time / dt);

while time_curr <= time
    % update time
    time_curr = time_curr + system.dt;
    current_step = current_step + 1;
    
    % solve mac
    tic;
    uk(:, current_step) = mpc(xTrue(:, current_step - 1), system, obs, params_mpc);
    solvetime(1, current_step - 1) = toc;
    
    % update state
    xTrue(:, current_step) = system.A * xTrue(:, current_step - 1) + system.B * uk(:, current_step);
end

%% solve average time
avg_time = sum(solvetime) / current_step;
disp(avg_time);

drow_figure(xTrue, uk, obs, x_target, current_step);

%% trajectory generator
function [x_ref, y_ref, psi_ref] = trajectory_generator(t, Vx)    
    % linspace 
    % ex) 区間 [-5,5] 内の 7 個の等間隔の点のベクトルを作成します
    % y1 = linspace(-5,5,7)
    
    x = linspace(0, Vx * t(end), length(t));
    y = 3 * tanh((t - t(end) / 2));
    
    dx = x(2:end) - x(1:end-1);
    dy = y(2:end) - y(1:end-1);

    psi    = zeros(1,length(x));
    psiInt = psi;

    psi(1)     = atan2(dy(1),dx(1));
    psi(2:end) = atan2(dy(:),dx(:));
    dpsi       = psi(2:end) - psi(1:end-1);

    psiInt(1) = psi(1);
    for i = 2:length(psiInt)
        if dpsi(i - 1) < -pi
            psiInt(i) = psiInt(i - 1) + (dpsi(i - 1) + 2 * pi);
        elseif dpsi(i - 1) > pi
            psiInt(i) = psiInt(i - 1) + (dpsi(i - 1) - 2 * pi);
        else
            psiInt(i) = psiInt(i - 1) + dpsi(i - 1);
        end
    end

    x_ref   = [t' x'];
    y_ref   = [t' y'];
    psi_ref = [t' psiInt'];
    
%     figure(1);
%     plot(x, y);
end

%% model predictive control
function uopt = mpc(xTrue, system, obs, params_mpc)
    % Solve MPC
    [feas, ~, u, ~] = solve_mpc(xTrue, system, obs, params_mpc);
    if ~feas
        uopt = [];
        return
    else
        uopt = u(:, 1);
    end
end
      
function [feas, xopt, uopt, Jopt] = solve_mpc(xTrue, system, obs, params)
    % extract variables
    N = params.N;
    % define variables and cost
    x = sdpvar(system.nx, N+1);
    u = sdpvar(system.nu, N);
    delta = sdpvar(1, N);
    constraints = [];
    cost = 0;
    
    % initial constraint
    constraints = [constraints; x(:,1) == xTrue];
    % add constraints and costs
    for i = 1:N
        constraints = [constraints;
            system.xl <= x(:,i) <= system.xu;
            system.ul <= u(:,i) <= system.uu
            x(:,i+1) == system.A * x(:,i) + system.B * u(:,i)];
        cost = cost + (x(:,i) - system.target)' * params.Q * (x(:,i) - system.target) + u(:,i)' * params.R * u(:,i);
    end
    % add collition avoidance constraints
    for i = 1:N
        pos = obs.pos;
        r = obs.r;
        distance = (x([1:2],i)-pos)'*((x([1:2],i)-pos)) - r^2;
        constraints = [constraints; distance >= delta(1, i)];
        cost = cost + delta(1, i)' * params.p * delta(1, i);
    end
    % add terminal cost
    cost = cost + (x(:,N+1) - system.target)' * params.P * (x(:,N+1) - system.target);
    ops = sdpsettings('solver','ipopt','verbose',0);
    % solve optimization
    diagnostics = optimize(constraints, cost, ops);
    if diagnostics.problem == 0
        feas = true;
        xopt = value(x);
        uopt = value(u);
        Jopt = value(cost);
    else
        feas = false;
        xopt = [];
        uopt = [];
        Jopt = [];
    end
end
        
function drow_figure(xlog, ulog, obs, x_target, current_step)
    % Plot simulation
    figure(1)
    subplot(2, 1, 1)
    plot(0:current_step - 1, ulog(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('delta [rad]','interpreter','latex','FontSize',10);
    yline(1.0, '--b', 'LineWidth', 2.0);
    yline(-1.0, '--b', 'LineWidth', 2.0);
    
    subplot(2, 1, 2);
    plot(0:current_step - 1, ulog(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Acceleration [$m/s^2$]','interpreter','latex','FontSize',10);
    yline(5.0, '--b', 'LineWidth', 2.0);
    yline(-5.0, '--b', 'LineWidth', 2.0);
    
    figure(2)
    % plot trajectory
    plot(xlog(1,:), xlog(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    hold on;
    grid on;
    axis equal;
    % plot initial position
    plot(xlog(1, 1), xlog(1, 2), 'dm', 'LineWidth', 2);
    % plot target position
    plot(x_target(1), x_target(2), 'dg', 'LineWidth', 2);
    xlabel('X[m]','interpreter','latex','FontSize',10);
    ylabel('Y[m]','interpreter','latex','FontSize',10);
    % plot obstacle
    pos = obs.pos;
    r = obs.obstacle_diameter;
    th = linspace(0,2*pi*100);
    x = cos(th); 
    y = sin(th);
    plot(pos(1) + r*x, pos(2) + r*y, 'r', 'LineWidth', 2);
    plot(pos(1), pos(2), 'r', 'MarkerSize', 5, 'LineWidth', 2);
    % plot vegicle
    for i = 1:current_step
        r_vegicle = obs.own_vehicle_diameter;
        th = linspace(0,2*pi*100);
        x = cos(th); 
        y = sin(th);
        X = xlog(1,i);
        Y = xlog(2,i);
        plot(X + r_vegicle * x, Y + r_vegicle * y, 'b','LineWidth', 2);
        hold on;
    end
    yline(0.5, '--k', 'LineWidth', 2.0);
    yline(2.0, 'k', 'LineWidth', 4.0);
    yline(-1.0, 'k', 'LineWidth', 4.0);
    
    legend('Motion trajectory','Initial position', 'Target position','Obstacle', 'Location','southeast',...
           'interpreter','latex','FontSize',10.0);
end
