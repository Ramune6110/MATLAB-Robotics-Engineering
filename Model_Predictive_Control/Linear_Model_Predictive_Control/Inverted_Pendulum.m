clear;
close all;
clc;

%% Ipopt solver path
%addpath('C:\Users\ardui\Desktop\MATLAB_MK_2021\Ipopt');

%% Setup and Parameters
x0 = [-0.02; 0.0; 0.1; 0.0]; % Initial state of the inverted pendulum

dt = 0.1; % sample time
nx = 4;   % Number of states
nu = 1;   % Number of input

P = 30 * eye(nx);
Q = diag([1.0, 1.0, 1.0, 1.0]);
R = 0.01;
N = 10;  % Predictive Horizon

% Range of states and inputs
xmin = [-5; -5; -5; -5];
xmax = [5; 5; 5; 5];
umin = -5;
umax = 5;

%% Linear Inverted Pendulum
M = 1.0;  % [kg]
m = 0.3;  % [kg]
g = 9.8;  % [m/s^2]
l = 2.0;  % [m]
system.dt = dt;
system.nx = nx;
system.nu = nu;
system.A = eye(nx) + [0.0, 1.0, 0.0, 0.0;
                      0.0, 0.0, m * g / M, 0.0;
                      0.0, 0.0, 0.0, 1.0;
                      0.0, 0.0, g * (M + m) / (l * M), 0.0] .* dt;
system.B = [0.0; 
            1.0 / M; 
            0.0; 
            1.0 / (l * M)] .* dt;
system.xl = xmin;
system.xu = xmax;
system.ul = umin;
system.uu = umax;

%% MPC parameters
params_mpc.Q = Q;
params_mpc.R = R;
params_mpc.P = P;
params_mpc.N = N;

%% main loop
xTrue(:, 1) = x0;
uk(:, 1) = 0;
time = 20.0;
time_curr = 0.0;
current_step = 1;
solvetime = zeros(1, time / dt);

while time_curr <= time
    % update time
    time_curr = time_curr + system.dt;
    current_step = current_step + 1;
    
    % solve mac
    tic;
    [~, uk(current_step)] = mpc(xTrue(:, current_step - 1), system, params_mpc);
    solvetime(1, current_step - 1) = toc;
    
    % update state
    xTrue(:, current_step) = system.A * xTrue(:, current_step - 1) + system.B * uk(current_step);
end

% solve average time
avg_time = sum(solvetime) / current_step;
disp(avg_time);

drow_figure(xTrue, uk, current_step);

%% model predictive control
function [xopt, uopt] = mpc(xk, system, params_mpc)
    % Solve MPC
    [feas, x, u, ~] = solve_mpc(xk, system, params_mpc);
    if ~feas
        xopt = [];
        uopt = [];
        return
    else
        xopt = x(:,2);
        uopt = u(:,1);
    end
end
      
function [feas, xopt, uopt, Jopt] = solve_mpc(xk, system, params)
    % extract variables
    N = params.N;
    % define variables and cost
    x = sdpvar(system.nx, N+1);
    u = sdpvar(system.nu, N);
    constraints = [];
    cost = 0;
    
    % initial constraint
    constraints = [constraints; x(:,1) == xk];
    % add constraints and costs
    for i = 1:N
        constraints = [constraints;
            system.xl <= x(:,i) <= system.xu;
            system.ul <= u(:,i) <= system.uu
            x(:,i+1) == system.A * x(:,i) + system.B * u(:,i)];
        cost = cost + x(:,i)' * params.Q * x(:,i) + u(:,i)' * params.R * u(:,i);
    end
    % add terminal cost
    cost = cost + x(:,N+1)' * params.P * x(:,N+1);
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
        
function drow_figure(xlog, ulog, current_step)
    % Plot simulation
    figure(1)
    subplot(2,2,1)
    plot(0:current_step - 1, xlog(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('X[m]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    subplot(2,2,2)
    plot(0:current_step - 1, xlog(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\dot{X}$[m/s]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    subplot(2,2,3)
    plot(0:current_step - 1, xlog(3,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\theta$[rad]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    subplot(2,2,4)
    plot(0:current_step - 1, xlog(4,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\dot{\theta}$[rad/s]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    figure(2)
    plot(0:current_step - 1, ulog(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Input F[N]','interpreter','latex','FontSize',10);
    yline(5.0, '--b', 'LineWidth', 2.0);
    yline(-5.0, '--b', 'LineWidth', 2.0);
end
