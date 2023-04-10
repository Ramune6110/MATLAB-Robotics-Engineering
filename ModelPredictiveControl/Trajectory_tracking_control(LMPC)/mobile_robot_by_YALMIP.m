clear;
close all;
clc;

%% Ipopt solver path
dir = pwd;
addpath(fullfile(dir,'Ipopt'))

%% Setup and Parameters
dt = 0.1; % sample time
nx = 3;   % Number of states
ny = 3;   % Number of ovservation
nu = 2;   % Number of input

S = diag([1.0, 1.0, 10.0]); % Termination cost weight matrix
Q = diag([1.0, 1.0, 10.0]); % Weight matrix of state quantities
R = diag([0.1, 0.001]);     % Weight matrix of input quantities
N = 5;                      % Predictive Horizon

% Range of inputs
umin = [-5; -4];
umax = [5; 4];

%% Mobile robot
Vx     = 0.0; % [m/s]
thetak = 0.0; % [rad]

system.dt = dt;
system.nx = nx;
system.ny = ny;
system.nu = nu;

% input
system.ul = umin;
system.uu = umax;

%% Load the trajectory
t = 0:dt:10.0;
[x_ref, y_ref, psi_ref] = trajectory_generator(t);    

sim_length = length(t); % Number of control loop iterations

refSignals = zeros(length(x_ref(:, 1)) * ny, 1);

ref_index = 1;
for i = 1:ny:length(refSignals)
    refSignals(i)     = x_ref(ref_index, 1);
    refSignals(i + 1) = y_ref(ref_index, 1);
    refSignals(i + 2) = psi_ref(ref_index, 1);
    ref_index = ref_index + 1;
end

% initial state
x0 = [x_ref(1, 1); y_ref(1, 1); psi_ref(1, 1)]; % Initial state of mobile robot

%% Create A, B, C matrix
% states = [x, y, psi];
[A, B, C] = model_system(Vx, thetak, dt);

system.A = A;
system.B = B;
system.C = C;

system.A_aug = [system.A, system.B;
                zeros(length(system.B(1,:)),length(system.A(1,:))),eye(length(system.B(1,:)))];
system.B_aug = [system.B;
                eye(length(system.B(1,:)))];
system.C_aug = [system.C,zeros(length(system.C(:,1)),length(system.B(1,:)))];

%% MPC parameters
params_mpc.Q = Q;
params_mpc.R = R;
params_mpc.S = S;
params_mpc.N = N;

%% main loop
xTrue(:, 1) = x0;
uk(:, 1) = [0.0; 0.0];
du(:, 1) = [0.0; 0.0];
current_step = 1;
solvetime = zeros(1, sim_length);

ref_sig_num = 1; % for reading reference signals
for i = 1:sim_length-15
    % update time
    current_step = current_step + 1;
    
    % Generating the current state and the reference vector
    [A, B, C] = model_system(Vx, thetak, dt);
    
    system.A = A;
    system.B = B;
    system.C = C;
    system.A_aug = [system.A, system.B;
                    zeros(length(system.B(1,:)),length(system.A(1,:))),eye(length(system.B(1,:)))];
    system.B_aug = [system.B;
                    eye(length(system.B(1,:)))];
    system.C_aug = [system.C,zeros(length(system.C(:,1)),length(system.B(1,:)))];
    
    xTrue_aug = [xTrue(:, current_step - 1); uk(:, current_step - 1)];
    
    ref_sig_num = ref_sig_num + ny;
    if ref_sig_num + ny * N - 1 <= length(refSignals)
        ref = refSignals(ref_sig_num:ref_sig_num+ny*N-1);
    else
        ref = refSignals(ref_sig_num:length(refSignals));
        N = N - 1;
    end
    
    % solve mac
    tic;
    du(:, current_step) = mpc(xTrue_aug, system, params_mpc, ref);
    solvetime(1, current_step - 1) = toc;
    
    % add du input
    uk(:, current_step) = uk(:, current_step - 1) + du(:, current_step);
    
    % update state
    T = dt*i:dt:dt*i+dt;
    [T, x] = ode45(@(t,x) nonlinear_lateral_car_model(t, x, uk(:, current_step)), T, xTrue(:, current_step - 1));
    xTrue(:, current_step) = x(end,:);
    X = xTrue(:, current_step);
    thetak = X(3);
end

%% solve average time
avg_time = sum(solvetime) / current_step;
disp(avg_time);

drow_figure(xTrue, uk, du, x_ref, y_ref, current_step);

%% model predictive control
function uopt = mpc(xTrue_aug, system, params, ref)
    % Solve MPC
    [feas, ~, u, ~] = solve_mpc(xTrue_aug, system, params, ref);
    if ~feas
        uopt = [];
        return
    else
        uopt = u(:, 1);
    end
end
      
function [feas, xopt, uopt, Jopt] = solve_mpc(xTrue_aug, system, params, ref)
    % extract variables
    N = params.N;
    % define variables and cost
    x = sdpvar(system.nx + system.nu, N+1);
    u = sdpvar(system.nu, N);
    constraints = [];
    cost = 0;
    ref_index = 1;
    
    % initial constraint
    constraints = [constraints; x(:,1) == xTrue_aug];
    % add constraints and costs
    for i = 1:N-1
        constraints = [constraints;
            system.ul <= u(:,i) <= system.uu
            x(:,i+1) == system.A_aug * x(:,i) + system.B_aug * u(:,i)];
        cost = cost + (ref(ref_index:ref_index + 2,1) - system.C_aug * x(:,i+1))' * params.Q * (ref(ref_index:ref_index + 2,1) - system.C_aug * x(:,i+1)) + u(:,i)' * params.R * u(:,i);
        ref_index = ref_index + system.ny;
    end
    % add terminal cost
    cost = cost + (ref(ref_index:ref_index + 2,1) - system.C_aug * x(:,N+1))' * params.S * (ref(ref_index:ref_index + 2,1) - system.C_aug * x(:,N+1));     
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
      
%% trajectory generator
function [x_ref, y_ref, psi_ref] = trajectory_generator(t)
    % Circle
    radius = 30;
    period = 100;
    for i = 1:length(t)
        x(1, i) = radius * sin(2 * pi * i / period);
        y(1, i) = -radius * cos(2 * pi * i / period);
    end
    
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

    x_ref   = x';
    y_ref   = y';
    psi_ref = psiInt';
end

function [A, B, C] = model_system(Vk,thetak,dt)
    A = [1.0, 0.0, -Vk*dt*sin(thetak);
         0.0 1.0 Vk*dt*cos(thetak);
         0.0, 0.0, 1.0];
    B = [dt*cos(thetak), -0.5*dt*dt*Vk*sin(thetak);
         dt*sin(thetak), 0.5*dt*dt*Vk*cos(thetak);
         0.0 dt];
    C = [1.0, 0.0, 0.0;
         0.0, 1.0, 0.0;
         0.0, 0.0, 1.0];
end

function dx = nonlinear_lateral_car_model(t, xTrue, u)
    % In this simulation, the body frame and its transformation is used
    % instead of a hybrid frame. That is because for the solver ode45, it
    % is important to have the nonlinear system of equations in the first
    % order form. 
   
    dx = [u(1)*cos(xTrue(3));
          u(1)*sin(xTrue(3));
          u(2)];
end
        
function drow_figure(xTrue, uk, du, x_ref, y_ref, current_step)
    % Plot simulation
    figure(1)
    subplot(2,2,1)
    plot(0:current_step - 1, du(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\Delta v$ [m/s]','interpreter','latex','FontSize',10);
    yline(-5, '--b', 'LineWidth', 2.0);
    yline(5, '--b', 'LineWidth', 2.0);
    
    subplot(2,2,2)
    plot(0:current_step - 1, du(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\Delta w$ [rad/s]','interpreter','latex','FontSize',10);
    yline(-4, '--b', 'LineWidth', 2.0);
    yline(4, '--b', 'LineWidth', 2.0);
    
    subplot(2,2,3)
    plot(0:current_step - 1, uk(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$v$ [m/s]','interpreter','latex','FontSize',10);
    
    subplot(2,2,4)
    plot(0:current_step - 1, uk(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$w$ [rad/s]','interpreter','latex','FontSize',10);
   
    figure(2)
    % plot trajectory
    plot(x_ref(1: current_step, 1),y_ref(1: current_step, 1),'--b','LineWidth',2)
    hold on
    plot(xTrue(1, 1: current_step), xTrue(2, 1:current_step),'r','LineWidth',2)
    hold on;
    grid on;
    axis equal;
    % plot initial position
    plot(x_ref(1, 1), y_ref(1, 1), 'dg', 'LineWidth', 2);
    xlabel('X[m]','interpreter','latex','FontSize',10);
    ylabel('Y[m]','interpreter','latex','FontSize',10);

    legend('Refernce trajectory','Motion trajectory','Initial position', 'Location','southeast',...
           'interpreter','latex','FontSize',10.0);
    
    % plot position error
    figure(3)
    Vertical_error = x_ref(1: current_step, 1) - xTrue(1, 1: current_step)';
    Lateral_error = y_ref(1: current_step, 1) - xTrue(2, 1:current_step)';
    
    subplot(2, 1, 1)
    plot(0:current_step - 1, Vertical_error, 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Vertical error [m]','interpreter','latex','FontSize',10);
    
    subplot(2, 1, 2)
    plot(0:current_step - 1, Lateral_error, 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Lateral error [m]','interpreter','latex','FontSize',10);
end
