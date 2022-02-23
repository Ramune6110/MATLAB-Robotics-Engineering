clear;
close all;
clc;

%% Choice the type of trajectory
while true
    model = input('Please choose the type of trajectory (1 = Lane change, 2 = Circle): ');
    switch model
        case 1
            trajectory_type = 'Lane change';
            break;
        case 2
            trajectory_type = 'Circle';
            break;
        otherwise
            disp('No System!');
    end
end

%% Setup and Parameters
dt = 0.1; % sample time
nx = 4;   % Number of states
ny = 2;   % Number of ovservation
nu = 1;   % Number of input

S = diag([10.0, 1.0]); % Termination cost weight matrix
Q = diag([10.0, 1.0]); % Weight matrix of state quantities
R = 30.0;              % Weight matrix of input quantities
N = 15;                % Predictive Horizon

% Range of inputs
umin = -pi / 60;
umax = pi / 60;

%% Linear Car models (Lateral Vehicle Dynamics)
M = 1500;     % [kg]
I = 3000;     % [kgm^2]
lf = 1.2;     % [m]
lr = 1.6;     % [m]
l = lf + lr;  % [m]
Kf = 19000;   % [N/rad]
Kr = 33000;   % [N/rad]
Vx = 20;      % [m/s]

system.dt = dt;
system.nx = nx;
system.ny = ny;
system.nu = nu;
        
% states = [y_dot, psi, psi_dot, Y];
% Vehicle Dynamics and Control (2nd edition) by Rajesh Rajamani. They are in Chapter 2.3. 
A11 = -2 * (Kf + Kr)/(M * Vx);
A13 = -Vx - 2 * (Kf*lf - Kr*lr)/(M * Vx);
A31 = -2 * (lf*Kf - lr*Kr)/(I * Vx);
A33 = -2 * (lf^2*Kf + lr^2*Kr)/(I * Vx);
system.A = eye(nx) + [A11, 0.0, A13, 0.0;
                      0.0, 0.0, 1.0, 0.0;
                      A31, 0.0, A33, 0.0;
                      1.0, Vx, 0.0, 0.0] * dt;
        
B11 = 2*Kf/M;
B13 = 2*lf*Kf/I;
system.B = [B11;
            0.0;
            B13;
            0.0] * dt;
        
system.C = [0 1 0 0;
            0 0 0 1];
    
system.A_aug = [system.A, system.B;
                zeros(length(system.B(1,:)),length(system.A(1,:))),eye(length(system.B(1,:)))];
system.B_aug = [system.B;
                eye(length(system.B(1,:)))];
system.C_aug = [system.C,zeros(length(system.C(:,1)),length(system.B(1,:)))];

% Constants
system.Vx = Vx;
system.M  = M;
system.I  = I;
system.Kf = Kf;
system.Kr = Kr;
system.lf = lf;
system.lr = lr;

% input
system.ul = umin;
system.uu = umax;

%% Load the trajectory
if strcmp(trajectory_type, 'Lane change')
    t = 0:dt:10.0;
elseif strcmp(trajectory_type, 'Circle')  
    t = 0:dt:80.0;
end
[x_ref, y_ref, psi_ref] = trajectory_generator(t, Vx, trajectory_type);    

sim_length = length(t); % Number of control loop iterations

refSignals = zeros(length(x_ref(:, 1)) * ny, 1);

ref_index = 1;
for i = 1:ny:length(refSignals)
    refSignals(i)     = psi_ref(ref_index, 1);
    refSignals(i + 1) = y_ref(ref_index, 1);
    ref_index = ref_index + 1;
end

% initial state
x0 = [0.0; psi_ref(1, 1); 0.0; y_ref(1, 1)]; % Initial state of Lateral Vehicle Dynamics

%% MPC parameters
params_mpc.Q = Q;
params_mpc.R = R;
params_mpc.S = S;
params_mpc.N = N;

%% main loop
xTrue(:, 1) = x0;
uk(:, 1) = 0.0;
du(:, 1) = 0.0;
current_step = 1;
solvetime = zeros(1, sim_length);

ref_sig_num = 1; % for reading reference signals
for i = 1:sim_length-15
    % update time
    current_step = current_step + 1;
    
    % Generating the current state and the reference vector
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
    [T, x] = ode45(@(t,x) nonlinear_lateral_car_model(t, x, uk(:, current_step), system), T, xTrue(:, current_step - 1));
    xTrue(:, current_step) = x(end,:);
end

%% solve average time
avg_time = sum(solvetime) / current_step;
disp(avg_time);

drow_figure(xTrue, uk, du, x_ref, y_ref, current_step, trajectory_type);

%% model predictive control
function uopt = mpc(xTrue_aug, system, params_mpc, ref)
    A_aug = system.A_aug;
    B_aug = system.B_aug;
    C_aug = system.C_aug;
    
    Q = params_mpc.Q;
    S = params_mpc.S;
    R = params_mpc.R;
    N = params_mpc.N;
    
    CQC = C_aug' * Q * C_aug;
    CSC = C_aug' * S * C_aug;
    QC  = Q * C_aug; 
    SC  = S * C_aug;
    
    Qdb = zeros(length(CQC(:,1))*N,length(CQC(1,:))*N);
    Tdb = zeros(length(QC(:,1))*N,length(QC(1,:))*N);
    Rdb = zeros(length(R(:,1))*N,length(R(1,:))*N);
    Cdb = zeros(length(B_aug(:,1))*N,length(B_aug(1,:))*N);
    Adc = zeros(length(A_aug(:,1))*N,length(A_aug(1,:)));
    
    % Filling in the matrices
    for i = 1:N
       if i == N
           Qdb(1+length(CSC(:,1))*(i-1):length(CSC(:,1))*i,1+length(CSC(1,:))*(i-1):length(CSC(1,:))*i) = CSC;
           Tdb(1+length(SC(:,1))*(i-1):length(SC(:,1))*i,1+length(SC(1,:))*(i-1):length(SC(1,:))*i) = SC;           
       else
           Qdb(1+length(CQC(:,1))*(i-1):length(CQC(:,1))*i,1+length(CQC(1,:))*(i-1):length(CQC(1,:))*i) = CQC;
           Tdb(1+length(QC(:,1))*(i-1):length(QC(:,1))*i,1+length(QC(1,:))*(i-1):length(QC(1,:))*i) = QC;
       end
       
       Rdb(1+length(R(:,1))*(i-1):length(R(:,1))*i,1+length(R(1,:))*(i-1):length(R(1,:))*i) = R;
       
       for j = 1:N
           if j<=i
               Cdb(1+length(B_aug(:,1))*(i-1):length(B_aug(:,1))*i,1+length(B_aug(1,:))*(j-1):length(B_aug(1,:))*j) = A_aug^(i-j)*B_aug;
           end
       end
       Adc(1+length(A_aug(:,1))*(i-1):length(A_aug(:,1))*i,1:length(A_aug(1,:))) = A_aug^(i);
    end
    Hdb  = Cdb' * Qdb * Cdb + Rdb;
    Fdbt = [Adc' * Qdb * Cdb; -Tdb * Cdb];
    
    % Calling the optimizer (quadprog)
    % Cost function in quadprog: min(du)*1/2*du'Hdb*du+f'du
    ft = [xTrue_aug', ref'] * Fdbt;

    % Call the solver (quadprog)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = ones(1, N) * system.ul;
    ub  = ones(1, N) * system.uu;
    x0  = [];
    options = optimset('Display', 'off');
    [du, ~] = quadprog(Hdb,ft,A,b,Aeq,beq,lb,ub,x0,options);
    uopt = du(1);
end
      
%% trajectory generator
function [x_ref, y_ref, psi_ref] = trajectory_generator(t, Vx, trajectory_type)
    if strcmp(trajectory_type, 'Lane change')
        x = linspace(0, Vx * t(end), length(t));
        y = 3 * tanh((t - t(end) / 2));
    elseif strcmp(trajectory_type, 'Circle')  
        radius = 30;
        period = 1000;
        for i = 1:length(t)
            x(1, i) = radius * sin(2 * pi * i / period);
            y(1, i) = -radius * cos(2 * pi * i / period);
        end
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

function dx = nonlinear_lateral_car_model(t, xTrue, u, system)
    % In this simulation, the body frame and its transformation is used
    % instead of a hybrid frame. That is because for the solver ode45, it
    % is important to have the nonlinear system of equations in the first
    % order form. 
    
    % Get the constants from the general pool of constants
    Vx = system.Vx;
    M  = system.M;
    I  = system.I;
    Kf = system.Kf;
    Kr = system.Kr;
    lf = system.lf;
    lr = system.lr;
    
    % % Assign the states
    y_dot   = xTrue(1);
    psi     = xTrue(2);
    psi_dot = xTrue(3);

    % The nonlinear equation describing the dynamics of the car
    dx(1,1) = -(2*Kf+2*Kr)/(M * Vx)*y_dot+(-Vx-(2*Kf*lf-2*Kr*lr)/(M * Vx))*psi_dot+2*Kf/M*u;
    dx(2,1) = psi_dot;
    dx(3,1) = -(2*lf*Kf-2*lr*Kr)/(I * Vx)*y_dot-(2*lf^2*Kf+2*lr^2*Kr)/(I * Vx)*psi_dot+2*lf*Kf/I*u;
    dx(4,1) = sin(psi) * Vx + cos(psi) * y_dot;
end
        
function drow_figure(xTrue, uk, du, x_ref, y_ref, current_step, trajectory_type)
    % Plot simulation
    figure(1)
    subplot(2, 1, 1)
    plot(0:current_step - 1, du(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\Delta \delta$ [rad]','interpreter','latex','FontSize',10);
    yline(pi / 60, '--b', 'LineWidth', 2.0);
    yline(-pi / 60, '--b', 'LineWidth', 2.0);
    
    subplot(2, 1, 2)
    plot(0:current_step - 1, uk(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\delta$ [rad]','interpreter','latex','FontSize',10);
   
    figure(2)
    if strcmp(trajectory_type, 'Lane change')
        % plot trajectory
        plot(x_ref(1: current_step, 1),y_ref(1: current_step, 1),'b','LineWidth',2)
        hold on
        plot(x_ref(1: current_step, 1),xTrue(4, 1:current_step),'r','LineWidth',2)
        hold on;
        grid on;
        % plot initial position
        plot(x_ref(1, 1), y_ref(1, 1), 'dg', 'LineWidth', 2);
        xlabel('X[m]','interpreter','latex','FontSize',10);
        ylabel('Y[m]','interpreter','latex','FontSize',10);
        yline(0.0, '--k', 'LineWidth', 2.0);
        yline(10.0, 'k', 'LineWidth', 4.0);
        yline(-10.0, 'k', 'LineWidth', 4.0);
        ylim([-12 12])
        
        legend('Refernce trajectory','Motion trajectory','Initial position', 'Location','southeast',...
               'interpreter','latex','FontSize',10.0);
    elseif strcmp(trajectory_type, 'Circle')  
        % plot trajectory
        plot(x_ref(1: current_step, 1),y_ref(1: current_step, 1),'--b','LineWidth',2)
        hold on
        plot(x_ref(1: current_step, 1),xTrue(4, 1:current_step),'r','LineWidth',1)
        hold on;
        grid on;
        axis equal;
        % plot initial position
        plot(x_ref(1, 1), y_ref(1, 1), 'db', 'LineWidth', 2);
        xlabel('X[m]','interpreter','latex','FontSize',10);
        ylabel('Y[m]','interpreter','latex','FontSize',10);

        legend('Refernce trajectory','Motion trajectory','Initial position', 'Location','southeast',...
               'interpreter','latex','FontSize',10.0);
    end
end
