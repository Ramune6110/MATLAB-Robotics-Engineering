clear;
close all;
clc;

%% Setup and Parameters
dt   = 0.01; % sample time
dimx = 6;    % Number of states
dimu = 6;    % Number of input(u1 = thrust1, u2 = thrust1, u3 = dummy input, u4 = dummy input, u5 = Lagrange multiplier1, u6 = Lagrange multiplier2)
diml = 6;    % Number of companion variable

sim_time = 20; % Simulation time [s]

%% Semi active dumper
system.M = 0.894;  % [kg]
system.I = 0.0125; % [kgÅEm^2]
system.r = 0.0485; % [m]

%% NMPC parameters
params_nmpc.tf = 1.0;                                         % Final value of prediction time [s]
params_nmpc.N = 10;                                           % Number of divisions of the prediction time [-]
params_nmpc.kmax = 20;                                        % Number of Newton's method
params_nmpc.hdir = 0.0001;                                    % step in the Forward Difference Approximation

params_nmpc.x0 = [ -0.25; 0.35; 0.0; 0.0; 0.0; 0.0 ];         % Initial state
params_nmpc.u0 = [ 0.2; 0.1; 0.2; 0.03; 0.01; 0.02 ];         % Initial u
params_nmpc.xf = [ 0.0; 0.0; 0.0; 0.0; 0.0; 0.0 ];            % target state

params_nmpc.sf = [ 10; 15; 0.1; 1; 1; 0.01 ];                 % Termination cost weight matrix
params_nmpc.q  = [ 10; 15; 0.1; 1; 1; 0.01 ];                 % Weight matrix of state quantities
params_nmpc.r  = [ 1; 1; 0.001; 0.001 ];                      % Weight matrix of input quantities
params_nmpc.ki = [ 0.28; 0.28 ];                              % Weight matrix of dummy input

params_nmpc.umin = -0.121;                                    % lower input limit [N]
params_nmpc.umax = 0.342;                                     % upper input limit [N]
params_nmpc.uavg = (params_nmpc.umax + params_nmpc.umin) / 2; % average input limit [N]

params_nmpc.dimx = dimx;                                      % Number of states
params_nmpc.dimu = dimu;                                      % Number of input(u1 = Attenuation coefficient, u2 = Dummy input, u3 = Lagrange multiplier)
params_nmpc.diml = diml;                                      % Number of companion variable
 
%% define systems
[xv, lmdv, uv, muv, fxu, Cxu] = defineSystem(system, params_nmpc);
[L, phi, qv, rv, kv, sfv]     = evaluationFunction(xv, uv, params_nmpc);
[obj, H, Hx, Hu, Huu, phix]   = generate_Euler_Lagrange_Equations(xv, lmdv, uv, muv, fxu, Cxu, L, phi, qv, rv, kv, sfv);

%% Initial input value calculation using Newton's method
lmd0 = obj.phix(params_nmpc.x0, params_nmpc.sf);

for cnt = 1:params_nmpc.kmax
    params_nmpc.u0 = params_nmpc.u0 - obj.Huu( params_nmpc.u0, params_nmpc.r ) \ obj.Hu( params_nmpc.x0, params_nmpc.u0, lmd0, params_nmpc.r, params_nmpc.ki );
end

%% main loop
xTrue(:, 1) = params_nmpc.x0;
uk(:, 1) = params_nmpc.u0;
uk_horizon(:, 1) = repmat( params_nmpc.u0, params_nmpc.N, 1 );
current_step = 1;
sim_length = length(1:dt:sim_time);
normF = zeros(1, sim_length + 1);
solvetime = zeros(1, sim_length + 1);

for i = 1:dt:sim_time
    i
    % update time
    current_step = current_step + 1;
    
    % solve nmpc
    tic;
    [uk(:, current_step), uk_horizon(:, current_step), algebraic_equation] = NMPC( obj, xTrue(:, current_step - 1), uk_horizon(:, current_step - 1), system, params_nmpc);
    solvetime(1, current_step - 1) = toc;
    
    % optimality error norm F
    total = 0;
    for j = 1:params_nmpc.dimu * params_nmpc.N
        total = total + algebraic_equation(j, :)^2;
    end
    normF(1, current_step - 1) = sqrt(total);
    
    % update state
    T = dt*current_step:dt:dt*current_step+dt;
    [T, x] = ode45(@(t,x) nonlinear_model(x, uk(:, current_step), system), T, xTrue(:, current_step - 1));
    xTrue(:, current_step) = x(end,:);
end

%% solve average time
avg_time = sum(solvetime) / current_step;
disp(avg_time);

plot_figure(xTrue, uk, normF, params_nmpc, current_step);

%% define systems
function [xv, lmdv, uv, muv, fxu, Cxu] = defineSystem(system, nmpc)
    syms x1 x2 x3 x4 x5 x6 lmd1 lmd2 lmd3 lmd4 lmd5 lmd6 u1 u2 u3 u4 u5 u6
    xv   = [x1; x2; x3; x4; x5; x6];             % state vector
    lmdv = [lmd1; lmd2; lmd3; lmd4; lmd5; lmd6]; % costate vector
    uv   = [u1; u2; u3; u4];                     % control input(include dummy input)
    muv  = [u5; u6];                             % Lagrange multiplier vectors for equality constraints
    
    % state equation
    fxu = [xv(4);
           xv(5);
           xv(6);
           ( uv(1) * cos(xv(3)) + uv(2) * cos(xv(3)) ) / system.M;
           ( uv(1) * sin(xv(3)) + uv(2) * sin(xv(3)) ) / system.M;
           ( uv(1) * system.r + uv(2) * system.r ) / system.I];          
    % constraints
    Cxu = [(uv(1) - nmpc.uavg)^2 + uv(3)^2 - (nmpc.umax - nmpc.uavg)^2;
           (uv(2) - nmpc.uavg)^2 + uv(4)^2 - (nmpc.umax - nmpc.uavg)^2]; 
end

function [L, phi, qv, rv, kv, sfv] = evaluationFunction(xv, uv, nmpc)
    syms q1 q2 q3 q4 q5 q6 r1 r2 r3 r4 k1 k2 sf1 sf2 sf3 sf4 sf5 sf6
    qv  = [ q1; q2; q3; q4; q5; q6 ];           % vector for diagonal matrix Q
    rv  = [ r1; r2; r3; r4 ];                   % vector for diagonal matrix R
    kv  = [ k1; k2 ];                           % weight vector
    sfv = [ sf1; sf2; sf3; sf4; sf5; sf6 ];     % vector for diagonal matrix Sf
    Q   = diag([q1, q2, q3, q4, q5, q6]);       % square matrix
    R   = diag([r1, r2, r3, r4]);               % square matrix
    Sf  = diag([sf1, sf2, sf3, sf4, sf5, sf6]); % square matrix
    
    % stage cost
    L   = simplify( ((xv - nmpc.xf)' * Q * (xv - nmpc.xf)) / 2 + (uv' * R * uv) / 2 - kv(1) * uv(3) - kv(2) * uv(4) ); 
    % terminal cost
    phi = simplify( ((xv - nmpc.xf)' * Sf * (xv - nmpc.xf)) / 2 );                                      
end

function [obj, H, Hx, Hu, Huu, phix] = generate_Euler_Lagrange_Equations(xv, lmdv, uv, muv, fxu, Cxu, L, phi, qv, rv, kv, sfv)
    H = simplify(L + lmdv' * fxu + muv' * Cxu); % H = Hamiltonian
    uv = [uv; muv];                             % extend input
    
    Hx   = simplify(gradient(H, xv));           % Hx = Å›H / Å›x
    Hu   = simplify(gradient(H, uv));           % Hu = Å›H / Å›u
    Huu  = simplify(jacobian(Hu, uv));          % Huu = Å›^2H / Å›u^2
    phix = simplify(gradient(phi, xv));         % Huu = Å›É” / Å›x
    
    obj.Hx   = matlabFunction(Hx, 'vars', {xv, uv, lmdv, qv});
    obj.Hu   = matlabFunction(Hu, 'vars', {xv, uv, lmdv, rv, kv});
    obj.Huu  = matlabFunction(Huu, 'vars', {uv, rv});
    obj.phix = matlabFunction(phix, 'vars', {xv, sfv});
end

%% NMPC
function [uk, uk_horizon, algebraic_equation] = NMPC( obj, x_current, uk_horizon, system, nmpc )
    for cnt = 1:nmpc.kmax
        uk_horizon = uk_horizon - ( dFdu( obj, x_current, uk_horizon, system, nmpc ) \ F( obj, x_current, uk_horizon, system, nmpc ) );
    end
    
    algebraic_equation = F( obj, x_current, uk_horizon, system, nmpc );
    uk = uk_horizon(1:nmpc.dimu);
end

%% calculation dF = Å›F / Å›u
function dF = dFdu( obj, x_current, u, system, nmpc )
    dF = zeros( nmpc.dimu * nmpc.N, nmpc.dimu * nmpc.N );
    diff = nmpc.hdir;
    
    for cnt = 1:nmpc.N*nmpc.dimu
        u_buff_p = u;
        u_buff_n = u;

        u_buff_p(cnt) = u_buff_p(cnt) + diff;
        u_buff_n(cnt) = u_buff_n(cnt) - diff;
        
        % dF = Å›F / Å›u is the Forward Difference Approximation
        dF(:,cnt) = ( F( obj, x_current, u_buff_p, system, nmpc ) - F( obj, x_current, u_buff_n, system, nmpc ) ) / ( diff );
    end
end

%% calculation F = nonlinear algebraic equation
function ans_F = F( obj, x_current, u, system, nmpc )
    x   = Forward( x_current, u, system, nmpc );
    lmd = Backward( obj, x, u, nmpc );

    ans_F = zeros( nmpc.dimu * nmpc.N, 1 );

    for cnt = 1:nmpc.N
        ans_F((1:nmpc.dimu)+nmpc.dimu*(cnt-1)) = obj.Hu(x((1:nmpc.dimx)+nmpc.dimx*(cnt-1)), ...
                                                        u((1:nmpc.dimu)+nmpc.dimu*(cnt-1)), ...
                                                        lmd((1:nmpc.diml)+nmpc.diml*(cnt-1)),...
                                                        nmpc.r,...
                                                        nmpc.ki);
    end
end

%% Prediction of the state from the current time to T seconds in the future (Euler approximation)
function x = Forward( x0, u, system, nmpc )
    dt = nmpc.tf / nmpc.N;
    
    x = zeros( nmpc.dimx * nmpc.N, 1 );
    x(1:nmpc.dimx) = x0;

    for cnt = 1 : nmpc.N-1
       x((1:nmpc.dimx)+nmpc.dimx*(cnt)) = x((1:nmpc.dimx)+nmpc.dimx*(cnt-1)) ...
                                        + nonlinear_model( x((1:nmpc.dimx)+nmpc.dimx*(cnt-1)), u((1:nmpc.dimu)+nmpc.dimu*(cnt-1)), system ) * dt; 
    end
end

%% Prediction of adjoint variables from current time to T seconds in the future (Euler approximation)
function lmd = Backward( obj, x, u, nmpc )
    dt = nmpc.tf / nmpc.N;
    
    lmd = zeros( nmpc.diml * nmpc.N, 1 );
    lmd((1:nmpc.diml)+nmpc.diml*(nmpc.N-1)) = obj.phix( x((1:nmpc.dimx)+nmpc.dimx*(nmpc.N-1)), nmpc.sf );
    
    for cnt = nmpc.N-1:-1:1                                                    
        lmd((1:nmpc.diml)+nmpc.diml*(cnt-1)) = lmd((1:nmpc.diml)+nmpc.diml*(cnt)) ...
                                             + obj.Hx( x((1:nmpc.dimx)+nmpc.dimx*(cnt)), u, lmd((1:nmpc.diml)+nmpc.diml*(cnt)), nmpc.q ) * dt;
    end
end

%% nonlinear state equation
function dx = nonlinear_model(xTrue, u, system)
    % dx = f( xTrue, u )
    dx = [xTrue(4);
          xTrue(5);
          xTrue(6);
          ( u(1) * cos(xTrue(3)) + u(2) * cos(xTrue(3)) ) / system.M;
          ( u(1) * sin(xTrue(3)) + u(2) * sin(xTrue(3)) ) / system.M;
          ( u(1) * system.r + u(2) * system.r ) / system.I];
end

%% plot figure
function plot_figure(xTrue, uk, normF, nmpc, current_step)
    % plot state 
    figure(1)
    subplot(3, 1, 1)
    plot(0:current_step - 1, xTrue(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('X [m]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    subplot(3, 1, 2)
    plot(0:current_step - 1, xTrue(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Y [m]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
     
    subplot(3, 1, 3)
    plot(0:current_step - 1, xTrue(3,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\theta$ [rad]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    % plot state 
    figure(2);
    subplot(3, 1, 1)
    plot(0:current_step - 1, xTrue(4,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\dot{X} [m/s]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    subplot(3, 1, 2)
    plot(0:current_step - 1, xTrue(5,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\dot{Y} [m/s]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
     
    subplot(3, 1, 3)
    plot(0:current_step - 1, xTrue(6,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\dot{\theta}$ [rad/s]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    % plot input
    figure(3);
    subplot(2, 1, 1)
    plot(0:current_step - 1, uk(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$u_{1}$ [N]','interpreter','latex','FontSize',10);
    subplot(2, 1, 2)
    plot(0:current_step - 1, uk(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$u_{2}$ [N]','interpreter','latex','FontSize',10);
    
    % plot dummy input
    figure(4);
    subplot(2, 1, 1)
    plot(0:current_step - 1, uk(3,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('dummy input $u_{d1}$','interpreter','latex','FontSize',10);
    subplot(2, 1, 2)
    plot(0:current_step - 1, uk(4,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('dummy input $u_{d2}$','interpreter','latex','FontSize',10);
    
    % plot Lagrange multiplier
    figure(5);
    subplot(2, 1, 1)
    plot(0:current_step - 1, uk(5,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Lagrange multiplier $\mu_{1}$','interpreter','latex','FontSize',10);
    subplot(2, 1, 2)
    plot(0:current_step - 1, uk(6,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Lagrange multiplier $\mu_{2}$','interpreter','latex','FontSize',10);
    
    % plot optimality error
    figure(6);
    plot(0:current_step - 1, normF(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('optimality error norm','interpreter','latex','FontSize',10);
    
    % plot trajectory
    figure(7)
    plot(xTrue(1,:), xTrue(2,:), 'r',...
        'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    % plot target position
    target = nmpc.xf;
    plot(target(1), target(2), 'dg', 'LineWidth', 2);
    xlabel('X [m]','interpreter','latex','FontSize',10);
    ylabel('Y [m]','interpreter','latex','FontSize',10);
    legend('Motion trajectory', 'Target position', 'Location','southeast',...
           'interpreter','latex','FontSize',10.0);
end