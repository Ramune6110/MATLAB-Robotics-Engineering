clear;
close all;
clc;

%% Setup and Parameters
dt   = 0.01; % sample time
dimx = 2;    % Number of states
dimu = 3;    % Number of input(u1 = Attenuation coefficient, u2 = Dummy input, u3 = Lagrange multiplier)
dimc = 2;    % Number of companion variable

sim_time = 20; % Simulation time [s]

%% Semi active dumper
system.m = 1; %[kg]
system.k = 1; %[N/m]
system.a = - system.k / system.m;
system.b = - 1 / system.m;

%% NMPC parameters
params_nmpc.tf = 1.0;             % Final value of prediction time [s]
params_nmpc.N = 5;                % Number of divisions of the prediction time [-]
params_nmpc.kmax = 20;            % Number of Newton's method
params_nmpc.hdir = 0.01;          % step in the Forward Difference Approximation

params_nmpc.x0 = [2;0];           % Initial state
params_nmpc.u0 = [0.01;0.9;0.03]; % Initial u

params_nmpc.sf = [ 1;10 ];        % Termination cost weight matrix
params_nmpc.q = [ 1;10 ];         % Weight matrix of state quantities
params_nmpc.r = [ 1;0.01 ];       % Weight matrix of input quantities

params_nmpc.umin = 0;             % upper input limit
params_nmpc.umax = 1;             % lower input limit

params_nmpc.dimx = dimx;          % Number of states
params_nmpc.dimu = dimu;          % Number of input(u1 = Attenuation coefficient, u2 = Dummy input, u3 = Lagrange multiplier)
params_nmpc.dimc = dimc;          % Number of companion variable
 
%% define systems
[xv, lmdv, uv, muv, fxu, Cxu] = defineSystem(system, params_nmpc);
[L, phi, qv, rv, sfv]         = evaluationFunction(xv, uv);
[obj, H, Hx, Hu, Huu, phix]   = generate_Euler_Lagrange_Equations(xv, lmdv, uv, muv, fxu, Cxu, L, phi, qv, rv, sfv);

%% Initial input value calculation using Newton's method
lmd0 = obj.phix(params_nmpc.x0, params_nmpc.sf);

for cnt = 1:params_nmpc.kmax
    params_nmpc.u0 = params_nmpc.u0 - obj.Huu( params_nmpc.u0, params_nmpc.r ) \ obj.Hu( params_nmpc.x0, params_nmpc.u0, lmd0, params_nmpc.r );
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

plot_figure(xTrue, uk, normF, current_step);

%% define systems
function [xv, lmdv, uv, muv, fxu, Cxu] = defineSystem(system, nmpc)
    syms x1 x2 lmd1 lmd2 u1 u2 u3
    xv   = [x1; x2];     % state vector
    lmdv = [lmd1; lmd2]; % costate vector
    uv   = [u1; u2];     % control input
    muv  = u3;           % multiple vector
    
    fxu = [xv(2);
           system.a * xv(1) + system.b* xv(2) * uv(1)];            % state equation
    Cxu = (uv(1) - nmpc.umax / 2)^2 + uv(2)^2 - (nmpc.umax / 2)^2; % constraints
end

function [L, phi, qv, rv, sfv] = evaluationFunction(xv, uv)
    syms q1 q2 r1 r2 sf1 sf2
    qv  = [q1; q2];         % vector for diagonal matrix Q
    rv  = [r1; r2];         % weight vector
    sfv = [sf1; sf2];       % vector for diagonal matrix Sf
    Q   = diag([q1, q2]);   % square matrix
    R   = diag([r1, r2]);   % square matrix
    Sf  = diag([sf1, sf2]); % square matrix
    
    L   = simplify( (xv' * Q * xv) / 2 + rv(1) * uv(1)^2 / 2 - rv(2) * uv(2) ); % Integrand in the performance matrix
    phi = simplify( (xv' * Sf * xv) / 2 );                                      % terminal penalty
end

function [obj, H, Hx, Hu, Huu, phix] = generate_Euler_Lagrange_Equations(xv, lmdv, uv, muv, fxu, Cxu, L, phi, qv, rv, sfv)
    H = simplify(L + lmdv' * fxu + muv' * Cxu); % H = Hamiltonian
    uv = [uv; muv];                             % extend input
    
    Hx   = simplify(gradient(H, xv));   % Hx = Å›H / Å›x
    Hu   = simplify(gradient(H, uv));   % Hu = Å›H / Å›u
    Huu  = simplify(jacobian(Hu, uv));  % Huu = Å›^2H / Å›u^2
    phix = simplify(gradient(phi, xv)); % Huu = Å›É” / Å›x
    
    obj.Hx   = matlabFunction(Hx, 'vars', {xv, uv, lmdv, qv});
    obj.Hu   = matlabFunction(Hu, 'vars', {xv, uv, lmdv, rv});
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
                                                        lmd((1:nmpc.dimc)+nmpc.dimc*(cnt-1)),...
                                                        nmpc.r );
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
    
    lmd = zeros( nmpc.dimc * nmpc.N, 1 );
    lmd((1:nmpc.dimc)+nmpc.dimc*(nmpc.N-1)) = obj.phix( x((1:nmpc.dimx)+nmpc.dimx*(nmpc.N-1)), nmpc.sf );
    
    for cnt = nmpc.N-1:-1:1                                                    
        lmd((1:nmpc.dimc)+nmpc.dimc*(cnt-1)) = lmd((1:nmpc.dimc)+nmpc.dimc*(cnt)) ...
                                             + obj.Hx( x((1:nmpc.dimx)+nmpc.dimx*(cnt)), u, lmd((1:nmpc.dimc)+nmpc.dimc*(cnt)), nmpc.q ) * dt;
    end
end

%% nonlinear state equation
function dx = nonlinear_model(xTrue, u, system)
    % dx = f( xTrue, u )
    dx = [xTrue(2);
          system.a * xTrue(1) + system.b * xTrue(2) * u(1)]; 
end

%% plot figure
function plot_figure(xTrue, uk, normF, current_step)
    % plot state
    figure(1)
    subplot(2, 1, 1)
    plot(0:current_step - 1, xTrue(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('y [m]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    subplot(2, 1, 2)
    plot(0:current_step - 1, xTrue(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\dot{y}$ [m/s]','interpreter','latex','FontSize',10);
    yline(0.0, '--r', 'LineWidth', 1.0);
    
    % plot input
    figure(2);
    subplot(2, 1, 1)
    plot(0:current_step - 1, uk(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Attenuation coefficient [N/(m/s)]','interpreter','latex','FontSize',10);
    subplot(2, 1, 2)
    plot(0:current_step - 1, uk(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('dummy input $\upsilon$','interpreter','latex','FontSize',10);
    
    % plot Lagrange multiplier
    figure(3);
    plot(0:current_step - 1, uk(3,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('Lagrange multiplier $\mu$','interpreter','latex','FontSize',10);
    
    % plot optimality error
    figure(4);
    plot(0:current_step - 1, normF(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('optimality error norm','interpreter','latex','FontSize',10);
end