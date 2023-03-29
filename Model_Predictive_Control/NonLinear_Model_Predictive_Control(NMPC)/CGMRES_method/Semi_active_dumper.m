clear;
close all;
clc;

%% Setup and Parameters
dt   = 0.01; % sample time
dimx = 2;    % Number of states
dimu = 3;    % Number of input(u1 = Attenuation coefficient, u2 = Dummy input, u3 = Lagrange multiplier)
diml = 2;    % Number of companion variable

%sim_time = 20; % Simulation time [s]

sim_time = 12; % Simulation time [s]

%sim_time = 2.8; % Simulation time [s]

%% Semi active dumper
system.m = 1; %[kg]
system.k = 1; %[N/m]
system.a = - system.k / system.m;
system.b = - 1 / system.m;

%% NMPC parameters
params_nmpc.tf = 1.0;             % Final value of prediction time [s]
params_nmpc.N = 5;                % Number of divisions of the prediction time [-]
params_nmpc.kmax = 20;            % Number of Newton's method
params_nmpc.hdir = 0.002;         % step in the Forward Difference Approximation

params_nmpc.x0 = [2;0];           % Initial state
params_nmpc.u0 = [0.01;0.9;0.03]; % Initial u

params_nmpc.sf = [ 1;10 ];        % Termination cost weight matrix
params_nmpc.q = [ 1;10 ];         % Weight matrix of state quantities
params_nmpc.r = [ 1;0.01 ];       % Weight matrix of input quantities

params_nmpc.umin = 0;             % lower input limit
params_nmpc.umax = 1;             % upper input limit

params_nmpc.dimx = dimx;          % Number of states
params_nmpc.dimu = dimu;          % Number of input(u1 = Attenuation coefficient, u2 = Dummy input, u3 = Lagrange multiplier)
params_nmpc.diml = diml;          % Number of companion variable
 
%% CGMRES parameters
cgmres.ht = 0.001;                      % ‘Oi·•ª‹ß—‚ÌŠÔ• (s)
cgmres.tf = 1.0;                        % —\‘ªŠÔ‚ÌÅI’l (s)
cgmres.zeta = 1000.0;                   % ‘€ì—Ê‚ÌˆÀ’è‰»ƒQƒCƒ“ (-)
cgmres.alpha = 0.5;                     % —\‘ªŠÔ‚Ìã¸‘¬“xƒQƒCƒ“ (-)
cgmres.threshold = 0.001;

% —\‘ªŠÔT‚ÌŒvZ—p’è”
buff = c2d( ss( tf( [ cgmres.tf ], [ (1/cgmres.alpha), 1 ] ) ), dt );
cgmres.T_outGain = buff.a;              % —\‘ªŠÔ·•ª•û’ö®
cgmres.T_inGain = buff.b;               % —\‘ªŠÔ·•ª•û’ö®

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
%du_horizon(:, 1) = repmat( params_nmpc.u0, params_nmpc.N, 1 );
du_horizon(:, 1) = repmat( zeros(params_nmpc.dimu, 1), params_nmpc.N, 1 );
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
    [uk(:, current_step), du_horizon(:, current_step), uk_horizon(:, current_step), algebraic_equation] = NMPC( obj, xTrue(:, current_step - 1), du_horizon(:, current_step - 1), uk_horizon(:, current_step - 1), system, params_nmpc, cgmres, dt);
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
    uv   = [u1; u2];     % control input(include dummy input)
    muv  = u3;           % Lagrange multiplier vectors for equality constraints
    
    % state equation
    fxu = [xv(2);
           system.a * xv(1) + system.b* xv(2) * uv(1)];
    % constraints
    Cxu = (uv(1) - nmpc.umax / 2)^2 + uv(2)^2 - (nmpc.umax / 2)^2; 
end

function [L, phi, qv, rv, sfv] = evaluationFunction(xv, uv)
    syms q1 q2 r1 r2 sf1 sf2
    qv  = [q1; q2];         % vector for diagonal matrix Q
    rv  = [r1; r2];         % weight vector
    sfv = [sf1; sf2];       % vector for diagonal matrix Sf
    Q   = diag([q1, q2]);   % square matrix
    R   = diag([r1, r2]);   % square matrix
    Sf  = diag([sf1, sf2]); % square matrix
    
    % stage cost
    L   = simplify( (xv' * Q * xv) / 2 + rv(1) * uv(1)^2 / 2 - rv(2) * uv(2) ); 
    % terminal cost
    phi = simplify( (xv' * Sf * xv) / 2 );                                      
end

function [obj, H, Hx, Hu, Huu, phix] = generate_Euler_Lagrange_Equations(xv, lmdv, uv, muv, fxu, Cxu, L, phi, qv, rv, sfv)
    H = simplify(L + lmdv' * fxu + muv' * Cxu); % H = Hamiltonian
    uv = [uv; muv];                             % extend input
    
    Hx   = simplify(gradient(H, xv));           % Hx = İH / İx
    Hu   = simplify(gradient(H, uv));           % Hu = İH / İu
    Huu  = simplify(jacobian(Hu, uv));          % Huu = İ^2H / İu^2
    phix = simplify(gradient(phi, xv));         % Huu = İƒÓ / İx
    
    obj.Hx   = matlabFunction(Hx, 'vars', {xv, uv, lmdv, qv});
    obj.Hu   = matlabFunction(Hu, 'vars', {xv, uv, lmdv, rv});
    obj.Huu  = matlabFunction(Huu, 'vars', {uv, rv});
    obj.phix = matlabFunction(phix, 'vars', {xv, sfv});
end

%% NMPC
function [uk, du_horizon, uk_horizon, algebraic_equation] = NMPC( obj, x_current, du_horizon, uk_horizon, system, nmpc, cgmres, dt )
    % CGMRES method
    du_horizon = GMRES( obj, x_current, du_horizon, uk_horizon, system, nmpc, cgmres );
    uk_horizon = uk_horizon + du_horizon * dt;
    
    % optimality error
    algebraic_equation = F( obj, x_current, uk_horizon, system, nmpc );
    uk = uk_horizon(1:nmpc.dimu);
    
    % satulation
    if (uk(1) >= 1.0) 
        uk(1) = 1.0;
    elseif (uk(1) <= 0)
        uk(1) = 0;
    end
end

%% ‘€ì—Ê‚Ì•Ï‰»—Ê‚ÌŒvZiC/GMRES–@j
function du_new = GMRES( obj, x_current, du, u, system, nmpc, cgmres )
    
    k = nmpc.dimu * nmpc.N; % GMRES‚ÌÅ‘åŒJ‚è•Ô‚µ‰ñ”i—£UŠÔ2“_‹«ŠE’l–â‘è‚Ì—v‘f”‚Æ“™‚µ‚¢j

    
    % GMRES–@‚Å‰ğ‚­‚×‚«–â‘è‚Ìİ’è
    
    dx = nonlinear_model( x_current, u, system ); % ó‘Ô•Ï‰»—Ê‚Ì‘Oi·•ª‹ß—
    
    Fxt = F( obj, x_current + ( dx * cgmres.ht ), u, system, nmpc ); % ”Ÿ”F‚Ìó‘Ô”÷•ª
    
    %F = F( x_current, u, T, system, nmpc ); % ”Ÿ”F
    
    Right = - cgmres.zeta * F( obj, x_current, u, system, nmpc ) - ( ( Fxt - F( obj, x_current, u, system, nmpc ) ) / cgmres.ht ); % Šù’m€‚ğ‰E•Ó‚É‚Ü‚Æ‚ß‚é
    
    Fuxt = F( obj, x_current + ( dx * cgmres.ht ), u + ( du * cgmres.ht ), system, nmpc ); % ”Ÿ”F‚Ì“ü—ÍCó‘Ô”÷•ª
    
    Left = ( ( Fuxt - Fxt ) / cgmres.ht ); % –¢’m€‚ğ¶•Ó‚É‚Ü‚Æ‚ß‚é
    
    
    % GMRES–@‚Å‚ÌŒvZ
    
    r0 = Right - Left; % ‰Šúc·
    
    du_new = zeros( nmpc.dimu * nmpc.N, 1 );
    
    v = zeros( nmpc.dimu * nmpc.N, k + 1 ); 
    v(:,1) = r0 / norm( r0 );

    h = zeros( k + 1 );

    y = zeros( nmpc.dimu * nmpc.N, 1 );
    y_pre = zeros( nmpc.dimu * nmpc.N, 1 );
    
    e = zeros( k + 1, 1 );
    e(1) = 1;
    
    for cnt = 1:k
        Fuxt = F( obj, x_current + ( dx * cgmres.ht ), u + ( v(:,cnt) * cgmres.ht ), system, nmpc );
        Av = ( ( Fuxt - Fxt ) / cgmres.ht );

        Sum = zeros( nmpc.dimu * nmpc.N, 1 );
    
        for cnt2 = 1:cnt
            h(cnt2,cnt) = Av' * ( v(:,cnt2) );
            Sum = Sum + h(cnt2,cnt) * v(:,cnt2);
        end

        v_est = Av - Sum;

        h(cnt+1,cnt) = norm( v_est );

        v(:,cnt+1) = v_est / h(cnt+1,cnt);

        y(1:cnt) = h(1:(cnt+1),1:cnt) \ ( norm( r0 ) * e(1:(cnt+1)) );
        
        % Œë·‚ª¬‚³‚­‚È‚Á‚½‚ç‘Å‚¿Ø‚è
        if ( norm( norm( r0 ) * e(1:(cnt+1)) - h(1:(cnt+1),1:cnt) * y(1:cnt) ) < cgmres.threshold )
            du_new = du + v(:,1:(cnt-1)) * y_pre(1:(cnt-1));
            break;
        end
        
        y_pre = y;
        
    end
end

%% calculation dF = İF / İu
function dF = dFdu( obj, x_current, u, system, nmpc )
    dF = zeros( nmpc.dimu * nmpc.N, nmpc.dimu * nmpc.N );
    diff = nmpc.hdir;
    
    for cnt = 1:nmpc.N*nmpc.dimu
        u_buff_p = u;
        u_buff_n = u;

        u_buff_p(cnt) = u_buff_p(cnt) + diff;
        u_buff_n(cnt) = u_buff_n(cnt) - diff;
        
        % dF = İF / İu is the Forward Difference Approximation
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