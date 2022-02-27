clear;
close all;
clc;

%% Setup and Parameters
dt   = 0.01; % sample time
nx   = 2;    % Number of states
nu   = 3;    % Number of input(u1=Attenuation coefficient, u2=Dummy input, u3=Lagrange multiplier)
nlmd = 2;    % Number of companion variable

sim_time = 10; % Simulation time [s]

sys.a = -1;     % システム変数
sys.b = -1;     % システム変数

system.m = 1; %[kg]
system.k = 1; %[N/m]

%% NMPC parameters
params_nmpc.tf = 1.0;             % Final value of prediction time [s]
params_nmpc.N = 5;                % Number of divisions of the prediction time [-]

params_nmpc.x0 = [2;0];           % Initial state
params_nmpc.u0 = [0.01;0.9;0.03]; % Initial u

params_nmpc.sf = [ 1;10 ];        % Termination cost weight matrix
params_nmpc.q = [ 1;10 ];         % Weight matrix of state quantities
params_nmpc.r = [ 1;0.01 ];       % Weight matrix of input quantities

params_nmpc.umin = -1;            % upper input limit
params_nmpc.umax = 1;             % lower input limit

params_nmpc.len_x   = nx;         % Number of states
params_nmpc.len_u   = nu;         % Number of input(u1=Attenuation coefficient, u2=Dummy input, u3=Lagrange multiplier)
params_nmpc.len_lmd = nlmd;       % Number of companion variable

%% Initial value calculation using Newton's method
lmd0 = dPhidx( params_nmpc.x0, params_nmpc );

for cnt = 1:20
    params_nmpc.u0 = params_nmpc.u0 - ddHddu(params_nmpc.u0, params_nmpc ) \ dHdu( params_nmpc.x0, params_nmpc.u0, lmd0, sys, params_nmpc );
end

%% main loop
xTrue(:, 1) = params_nmpc.x0;
uk(:, 1) = params_nmpc.u0;
uk_horizon(:, 1) = repmat( params_nmpc.u0, params_nmpc.N, 1 );
current_step = 1;
sim_length = length(1:dt:sim_time);
solvetime = zeros(1, sim_length + 1);

for i = 1:dt:sim_time
    i
    % update time
    current_step = current_step + 1;
    
    % solve nmpc
    tic;
    [uk(:, current_step), uk_horizon(:, current_step)] = NMPC( xTrue(:, current_step - 1), uk_horizon(:, current_step - 1), sys, params_nmpc);
    solvetime(1, current_step - 1) = toc;
    
    % update state
    u = uk(:, current_step);
    T = dt*current_step:dt:dt*current_step+dt;
    [T, x] = ode45(@(t,x) nonlinear_model(x, u(1), sys), T, xTrue(:, current_step - 1));
    xTrue(:, current_step) = x(end,:);
end

%% solve average time
avg_time = sum(solvetime) / current_step;
disp(avg_time);

drow_figure(xTrue, uk, current_step);

%% Hの入力微分
function Hu = dHdu( x, u, lmd, sys, nmpc )
    Hu = [ ...
        sys.b * lmd(2) * x(2) + 2 * u(3) * ( u(1) - ( nmpc.umin + nmpc.umax ) / 2 ) + nmpc.r(1) * u(1);
		2 * u(2) * u(3) - nmpc.r(2);
		( u(1) - ( nmpc.umin + nmpc.umax ) / 2 )^2 - ( nmpc.umax - nmpc.umin )^2 / 4 + u(2)^2;
    ];
end

%% Hの2階入力微分
function Huu = ddHddu(u, nmpc )
    Huu = [ ...
		2 * u(3) + nmpc.r(1), 0, 2 * ( u(1) - ( nmpc.umin + nmpc.umax ) / 2 );
		0, 2 * u(3), 2 * u(2);
		2 * ( u(1) - ( nmpc.umin + nmpc.umax ) / 2 ), 2 * u(2), 0;
	];
end

%% ヘッセ行列の状態微分
function Phix = dPhidx( x, cgmres )
    Phix = [ x(1) * cgmres.sf(1);x(2) * cgmres.sf(2) ];
end

%% NMPC
function [uk, uk_horizon] = NMPC( x_current, uk_horizon, sys, nmpc )
    for cnt = 1:10
        uk_horizon = uk_horizon - ( dFdu( x_current, uk_horizon, sys, nmpc ) \ F( x_current, uk_horizon, sys, nmpc ) );
    end
        
    uk = uk_horizon(1:nmpc.len_u);
end

%% C/GMRESでの函数Fの計算
function dF = dFdu( x_current, u, sys, nmpc )

    dF = zeros( nmpc.len_u * nmpc.N, nmpc.len_u * nmpc.N );
    diff = 0.01;
    
    for cnt = 1:nmpc.N*nmpc.len_u
        u_buff_p = u;
        u_buff_n = u;

        u_buff_p(cnt) = u_buff_p(cnt) + diff;
        u_buff_n(cnt) = u_buff_n(cnt) - diff;
        
        dF(:,cnt) = ( F( x_current, u_buff_p, sys, nmpc ) - F( x_current, u_buff_n, sys, nmpc ) ) / ( 2 * diff );
    end
end

%% C/GMRESでの函数Fの計算
function ans_F = F( x_current, u, sys, nmpc )
    x = Forward( x_current, u, sys, nmpc );
    lmd = Backward( x, u, sys, nmpc );

    ans_F = zeros( nmpc.len_u * nmpc.N, 1 );

    for cnt = 1:nmpc.N
        ans_F((1:nmpc.len_u)+nmpc.len_u*(cnt-1)) = dHdu(x((1:nmpc.len_x)+nmpc.len_x*(cnt-1)), ...
                                                    u((1:nmpc.len_u)+nmpc.len_u*(cnt-1)), ...
                                                    lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(cnt-1)), sys, nmpc );
    end
end

%% 現在時刻からT秒未来までの状態の予測（Euler近似）
function x = Forward( x0, u, sys, nmpc )
    dt = nmpc.tf / nmpc.N;
    
    x = zeros( nmpc.len_x * nmpc.N, 1 );
    x(1:nmpc.len_x) = x0;

    for cnt = 1 : nmpc.N-1
       x((1:nmpc.len_x)+nmpc.len_x*(cnt)) = x((1:nmpc.len_x)+nmpc.len_x*(cnt-1)) ...
                                                + Func( x((1:nmpc.len_x)+nmpc.len_x*(cnt-1)), u((1:nmpc.len_u)+nmpc.len_u*(cnt-1)), sys ) * dt; 
    end
end

%% 現在時刻からT秒未来までの随伴変数の予測（Euler近似）
function lmd = Backward( x, u, sys, nmpc )
    dt = nmpc.tf / nmpc.N;
    
    lmd = zeros( nmpc.len_lmd * nmpc.N, 1 );
    lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(nmpc.N-1)) = dPhidx( x((1:nmpc.len_x)+nmpc.len_x*(nmpc.N-1)), nmpc );
    
    for cnt = nmpc.N-1:-1:1
        lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(cnt-1)) = lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(cnt)) ...
                                                            + dHdx( x((1:nmpc.len_x)+nmpc.len_x*(cnt)), u, lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(cnt)), sys, nmpc ) * dt;
    end
end

%% 状態方程式
% dx = f( x, u )
function dx = Func( x, u, sys )
    dx = [ ...
        x(2); ...
        sys.a * x(1) + sys.b * x(2) * u(1); ...
        ]; 
end

function dx = nonlinear_model(xTrue, u, sys)
    % In this simulation, the body frame and its transformation is used
    % instead of a hybrid frame. That is because for the solver ode45, it
    % is important to have the nonlinear system of equations in the first
    % order form. 
    
    dx = [ ...
        xTrue(2); ...
        sys.a * xTrue(1) + sys.b * xTrue(2) * u; ...
        ]; 
end

%% Hの状態微分
function Hx = dHdx( x, u, lmd, sys, nmpc )
    Hx = [ ...
		nmpc.q(1) * x(1) + sys.a * lmd(2);
		nmpc.q(2) * x(2) + sys.b * lmd(2) * u(1) + lmd(1);
	];
end

function drow_figure(xTrue, uk, current_step)
    % plot state
    figure(1)
    subplot(2, 1, 1)
    plot(0:current_step - 1, xTrue(1,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('y [m]','interpreter','latex','FontSize',10);
    
    subplot(2, 1, 2)
    plot(0:current_step - 1, xTrue(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    xlabel('Time Step','interpreter','latex','FontSize',10);
    ylabel('$\dot{y}$ [m/s]','interpreter','latex','FontSize',10);
    
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
end