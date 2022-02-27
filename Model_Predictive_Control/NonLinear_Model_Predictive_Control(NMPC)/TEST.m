clear;
close all;
clc;

%% システム定義
sys.a = -1;     % システム変数
sys.b = -1;     % システム変数

%% シミュレーション定義
SimPeriod = 10;                         % シミュレーション時間 (s)
dSamplingPeriod = 0.01;                 % サンプリング周期 (s)

%% C/GMRESのコントローラ定義
nmpc.tf = 1.0;                          % 予測時間の最終値 (s)
nmpc.dv = 5;                            % 予測時間の分割数 (-) （評価函数によって評価するポイントの数）

nmpc.x0 = [2;0];                        % コントローラに与える初期状態
nmpc.u0 = [0.01;0.9;0.03];              % コントローラに与える初期操作量

nmpc.sf = [ 1;10 ];                     % 予測時間の最終状態に対する重み（終端コスト）
nmpc.q = [ 1;10 ];                      % 状態に対する重み（ステージコスト）
nmpc.r = [ 1;0.01 ];                    % 操作量に対する重み

nmpc.umin = -1;                         % 入力上限（下限はゼロに設定している）
nmpc.umax = 1;                          % 入力上限（下限はゼロに設定している）

%% C/GMRESのコントローラ用計算

% 初期入力値の計算（Newton法）
lmd0 = dPhidx( nmpc.x0, nmpc );
u0 = [1;2;3;]; % Newton法の初期値

for cnt = 1:20
    nmpc.u0 = nmpc.u0 - ddHddu( nmpc.x0, nmpc.u0, lmd0, sys, nmpc ) \ dHdu( nmpc.x0, nmpc.u0, lmd0, sys, nmpc );
end

nmpc.len_x = length( nmpc.x0 );     % 状態の数
nmpc.len_u = length( nmpc.u0 );     % 操作量の数
nmpc.len_lmd = nmpc.len_x;          % 随伴変数の数

xTrue(:, 1) = [2;0];
uk(:, 1) = nmpc.u0;
uk_horizon(:, 1) = repmat( nmpc.u0, nmpc.dv, 1 );
current_step = 1;
sim_length = length(1:dSamplingPeriod:SimPeriod);
solvetime = zeros(1, sim_length + 1);

%% main loop
for i = 1:dSamplingPeriod:SimPeriod
    i
    % update time
    current_step = current_step + 1;
    
    % solve nmpc
    tic;
    [uk(:, current_step), uk_horizon(:, current_step)] = NMPC( xTrue(:, current_step - 1), uk_horizon(:, current_step - 1), sys, nmpc);
    solvetime(1, current_step - 1) = toc;
    
    % update state
    u = uk(:, current_step);
    T = dSamplingPeriod*current_step:dSamplingPeriod:dSamplingPeriod*current_step+dSamplingPeriod;
    [T, x] = ode45(@(t,x) nonlinear_model(t, x, u(1), sys), T, xTrue(:, current_step - 1));
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
function Huu = ddHddu( x, u, lmd, sys, nmpc )
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
        uk_horizon = uk_horizon - ( dFdu( x_current, uk_horizon, sys, nmpc ) \ CalcF( x_current, uk_horizon, sys, nmpc ) );
    end
        
    uk = uk_horizon(1:nmpc.len_u);
end

%% C/GMRESでの函数Fの計算
function dF = dFdu( x_current, u, sys, nmpc )

    dF = zeros( nmpc.len_u * nmpc.dv, nmpc.len_u * nmpc.dv );
    diff = 0.01;
    
    for cnt = 1:nmpc.dv*nmpc.len_u
        u_buff_p = u;
        u_buff_n = u;

        u_buff_p(cnt) = u_buff_p(cnt) + diff;
        u_buff_n(cnt) = u_buff_n(cnt) - diff;
        
        dF(:,cnt) = ( CalcF( x_current, u_buff_p, sys, nmpc ) - CalcF( x_current, u_buff_n, sys, nmpc ) ) / ( 2 * diff );
    end
end

%% C/GMRESでの函数Fの計算
function F = CalcF( x_current, u, sys, nmpc )
    x = Forward( x_current, u, sys, nmpc );
    lmd = Backward( x, u, sys, nmpc );

    F = zeros( nmpc.len_u * nmpc.dv, 1 );

    for cnt = 1:nmpc.dv
        F((1:nmpc.len_u)+nmpc.len_u*(cnt-1)) = dHdu(x((1:nmpc.len_x)+nmpc.len_x*(cnt-1)), ...
                                                    u((1:nmpc.len_u)+nmpc.len_u*(cnt-1)), ...
                                                    lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(cnt-1)), sys, nmpc );
    end
end

%% 現在時刻からT秒未来までの状態の予測（Euler近似）
function x = Forward( x0, u, sys, nmpc )
    dt = nmpc.tf / nmpc.dv;
    
    x = zeros( nmpc.len_x * nmpc.dv, 1 );
    x(1:nmpc.len_x) = x0;

    for cnt = 1 : nmpc.dv-1
       x((1:nmpc.len_x)+nmpc.len_x*(cnt)) = x((1:nmpc.len_x)+nmpc.len_x*(cnt-1)) ...
                                                + Func( x((1:nmpc.len_x)+nmpc.len_x*(cnt-1)), u((1:nmpc.len_u)+nmpc.len_u*(cnt-1)), sys ) * dt; 
    end
end

%% 現在時刻からT秒未来までの随伴変数の予測（Euler近似）
function lmd = Backward( x, u, sys, nmpc )
    dt = nmpc.tf / nmpc.dv;
    
    lmd = zeros( nmpc.len_lmd * nmpc.dv, 1 );
    lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(nmpc.dv-1)) = dPhidx( x((1:nmpc.len_x)+nmpc.len_x*(nmpc.dv-1)), nmpc );
    
    for cnt = nmpc.dv-1:-1:1
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

function dx = nonlinear_model(t, xTrue, u, sys)
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