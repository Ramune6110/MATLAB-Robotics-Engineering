clear;
close all;
clc;

%% �V�X�e����`
sys.a = -1;     % �V�X�e���ϐ�
sys.b = -1;     % �V�X�e���ϐ�

%% �V�~�����[�V������`
SimPeriod = 10;                         % �V�~�����[�V�������� (s)
dSamplingPeriod = 0.01;                 % �T���v�����O���� (s)

%% C/GMRES�̃R���g���[����`
nmpc.tf = 1.0;                          % �\�����Ԃ̍ŏI�l (s)
nmpc.dv = 5;                            % �\�����Ԃ̕����� (-) �i�]�������ɂ���ĕ]������|�C���g�̐��j

nmpc.x0 = [2;0];                        % �R���g���[���ɗ^���鏉�����
nmpc.u0 = [0.01;0.9;0.03];              % �R���g���[���ɗ^���鏉�������

nmpc.sf = [ 1;10 ];                     % �\�����Ԃ̍ŏI��Ԃɑ΂���d�݁i�I�[�R�X�g�j
nmpc.q = [ 1;10 ];                      % ��Ԃɑ΂���d�݁i�X�e�[�W�R�X�g�j
nmpc.r = [ 1;0.01 ];                    % ����ʂɑ΂���d��

nmpc.umin = -1;                         % ���͏���i�����̓[���ɐݒ肵�Ă���j
nmpc.umax = 1;                          % ���͏���i�����̓[���ɐݒ肵�Ă���j

%% C/GMRES�̃R���g���[���p�v�Z

% �������͒l�̌v�Z�iNewton�@�j
lmd0 = dPhidx( nmpc.x0, nmpc );
u0 = [1;2;3;]; % Newton�@�̏����l

for cnt = 1:20
    nmpc.u0 = nmpc.u0 - ddHddu( nmpc.x0, nmpc.u0, lmd0, sys, nmpc ) \ dHdu( nmpc.x0, nmpc.u0, lmd0, sys, nmpc );
end

nmpc.len_x = length( nmpc.x0 );     % ��Ԃ̐�
nmpc.len_u = length( nmpc.u0 );     % ����ʂ̐�
nmpc.len_lmd = nmpc.len_x;          % �����ϐ��̐�

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

%% H�̓��͔���
function Hu = dHdu( x, u, lmd, sys, nmpc )
    Hu = [ ...
        sys.b * lmd(2) * x(2) + 2 * u(3) * ( u(1) - ( nmpc.umin + nmpc.umax ) / 2 ) + nmpc.r(1) * u(1);
		2 * u(2) * u(3) - nmpc.r(2);
		( u(1) - ( nmpc.umin + nmpc.umax ) / 2 )^2 - ( nmpc.umax - nmpc.umin )^2 / 4 + u(2)^2;
    ];
end

%% H��2�K���͔���
function Huu = ddHddu( x, u, lmd, sys, nmpc )
    Huu = [ ...
		2 * u(3) + nmpc.r(1), 0, 2 * ( u(1) - ( nmpc.umin + nmpc.umax ) / 2 );
		0, 2 * u(3), 2 * u(2);
		2 * ( u(1) - ( nmpc.umin + nmpc.umax ) / 2 ), 2 * u(2), 0;
	];
end

%% �w�b�Z�s��̏�Ԕ���
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

%% C/GMRES�ł̔���F�̌v�Z
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

%% C/GMRES�ł̔���F�̌v�Z
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

%% ���ݎ�������T�b�����܂ł̏�Ԃ̗\���iEuler�ߎ��j
function x = Forward( x0, u, sys, nmpc )
    dt = nmpc.tf / nmpc.dv;
    
    x = zeros( nmpc.len_x * nmpc.dv, 1 );
    x(1:nmpc.len_x) = x0;

    for cnt = 1 : nmpc.dv-1
       x((1:nmpc.len_x)+nmpc.len_x*(cnt)) = x((1:nmpc.len_x)+nmpc.len_x*(cnt-1)) ...
                                                + Func( x((1:nmpc.len_x)+nmpc.len_x*(cnt-1)), u((1:nmpc.len_u)+nmpc.len_u*(cnt-1)), sys ) * dt; 
    end
end

%% ���ݎ�������T�b�����܂ł̐����ϐ��̗\���iEuler�ߎ��j
function lmd = Backward( x, u, sys, nmpc )
    dt = nmpc.tf / nmpc.dv;
    
    lmd = zeros( nmpc.len_lmd * nmpc.dv, 1 );
    lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(nmpc.dv-1)) = dPhidx( x((1:nmpc.len_x)+nmpc.len_x*(nmpc.dv-1)), nmpc );
    
    for cnt = nmpc.dv-1:-1:1
        lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(cnt-1)) = lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(cnt)) ...
                                                            + dHdx( x((1:nmpc.len_x)+nmpc.len_x*(cnt)), u, lmd((1:nmpc.len_lmd)+nmpc.len_lmd*(cnt)), sys, nmpc ) * dt;
    end
end

%% ��ԕ�����
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

%% H�̏�Ԕ���
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