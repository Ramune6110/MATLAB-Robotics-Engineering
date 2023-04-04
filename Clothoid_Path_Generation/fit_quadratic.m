function [] = fit_quadratic()
    clear;
    close all;
    clc;
    
%     points = [1, 3;
%               2, 5;
%               3, 10;
%               4, 18];
          
    points = [0, 0;
              5, 3;
              10, 8;
              15, 15];
          
    [a, b, c] = calc_fit_quadratic(points);
    
    % �O���t��ɕ`�悷��_�̐�
    num_points = 100;

    % �`��͈͂̐ݒ�
    x_range = linspace(min(points(:, 1)), max(points(:, 1)), num_points);

    % 2���������֐��̌v�Z
    y_range = a * x_range.^2 + b * x_range + c;
    
    % �O���t�̕`��
    figure;
    plot(x_range, y_range, 'LineWidth', 2); % 2���������֐���`��
    hold on;
    scatter(points(:, 1), points(:, 2), 'r', 'filled'); % �^����ꂽ4�_��Ԃŕ`��
    xlabel('x');
    ylabel('y');
    title('2���������֐��Ɨ^����ꂽ4�̓_�̕`��');
    legend('2���������֐�', '�^����ꂽ4�_');
    grid on;
    hold off;
    
    % 2���������֐��̔���
    syms x_sym;
    y_sym = a * x_sym^2 + b * x_sym + c;
    dy_sym = diff(y_sym, x_sym);
    d2y_sym = diff(dy_sym, x_sym);

    % �ȗ� (��) �̌v�Z
    kappa_sym = abs(d2y_sym) / (1 + dy_sym^2)^(3/2);
    kappa = matlabFunction(kappa_sym, 'Vars', x_sym);

    % �ȗ��ω��� (����) �̌v�Z
    dkappa_sym = diff(kappa_sym, x_sym);
    delta_kappa = matlabFunction(dkappa_sym, 'Vars', x_sym);

    % �ȗ��Ƌȗ��ω������v�Z����x���W�͈̔�
    x_range = linspace(min(points(:, 1)), max(points(:, 1)), num_points);

    % �ȗ��Ƌȗ��ω������v�Z
    kappa_values = kappa(x_range);
    delta_kappa_values = delta_kappa(x_range);

    % �ȗ��Ƌȗ��ω�����`��
    figure;
    subplot(2, 1, 1);
    plot(x_range, kappa_values, 'LineWidth', 2);
    xlabel('x');
    ylabel('�ȗ� ��');
    title('�ȗ�');
    grid on;

    subplot(2, 1, 2);
    plot(x_range, delta_kappa_values, 'LineWidth', 2);
    xlabel('x');
    ylabel('�ȗ��ω��� ����');
    title('�ȗ��ω���');
    grid on;

    % 2���������֐��ɍł��t�B�b�e�B���O����ȗ��Ƌȗ��ω�����������
[~, max_kappa_idx] = max(abs(kappa_values));
[~, max_delta_kappa_idx] = max(abs(delta_kappa_values));

kappa_fit = kappa_values(max_kappa_idx)
delta_kappa_fit = delta_kappa_values(max_delta_kappa_idx)

% % �N���\�C�h�Ȑ��p�����[�^���v�Z
% L = 10; % �N���\�C�h�Ȑ��̒����i�C�ӂ̒l�j
% A_clothoid = sqrt(2 * L * kappa_fit / delta_kappa_fit);
% 
% % �N���\�C�h�Ȑ��̌v�Z
% t_clothoid = linspace(0, L, num_points);
% x_clothoid = A_clothoid * sqrt(2 * t_clothoid) .* cos(t_clothoid);
% y_clothoid = A_clothoid * sqrt(2 * t_clothoid) .* sin(t_clothoid);

% �N���\�C�h�Ȑ��̕`��
% figure;
% plot(x_clothoid, y_clothoid, 'LineWidth', 2);
% xlabel('x');
% ylabel('y');
% title('2���������֐��Ƀt�B�b�e�B���O����N���\�C�h�Ȑ�');
% grid on;

end

function [a, b, c] = calc_fit_quadratic(points)

% �_�̐����m�F
assert(size(points, 1) == 4, '4�̓_���K�v�ł�');

% �ړI�֐��̒�`
obj_fun = @(p) sum((points(:, 2) - (p(1) * points(:, 1).^2 + p(2) * points(:, 1) + p(3))).^2);

% �����p�����[�^
init_params = [0, 0, 0];

% �œK��
options = optimoptions('fminunc', 'Display', 'none', 'Algorithm', 'quasi-newton');
params = fminunc(obj_fun, init_params, options);

% �p�����[�^���擾
a = params(1);
b = params(2);
c = params(3);

end
