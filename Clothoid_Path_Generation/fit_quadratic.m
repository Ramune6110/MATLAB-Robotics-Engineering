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
    
    % グラフ上に描画する点の数
    num_points = 100;

    % 描画範囲の設定
    x_range = linspace(min(points(:, 1)), max(points(:, 1)), num_points);

    % 2次多項式関数の計算
    y_range = a * x_range.^2 + b * x_range + c;
    
    % グラフの描画
    figure;
    plot(x_range, y_range, 'LineWidth', 2); % 2次多項式関数を描画
    hold on;
    scatter(points(:, 1), points(:, 2), 'r', 'filled'); % 与えられた4点を赤で描画
    xlabel('x');
    ylabel('y');
    title('2次多項式関数と与えられた4つの点の描画');
    legend('2次多項式関数', '与えられた4点');
    grid on;
    hold off;
    
    % 2次多項式関数の微分
    syms x_sym;
    y_sym = a * x_sym^2 + b * x_sym + c;
    dy_sym = diff(y_sym, x_sym);
    d2y_sym = diff(dy_sym, x_sym);

    % 曲率 (κ) の計算
    kappa_sym = abs(d2y_sym) / (1 + dy_sym^2)^(3/2);
    kappa = matlabFunction(kappa_sym, 'Vars', x_sym);

    % 曲率変化率 (Δκ) の計算
    dkappa_sym = diff(kappa_sym, x_sym);
    delta_kappa = matlabFunction(dkappa_sym, 'Vars', x_sym);

    % 曲率と曲率変化率を計算するx座標の範囲
    x_range = linspace(min(points(:, 1)), max(points(:, 1)), num_points);

    % 曲率と曲率変化率を計算
    kappa_values = kappa(x_range);
    delta_kappa_values = delta_kappa(x_range);

    % 曲率と曲率変化率を描画
    figure;
    subplot(2, 1, 1);
    plot(x_range, kappa_values, 'LineWidth', 2);
    xlabel('x');
    ylabel('曲率 κ');
    title('曲率');
    grid on;

    subplot(2, 1, 2);
    plot(x_range, delta_kappa_values, 'LineWidth', 2);
    xlabel('x');
    ylabel('曲率変化率 Δκ');
    title('曲率変化率');
    grid on;

    % 2次多項式関数に最もフィッティングする曲率と曲率変化率を見つける
[~, max_kappa_idx] = max(abs(kappa_values));
[~, max_delta_kappa_idx] = max(abs(delta_kappa_values));

kappa_fit = kappa_values(max_kappa_idx)
delta_kappa_fit = delta_kappa_values(max_delta_kappa_idx)

% % クロソイド曲線パラメータを計算
% L = 10; % クロソイド曲線の長さ（任意の値）
% A_clothoid = sqrt(2 * L * kappa_fit / delta_kappa_fit);
% 
% % クロソイド曲線の計算
% t_clothoid = linspace(0, L, num_points);
% x_clothoid = A_clothoid * sqrt(2 * t_clothoid) .* cos(t_clothoid);
% y_clothoid = A_clothoid * sqrt(2 * t_clothoid) .* sin(t_clothoid);

% クロソイド曲線の描画
% figure;
% plot(x_clothoid, y_clothoid, 'LineWidth', 2);
% xlabel('x');
% ylabel('y');
% title('2次多項式関数にフィッティングするクロソイド曲線');
% grid on;

end

function [a, b, c] = calc_fit_quadratic(points)

% 点の数を確認
assert(size(points, 1) == 4, '4つの点が必要です');

% 目的関数の定義
obj_fun = @(p) sum((points(:, 2) - (p(1) * points(:, 1).^2 + p(2) * points(:, 1) + p(3))).^2);

% 初期パラメータ
init_params = [0, 0, 0];

% 最適化
options = optimoptions('fminunc', 'Display', 'none', 'Algorithm', 'quasi-newton');
params = fminunc(obj_fun, init_params, options);

% パラメータを取得
a = params(1);
b = params(2);
c = params(3);

end
