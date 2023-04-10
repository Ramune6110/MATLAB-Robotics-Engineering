function [] = CurvatureCalcuration()
    clear;
    close all;
    clc;

    cx = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0];
    cy = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0];
    [x, y, yaw, k, travel] = calc_spline_course(cx, cy, 200);

    fig = figure(1);
    fig.Position = [0 0 1368 700];
    subplot(121);
    plot(cx, cy, 'xb', 'DisplayName', 'input');
    hold on;
    plot(x, y, '-r', 'DisplayName', 'spline');
    grid on;
    axis equal;
    xlabel("x[m]");
    ylabel("y[m]");
    legend;

    % Calculate curvature with various methods
    curvature_circle_fitting = calc_curvature_circle_fitting(x, y);
    curvature_yaw_diff = calc_curvature_with_yaw_diff(x, y, yaw);
    curvature_range_kutta = calc_curvature_range_kutta(x, y);
    curvature_2_derivative = calc_curvature_2_derivative(x, y);

    subplot(122);
    plot(travel, k, '-r', 'DisplayName', 'analytic curvature');
    hold on;
    plot(travel, curvature_circle_fitting, '-b', 'DisplayName', 'circle_fitting');
    plot(travel, curvature_yaw_diff, '-g', 'DisplayName', 'yaw_angle_diff');
    plot(travel, curvature_range_kutta, '-c', 'DisplayName', 'range_kutta');
    plot(travel(1:end-1), curvature_2_derivative, '-k', 'DisplayName', '2_derivative');
    grid on;
    legend;
    xlabel("line length[m]");
    ylabel("curvature [1/m]");
end

function [r_x, r_y, r_yaw, r_k, travel] = calc_spline_course(x, y, num)
    % Calc 2D spline course with interpolation
    % Input:
    %  - x: interpolated x positions
    %  - y: interpolated y positions
    %  - num: number of path points
    % Output:
    %  - r_x: x positions
    %  - r_y: y positions
    %  - r_yaw: yaw angle list
    %  - r_k: curvature list
    %  - travel: path length from start point

    sp = CubicSpline2D(x, y);
    s = linspace(0, sp.s(end), num+1);
    s(end) = [];

    r_x = zeros(1, length(s));
    r_y = zeros(1, length(s));
    r_yaw = zeros(1, length(s));
    r_k = zeros(1, length(s));

    for i = 1:length(s)
        i_s = s(i);
        [ix, iy] = sp.calc_position(i_s);
        r_x(i) = ix;
        r_y(i) = iy;
        r_yaw(i) = sp.calc_yaw(i_s);
        r_k(i) = sp.calc_curvature(i_s);
    end

    travel = cumsum(sqrt(diff(r_x).^2 + diff(r_y).^2));
    travel = [0, travel];
end

function [ cx, cy, r ] = circleFitting(x,y)
%CIRCLEFITTING 最小二乗法による円フィッテングをする関数
% input: x,y 円フィッティングする点群
% output cx 中心x座標
%        cy 中心y座標
%        r  半径
% 参考
% 一般式による最小二乗法（円の最小二乗法）　画像処理ソリューション
% http://imagingsolution.blog107.fc2.com/blog-entry-16.html

sumx=sum(x);
sumy=sum(y);
sumx2=sum(x.^2);
sumy2=sum(y.^2);
sumxy=sum(x.*y);

F=[sumx2 sumxy sumx;
   sumxy sumy2 sumy;
   sumx  sumy  length(x)];

G=[-sum(x.^3+x.*y.^2);
   -sum(x.^2.*y+y.^3);
   -sum(x.^2+y.^2)];

T=F\G;

cx=T(1)/-2;
cy=T(2)/-2;
r=sqrt(cx^2+cy^2-T(3));

end

function cv = calc_curvature_circle_fitting(x, y)
    % Calc curvature
    % Input:
    %  - x: x position list
    %  - y: y position list
    %  - npo: the number of points using Calculation curvature

    n_data = length(x);
    cv = zeros(1, n_data);
    npo = 1;
    
    for i = 1:n_data
        lind = i - npo;
        hind = i + npo + 1;

        if lind < 1
            lind = 1;
        end
        if hind > n_data
            hind = n_data;
        end

        xs = x(lind:hind);
        ys = y(lind:hind);
        [cxe, cye, re] = circleFitting(xs, ys);

        if length(xs) >= 3
            % sign evaluation
            c_index = ceil((length(xs) - 1) / 2);
            sign = (xs(1) - xs(c_index)) * (ys(end) - ys(c_index)) - ...
                   (ys(1) - ys(c_index)) * (xs(end) - xs(c_index));

            % check straight line
            a = [xs(1) - xs(c_index), ys(1) - ys(c_index)];
            b = [xs(end) - xs(c_index), ys(end) - ys(c_index)];
            theta = rad2deg(acos(dot(a, b) / (norm(a) * norm(b))));

            if theta == 180.0
                cv(i) = 0.0;  % straight line
            elseif sign > 0
                cv(i) = 1.0 / -re;
            else
                cv(i) = 1.0 / re;
            end
        else
            cv(i) = 0.0;
        end
    end
end

function curvatures = calc_curvature_with_yaw_diff(x, y, yaw)
    dists = sqrt(diff(x).^2 + diff(y).^2);
    d_yaw = diff(make_angles_continuous(yaw));
    curvatures = d_yaw ./ dists;
    curvatures(end + 1) = 0.0;
end

function angles = make_angles_continuous(angles)
    angles = wrapToPi(angles);
    for i = 1:length(angles) - 1
        d_angle = angles(i + 1) - angles(i);
        if d_angle >= pi
            angles(i + 1:end) = angles(i + 1:end) - 2.0 * pi;
        elseif d_angle <= -pi
            angles(i + 1:end) = angles(i + 1:end) + 2.0 * pi;
        end
    end
end

function curvatures = calc_curvature_range_kutta(x, y)
    dists = sqrt(diff(x).^2 + diff(y).^2);
    curvatures = [0.0, 0.0];
    for i = 3:length(x) - 1
        dx = (x(i + 1) - x(i)) / dists(i);
        dy = (y(i + 1) - y(i)) / dists(i);
        ddx = (x(i - 2) - x(i - 1) - x(i) + x(i + 1)) / (2 * dists(i)^2);
        ddy = (y(i - 2) - y(i - 1) - y(i) + y(i + 1)) / (2 * dists(i)^2);
        curvature = (ddy * dx - ddx * dy) / ((dx^2 + dy^2)^1.5);
        curvatures(end + 1) = curvature;
    end
    curvatures(end + 1) = 0.0;
end

function curvatures = calc_curvature_2_derivative(x, y)
    curvatures = [0.0];
    for i = 2:length(x) - 1
        dxn = x(i) - x(i - 1);
        dxp = x(i + 1) - x(i);
        dyn = y(i) - y(i - 1);
        dyp = y(i + 1) - y(i);
        dn = hypot(dxn, dyn);
        dp = hypot(dxp, dyp);
        dx = 1.0 / (dn + dp) * (dp / dn * dxn + dn / dp * dxp);
        ddx = 2.0 / (dn + dp) * (dxp / dp - dxn / dn);
        dy = 1.0 / (dn + dp) * (dp / dn * dyn + dn / dp * dyp);
        ddy = 2.0 / (dn + dp) * (dyp / dp - dyn / dn);
        curvature = (ddy * dx - ddx * dy) / ((dx^2 + dy^2)^1.5);
        curvatures(end + 1) = curvature;
    end
end
