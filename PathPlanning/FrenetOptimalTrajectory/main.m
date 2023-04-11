% frenet_optimal_planning.m
clear;
close all;
clc;

%% Parameters
param.MAX_SPEED = 50.0 / 3.6;  % maximum speed [m/s]
param.MAX_ACCEL = 2.0;  % maximum acceleration [m/ss]
param.MAX_CURVATURE = 1.0;  % maximum curvature [1/m]
param.MAX_ROAD_WIDTH = 7.0;  % maximum road width [m]
param.D_ROAD_W = 1.0;  % road width sampling length [m]
param.DT = 0.2;  % time tick [s]
param.MAX_T = 5.0;  % max prediction time [m]
param.MIN_T = 4.0;  % min prediction time [m]
param.TARGET_SPEED = 30.0 / 3.6;  % target speed [m/s]
param.D_T_S = 5.0 / 3.6;  % target speed sampling length [m/s]
param.N_S_SAMPLE = 1;  % sampling number of target speed
param.ROBOT_RADIUS = 1.0;
param.OBSTACLE_RADIUS = 1.0;
param.PLANNING_RADIUS = param.ROBOT_RADIUS + param.OBSTACLE_RADIUS;  % robot radius [m]

% cost weights
param.K_J = 0.1;
param.K_T = 0.1;
param.K_D = 1.0;
param.K_LAT = 1.0;
param.K_LON = 1.0;

%% main loop
disp(strcat(mfilename, " start!!"));

% way points
wx = [0.0, 10.0, 20.5, 35.0, 70.5];
wy = [0.0, -6.0, 5.0, 6.5, 0.0];

% obstacle lists
ob = [20.0, 10.0;
      30.0, 6.0;
      30.0, 8.0;
      35.0, 8.0;
      50.0, 3.0];

[tx, ty, tyaw, tc, csp] = generate_target_course(wx, wy);

% initial state
c_speed = 10.0 / 3.6;  % current speed [m/s]
c_accel = 0.0;  % current acceleration [m/ss]
c_d = 2.0;  % current lateral position [m]
c_d_d = 0.0;  % current lateral speed [m/s]
c_d_dd = 0.0;  % current lateral acceleration [m/s]
s0 = 0.0;  % current course position

area = 20.0;  % animation area length [m]

SIM_LOOP = 500;

for i = 1:SIM_LOOP
    [path, fplist] = frenet_optimal_planning(param, csp, s0, c_speed, c_accel, ...
                                   c_d, c_d_d, c_d_dd, ob);

    s0 = path.s(2);
    c_d = path.d(2);
    c_d_d = path.d_d(2);
    c_d_dd = path.d_dd(2);
    c_speed = path.s_d(2);
    c_accel = path.s_dd(2);

    if hypot(path.x(2) - tx(end), path.y(2) - ty(end)) <= 1.0
        disp("Goal");
        break;
    end

    % show_animation
    hold off;
    fig = figure(1);
    fig.Position = [0 0 1368 700];
    plot(tx, ty, '-b'); hold on;
    
    th = linspace(0,2*pi*100);
    circle_x = cos(th) ; 
    circle_y = sin(th) ;
    plot(path.x(2) + param.ROBOT_RADIUS * circle_x, ...
         path.y(2) + param.ROBOT_RADIUS * circle_y, 'm'); hold on;
    
    obs_radius = param.PLANNING_RADIUS / 2;
    for j = 1:length(ob(:, 1))
        plot(ob(j, 1) + param.OBSTACLE_RADIUS * circle_x, ...
             ob(j, 2) + param.OBSTACLE_RADIUS * circle_y, 'k'); hold on;
    end
    
    for k = 1:length(fplist)
        fp = fplist(k);
        plot(fp.x(2:end), fp.y(2:end), 'g'); hold on;
    end
    
    plot(path.x(2:end), path.y(2:end), 'r', 'LineWidth', 5.0); hold on;
    xlim([-10, 80]);
    ylim([-20, 20]);
    title(strcat("v[km/h]:", num2str(c_speed * 3.6, '%.2f')));
    axis equal;
    grid on;
end

disp("Finish");

% MATLAB version of the cubic_spline_planner should be implemented
% and used as generate_target_course function.
function [rx, ry, ryaw, rk, csp] = generate_target_course(x, y)
    csp = CubicSpline2D(x, y);
    s = 0 : 0.1 : csp.s(end);

    rx = [];
    ry = [];
    ryaw = [];
    rk = [];
    for i = 1:length(s)
        [ix, iy] = csp.calc_position(s(i));
        rx = [rx, ix];
        ry = [ry, iy];
        ryaw = [ryaw, csp.calc_yaw(s(i))];
        rk = [rk, csp.calc_curvature(s(i))];
    end
end

function frenet_paths = calc_frenet_paths(param, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
    frenet_paths = [];

    % generate path to each offset goal
    for di = -param.MAX_ROAD_WIDTH : param.D_ROAD_W : param.MAX_ROAD_WIDTH

        % Lateral motion planning
        for Ti = param.MIN_T : param.DT : param.MAX_T
            fp = FrenetPath();

            % lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);

            fp.t = 0 : param.DT : Ti;
            fp.d = lat_qp.calc_point(fp.t);
            fp.d_d = lat_qp.calc_first_derivative(fp.t);
            fp.d_dd = lat_qp.calc_second_derivative(fp.t);
            fp.d_ddd = lat_qp.calc_third_derivative(fp.t);

            % Longitudinal motion planning (Velocity keeping)
            for tv = param.TARGET_SPEED - param.D_T_S * param.N_S_SAMPLE : param.D_T_S : param.TARGET_SPEED + param.D_T_S * param.N_S_SAMPLE
                %tfp = copy(fp);
                tfp = fp;
                lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti);

                tfp.s = lon_qp.calc_point(fp.t);
                tfp.s_d = lon_qp.calc_first_derivative(fp.t);
                tfp.s_dd = lon_qp.calc_second_derivative(fp.t);
                tfp.s_ddd = lon_qp.calc_third_derivative(fp.t);

                Jp = sum(tfp.d_ddd.^2);  % square of jerk
                Js = sum(tfp.s_ddd.^2); 
                
                % Square of diff from target speed
                ds = (param.TARGET_SPEED - tfp.s_d(end))^2;

                tfp.cd = param.K_J * Jp + param.K_T * Ti + param.K_D * tfp.d(end)^2;
                tfp.cv = param.K_J * Js + param.K_T * Ti + param.K_D * ds;
                tfp.cf = param.K_LAT * tfp.cd + param.K_LON * tfp.cv;

                frenet_paths = [frenet_paths, tfp];
            end
        end
    end
end

function fplist = calc_global_paths(fplist, csp)
    for fp_idx = 1:numel(fplist)
        fp = fplist(fp_idx);

        % calc global positions
        for i = 1:length(fp.s)
            [ix, iy] = calc_position(csp, fp.s(i));
            if isempty(ix)
                break;
            end
            i_yaw = calc_yaw(csp, fp.s(i));
            di = fp.d(i);
            fx = ix + di * cos(i_yaw + pi / 2.0);
            fy = iy + di * sin(i_yaw + pi / 2.0);
            fp.x = [fp.x, fx];
            fp.y = [fp.y, fy];
        end

        % calc yaw and ds
        for i = 1:length(fp.x) - 1
            dx = fp.x(i + 1) - fp.x(i);
            dy = fp.y(i + 1) - fp.y(i);
            fp.yaw = [fp.yaw, atan2(dy, dx)];
            fp.ds = [fp.ds, hypot(dx, dy)];
        end

        fp.yaw = [fp.yaw, fp.yaw(end)];
        fp.ds = [fp.ds, fp.ds(end)];

        % calc curvature
        for i = 1:length(fp.yaw) - 1
            fp.c = [fp.c, (fp.yaw(i + 1) - fp.yaw(i)) / fp.ds(i)];
        end

        fplist(fp_idx) = fp;
    end
end

function valid_paths = check_paths(fplist, ob, param)
    ok_ind = [];
    MAX_SPEED = param.MAX_SPEED;
    MAX_ACCEL = param.MAX_ACCEL;
    MAX_CURVATURE = param.MAX_CURVATURE;

    for i = 1:numel(fplist)
        fp = fplist(i);
        % Max speed check
        if any(fp.s_d > MAX_SPEED)
            continue;
        end

        % Max accel check
        if any(abs(fp.s_dd) > MAX_ACCEL)
            continue;
        end

        % Max curvature check
        if any(abs(fp.c) > MAX_CURVATURE)
            continue;
        end

        % Collision check
        if ~check_collision(fp, ob, param)
            continue;
        end

        ok_ind = [ok_ind, i];
    end

    valid_paths = fplist(ok_ind);
end

function is_collision_free = check_collision(fp, ob, param)
    is_collision_free = true;
    PLANNING_RADIUS = param.PLANNING_RADIUS;

    for i = 1:size(ob, 1)
        d = arrayfun(@(ix, iy) (ix - ob(i, 1))^2 + (iy - ob(i, 2))^2, fp.x, fp.y);

        collision = any(d <= PLANNING_RADIUS^2);

        if collision
            is_collision_free = false;
            return;
        end
    end
end

function [best_path, fplist] = frenet_optimal_planning(param, csp, s0, c_speed, c_accel, ...
                                               c_d, c_d_d, c_d_dd, ob)
    fplist = calc_frenet_paths(param, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0);
    fplist = calc_global_paths(fplist, csp);
    fplist = check_paths(fplist, ob, param);

    % find minimum cost path
    min_cost = Inf;
    best_path = [];
    for i = 1:length(fplist)
        fp = fplist(i);
        if min_cost >= fp.cf
            min_cost = fp.cf;
            best_path = fp;
        end
    end
end
