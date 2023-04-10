function QuinticPolynomialExample()
    clear;
    close all;
    clc;
    
    disp("main start!");

    sx = 10.0;  % start x position [m]
    sy = 10.0;  % start y position [m]
    syaw = deg2rad(10.0);  % start yaw angle [rad]
    sv = 1.0;  % start speed [m/s]
    sa = 0.1;  % start accel [m/ss]
    gx = 30.0;  % goal x position [m]
    gy = -10.0;  % goal y position [m]
    gyaw = deg2rad(20.0);  % goal yaw angle [rad]
    gv = 1.0;  % goal speed [m/s]
    ga = 0.1;  % goal accel [m/ss]
    max_accel = 1.0;  % max accel [m/ss]
    max_jerk = 0.5;  % max jerk [m/sss]
    dt = 0.1;  % time tick [s]

    [time, x, y, yaw, v, a, j] = quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt);

    % Plot x, y
    figure;
    plot(x, y, "-r");
    grid on;
    
    % Plot yaw
    figure;
    plot(time, rad2deg(yaw), "-r");
    xlabel("Time[s]");
    ylabel("Yaw[deg]");
    grid on;

    % Plot speed
    figure;
    plot(time, v, "-r");
    xlabel("Time[s]");
    ylabel("Speed[m/s]");
    grid on;

    % Plot acceleration
    figure;
    plot(time, a, "-r");
    xlabel("Time[s]");
    ylabel("accel[m/ss]");
    grid on;

    % Plot jerk
    figure;
    plot(time, j, "-r");
    xlabel("Time[s]");
    ylabel("jerk[m/sss]");
    grid on;
end

function [time, rx, ry, ryaw, rv, ra, rj] = quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt)
    MIN_T = 0.1;
    MAX_T = 100.0;

    vxs = sv * cos(syaw);
    vys = sv * sin(syaw);
    vxg = gv * cos(gyaw);
    vyg = gv * sin(gyaw);

    axs = sa * cos(syaw);
    ays = sa * sin(syaw);
    axg = ga * cos(gyaw);
    ayg = ga * sin(gyaw);

    time = [];
    rx = [];
    ry = [];
    ryaw = [];
    rv = [];
    ra = [];
    rj = [];

    for T = MIN_T:MIN_T:MAX_T
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T);
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T);

        time_temp = [];
        rx_temp = [];
        ry_temp = [];
        ryaw_temp = [];
        rv_temp = [];
        ra_temp = [];
        rj_temp = [];

        for t = 0.0:dt:T
            time_temp = [time_temp, t];
            rx_temp = [rx_temp, xqp.calc_point(t)];
            ry_temp = [ry_temp, yqp.calc_point(t)];

            vx = xqp.calc_first_derivative(t);
            vy = yqp.calc_first_derivative(t);
            v = hypot(vx, vy);
            yaw = atan2(vy, vx);
            rv_temp = [rv_temp, v];
            ryaw_temp = [ryaw_temp, yaw];

            ax = xqp.calc_second_derivative(t);
            ay = yqp.calc_second_derivative(t);
            a = hypot(ax, ay);
            if length(rv_temp) >= 2 && rv_temp(end) - rv_temp(end-1) < 0.0
                a = -a;
            end
            ra_temp = [ra_temp, a];

            jx = xqp.calc_third_derivative(t);
            jy = yqp.calc_third_derivative(t);
            j = hypot(jx, jy);
            if length(ra_temp) >= 2 && ra_temp(end) - ra_temp(end-1) < 0.0
                j = -j;
            end
            rj_temp = [rj_temp, j];
        end

        if max(abs(ra_temp)) <= max_accel && max(abs(rj_temp)) <= max_jerk
            fprintf("find path!!\n");
            time = time_temp;
            rx = rx_temp;
            ry = ry_temp;
            ryaw = ryaw_temp;
            rv = rv_temp;
            ra = ra_temp;
            rj = rj_temp;
            break;
        end
    end
end
