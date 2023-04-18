function [] = DynamicBicycleModel()
    clear;
    close all;
    clc;
    
    disp("start Dynamic Bicycle model simulation");

    T = 100;
    a = ones(1, T);
    delta = deg2rad(ones(1, T));

    state = struct('x', 0, 'y', 0, 'yaw', 0, 'vx', 0.01, 'vy', 0, 'omega', 0);

    x = [];
    y = [];
    yaw = [];
    vx = [];
    vy = [];
    time = [];
    t = 0.0;

    dt = 0.1;  
    Cf = 1600 * 2.0; 
    Cr = 1700 * 2.0; 
    L = 5.0;
    Lr = L / 2.0;  
    Lf = L - Lr;  
    m = 1500;  
    Iz = 2250; 

    for i = 1:T
        t = t + dt;
        state = update_kinematic_bicycle(state, a(i), delta(i), dt, Cf, Cr, Lf, Lr, m, Iz);
        x = [x, state.x];
        y = [y, state.y];
        yaw = [yaw, state.yaw];
        vx = [vx, state.vx];
        vy = [vy, state.vy];
        time = [time, t];
    end
    
    figure;
    plot(x, y);
    xlabel("x[m]");
    ylabel("y[m]");
    axis("equal");
    grid on;
end

function state = update_kinematic_bicycle(state, a, delta, dt, Cf, Cr, Lf, Lr, m, Iz)
    state.x = state.x + state.vx * cos(state.yaw) * dt - state.vy * sin(state.yaw) * dt;
    state.y = state.y + state.vx * sin(state.yaw) * dt + state.vy * cos(state.yaw) * dt;
    state.yaw = state.yaw + state.omega * dt;
    Ffy = -Cf * atan2(((state.vy + Lf * state.omega) / state.vx - delta), 1.0);
    Fry = -Cr * atan2((state.vy - Lr * state.omega) / state.vx, 1.0);
    state.vx = state.vx + (a - Ffy * sin(delta) / m + state.vy * state.omega) * dt;
    state.vy = state.vy + (Fry / m + Ffy * cos(delta) / m - state.vx * state.omega) * dt;
    state.omega = state.omega + (Ffy * Lf * cos(delta) - Fry * Lr) / Iz * dt;
end
