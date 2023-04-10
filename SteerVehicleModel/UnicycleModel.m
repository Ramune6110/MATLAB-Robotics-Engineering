function [] = UnicycleModel()
    clear;
    close all;
    clc;
    
    disp("start unicycle simulation");

    T = 100;
    a = ones(1, T);
    delta = deg2rad(ones(1, T));

    state = struct('x', 0, 'y', 0, 'yaw', 0, 'v', 0);

    x = [];
    y = [];
    yaw = [];
    v = [];
    dt = 0.1;  % 仮定：dtが定義されているところが見つからなかったため
    L = 2.9;   % 仮定：Lが定義されているところが見つからなかったため

    for i = 1:T
        state = update_unicycle(state, a(i), delta(i), L, dt);

        x = [x, state.x];
        y = [y, state.y];
        yaw = [yaw, state.yaw];
        v = [v, state.v];
    end

    figure;
    plot(x, y);
    axis("equal");
    grid on;
end

function state = update_unicycle(state, a, delta, L, dt)
    state.x = state.x + state.v * cos(state.yaw) * dt;
    state.y = state.y + state.v * sin(state.yaw) * dt;
    state.yaw = state.yaw + state.v / L * tan(delta) * dt;
    state.v = state.v + a * dt;
end
