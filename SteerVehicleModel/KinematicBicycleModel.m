function [] = KinematicBicycleModel()
    clear;
    close all;
    clc;

    disp("start Kinematic Bicycle model simulation");

    T = 100;
    a = ones(1, T);
    delta = deg2rad(ones(1, T));

    state = struct('x', 0, 'y', 0, 'yaw', 0, 'v', 0, 'beta', 0);

    x = [];
    y = [];
    yaw = [];
    v = [];
    beta = [];
    time = [];
    t = 0.0;
    dt = 0.1;  % ����Fdt����`����Ă���Ƃ��낪������Ȃ���������
    L = 2.9;   % ����FL����`����Ă���Ƃ��낪������Ȃ���������
    Lr = L / 2;

    for i = 1:T
        t = t + dt;
        state = update(state, a(i), delta(i), Lr, L, dt);
        x = [x, state.x];
        y = [y, state.y];
        yaw = [yaw, state.yaw];
        v = [v, state.v];
        beta = [beta, state.beta];
        time = [time, t];
    end

    fig = figure(1);
    fig.Position = [0 0 1368 700];
    subplot(211)
    plot(x, y);
    xlabel("x[m]");
    ylabel("y[m]");
    axis("equal");
    grid on;
    
    subplot(212)
    plot(time, delta);
    xlabel("Time[s]");
    ylabel("delta[rad]");
    ylim([0, 0.03])
    axis("equal");
    grid on;
end

function state = update(state, a, delta, Lr, L, dt)
    state.beta = atan2(Lr / L * tan(delta), 1.0);
    state.x = state.x + state.v * cos(state.yaw + state.beta) * dt;
    state.y = state.y + state.v * sin(state.yaw + state.beta) * dt;
    state.yaw = state.yaw + state.v / Lr * sin(state.beta) * dt;
    state.v = state.v + a * dt;
end
