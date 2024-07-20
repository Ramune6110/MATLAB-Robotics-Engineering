clear;
close all;
clc;

% 車両のパラメータ
vehicle_length = 4; % 車両の長さ（メートル）
vehicle_width = 2;  % 車両の幅（メートル）
initial_position = [0, 0]; % 初期位置
center_height_offset = 1.0; % 車両中心の高さオフセット
front_tire_offset = 1.5; % 前タイヤの位置オフセット
rear_tire_offset = -1.5; % 後タイヤの位置オフセット

% シミュレーションのパラメータ
time_step = 0.1; % 時間の刻み幅
total_time = 5; % シミュレーションの総時間
acceleration = -2; % 任意の加速度（m/s^2）
slope_angle = 10; % 勾配の角度（度）

% 変数の初期化
time = 0:time_step:total_time;
num_steps = length(time);
positions = zeros(num_steps, 2);
velocities = zeros(num_steps, 1);
accelerations = zeros(num_steps, 1);
positions(1, :) = initial_position;

% 車両の動きを計算
for i = 2:num_steps
    accelerations(i) = acceleration; % 加速度の計算
    velocities(i) = accelerations(i) * time(i); % 速度の計算
    displacement = 0.5 * accelerations(i) * time(i)^2; % 変位の計算
    positions(i, 1) = displacement; % X方向の位置
    positions(i, 2) = displacement * tand(slope_angle); % 坂道の高さ
end

% アニメーションのセットアップ
figure('Position', [50, 50, 1000, 600]); % ウィンドウのサイズを大きく設定
subplot(4, 2, [1 3 5 7]); % 左側全てを使用
axis equal;
hold on;
xlabel('X Position (m)');
ylabel('Z Position (m)');
main_title = title(['Vehicle Dynamics on a Slope (Slope Angle = ', num2str(slope_angle), '°)']);
grid on;

% セダン型の車両の描画データ
vehicle_shape_x = [-2, -0.5, 1.5, 2, 3.5, 3.5, 2, 1.5, -1.5, -2, -2] .* (-1);
vehicle_shape_z = [-0.5, -0.5, -0.5, -0.5, -0.5, 0, 0.5, 1, 1, 0.5, -0.5];

% タイヤの描画データ
tire_radius = 0.5;
tire_angle = linspace(0, 2*pi, 20);
tire_x = tire_radius * cos(tire_angle);
tire_z = tire_radius * sin(tire_angle);

% 坂道の描画
plot([positions(1,1), positions(end,1)], [positions(1,2), positions(end,2)], 'k-', 'LineWidth', 2);
plot([positions(1,1), positions(end,1)], [positions(end,2), positions(end,2)], 'k-', 'LineWidth', 2); % 地面のラインを描画

% 壁の描画
brick_wall_x = [positions(end,1) - 1, positions(end,1) - 1, positions(end,1), positions(end,1)];
brick_wall_y = [0, positions(end,2), positions(end,2), 0];
fill(brick_wall_x, brick_wall_y, [0.8, 0.3, 0]); % 茶色の壁を描画

% 車両とタイヤの初期描画
vehicle_plot = fill(vehicle_shape_x, vehicle_shape_z, 'b');
front_tire_plot = fill(tire_x, tire_z, 'k');
rear_tire_plot = fill(tire_x, tire_z, 'k');
cg_plot = plot(0, 0, 'ro', 'MarkerFaceColor', 'r'); % 車両重心の初期描画
% X軸とZ軸のリアルタイム縦線と横線
cg_x_line = plot([0, 0], [-5, 15], 'r--');
cg_z_line = plot([5, -30], [0, 0], 'r--');

% サブプロットの設定
subplot(4, 2, 2);
pos_x_plot = plot(time, positions(:,1));
hold on;
current_pos_x_line = plot([0, 0], [min(positions(:,1)), max(positions(:,1))], 'k');
current_pos_x_marker = plot(0, positions(1,1), 'ko');
pos_x_title = title(sprintf('X Position: %.2f m', positions(1,1)));
xlabel('Time (s)');
ylabel('X Position (m)');
grid on;
xlim([0, total_time]);
ylim([min(positions(:,1)), max(positions(:,1))]);

subplot(4, 2, 4);
pos_z_plot = plot(time, positions(:,2));
hold on;
current_pos_z_line = plot([0, 0], [min(positions(:,2)), max(positions(:,2))], 'k');
current_pos_z_marker = plot(0, positions(1,2), 'ko');
pos_z_title = title(sprintf('Z Position: %.2f m', positions(1,2)));
xlabel('Time (s)');
ylabel('Z Position (m)');
grid on;
xlim([0, total_time]);
ylim([min(positions(:,2)), max(positions(:,2))]);

subplot(4, 2, 6);
vel_plot = plot(time, velocities);
hold on;
current_vel_line = plot([0, 0], [min(velocities), max(velocities)], 'k');
current_vel_marker = plot(0, velocities(1), 'ko');
vel_title = title(sprintf('Velocity: %.2f m/s', velocities(1)));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;
xlim([0, total_time]);
ylim([min(velocities), max(velocities)]);

subplot(4, 2, 8);
acc_plot = plot(time, accelerations);
hold on;
current_acc_line = plot([0, 0], [min(accelerations), max(accelerations)], 'k');
current_acc_marker = plot(0, accelerations(1), 'ko');
acc_title = title(sprintf('Acceleration: %.2f m/s^2', accelerations(1)));
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
grid on;
xlim([0, total_time]);
ylim([min(accelerations), max(accelerations)]);

% GIF保存のための設定
gif_filename = 'vehicle_dynamics.gif';
frame_delay = 0.1; % GIFのフレームの遅延（秒）
is_first_frame = true;

% 動画保存の設定
video_filename = 'vehicle_dynamics.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 1/time_step;
open(v);

for i = 1:num_steps
    % 車両の姿勢角度を計算
    angle = slope_angle; % 勾配角度
    
    % 車両の位置と姿勢を更新
    [cg_x, cg_z] = rotate_and_translate(vehicle_plot, front_tire_plot, rear_tire_plot, positions(i, :), angle, vehicle_shape_x, vehicle_shape_z, tire_x, tire_z, slope_angle, center_height_offset, front_tire_offset, rear_tire_offset);
    
    % サブプロットの更新
    set(pos_x_plot, 'YData', positions(:, 1));
    set(pos_x_plot, 'XData', time);
    set(pos_z_plot, 'YData', positions(:, 2));
    set(pos_z_plot, 'XData', time);
    set(vel_plot, 'YData', velocities);
    set(vel_plot, 'XData', time);
    set(acc_plot, 'YData', accelerations);
    set(acc_plot, 'XData', time);
    
    % 現在の値を示す縦線の更新
    set(current_pos_x_line, 'XData', [time(i), time(i)]);
    set(current_pos_x_marker, 'XData', time(i), 'YData', positions(i, 1));
    set(pos_x_title, 'String', sprintf('X Position: %.2f m', positions(i, 1)));
    
    set(current_pos_z_line, 'XData', [time(i), time(i)]);
    set(current_pos_z_marker, 'XData', time(i), 'YData', positions(i, 2));
    set(pos_z_title, 'String', sprintf('Z Position: %.2f m', positions(i, 2)));
    
    set(current_vel_line, 'XData', [time(i), time(i)]);
    set(current_vel_marker, 'XData', time(i), 'YData', velocities(i));
    set(vel_title, 'String', sprintf('Velocity: %.2f m/s', velocities(i)));
    
    set(current_acc_line, 'XData', [time(i), time(i)]);
    set(current_acc_marker, 'XData', time(i), 'YData', accelerations(i));
    set(acc_title, 'String', sprintf('Acceleration: %.2f m/s^2', accelerations(i)));
    
    % 車両重心の位置を更新
    set(cg_plot, 'XData', cg_x, 'YData', cg_z);
    
    % X軸とZ軸の縦線と横線を更新
    set(cg_x_line, 'XData', [cg_x, cg_x]);
    set(cg_z_line, 'YData', [cg_z, cg_z]);

    % メインタイトルの更新
    set(main_title, 'String', sprintf('Vehicle Dynamics on a Slope (Slope Angle = %d°) | CG X: %.2f m, CG Z: %.2f m', slope_angle, cg_x, cg_z));
    
    % 描画の更新
    drawnow;
    
    % GIFのフレームを保存
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if is_first_frame
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', frame_delay);
        is_first_frame = false;
    else
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_delay);
    end
    
    % 動画のフレームを保存
    writeVideo(v, frame);
    
    pause(time_step);
end

% 動画ファイルを閉じる
close(v);

% 描画の更新関数
function [cg_x, cg_z] = rotate_and_translate(vehicle_plot, front_tire_plot, rear_tire_plot, position, angle, vehicle_shape_x, vehicle_shape_z, tire_x, tire_z, slope_angle, center_height_offset, front_tire_offset, rear_tire_offset)
    % 回転行列
    R = [cosd(angle), -sind(angle); sind(angle), cosd(angle)];
    
    % 車両の頂点を計算
    vertices = [vehicle_shape_x; vehicle_shape_z];
    
    % タイヤの位置を計算
    front_tire_position = position' + [front_tire_offset; 0];
    rear_tire_position = position' + [rear_tire_offset; 0];
    
    % タイヤの位置を坂道に合わせて調整
    front_tire_position(2) = front_tire_position(1) * tand(slope_angle);
    rear_tire_position(2) = rear_tire_position(1) * tand(slope_angle);
    
    % 車両の中心位置をタイヤ位置から計算
    vehicle_center_position = (front_tire_position + rear_tire_position) / 2 + [0; center_height_offset];
    
    % 頂点を回転
    rotated_vertices = R * vertices;
    
    % 頂点を平行移動
    translated_vertices = rotated_vertices + vehicle_center_position;
    
    % 車両の高さ調整
    front_tire_position(2) = front_tire_position(2) + (center_height_offset - 0.5);
    rear_tire_position(2) = rear_tire_position(2) + (center_height_offset - 0.5);
    
    % 描画の更新
    set(vehicle_plot, 'XData', translated_vertices(1, :));
    set(vehicle_plot, 'YData', translated_vertices(2, :));
    
    % タイヤの更新
    set(front_tire_plot, 'XData', tire_x + front_tire_position(1));
    set(front_tire_plot, 'YData', tire_z + front_tire_position(2));
    set(rear_tire_plot, 'XData', tire_x + rear_tire_position(1));
    set(rear_tire_plot, 'YData', tire_z + rear_tire_position(2));
    
    % 車両重心の位置を返す
    cg_x = vehicle_center_position(1);
    cg_z = vehicle_center_position(2);
end
