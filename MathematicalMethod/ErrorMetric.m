function [] = ErrorMetric()
    % 参考資料：https://analysis-navi.com/?p=2875
    clear;
    close all;
    clc;
    
    % サンプリング周期設定
    sampling_period = 0.1;

    % 時間軸を生成（データ長さ100）
    time = 0:sampling_period:(100 - 1) * sampling_period;

    % 時間軸に対する正解値と予測値の時系列データを生成
    % ここではサンプルデータとして乱数を使用
    rng(0); % 乱数のシードを固定（再現性のため）
    true_values = rand(1, 100) * 100;
    predicted_values = true_values + randn(1, 100) * 50; % ノイズを加えた予測値

    % 誤差を計算
    errors = true_values - predicted_values;

    % 1. 平均誤差 (Mean Error：ME)
    mean_error = mean(errors);

    % 2. 平均絶対誤差 (Mean Absolute Error：MAE)
    mean_absolute_error = mean(abs(errors));

    % 3. 平均平方二乗誤差 (Root Mean Squared Error：RMSE)
    mean_squared_error = sqrt(mean(errors.^2));

    % 4. 平均誤差率 (Mean Error Rate：MPE)
    mean_error_rate =  mean(errors ./ true_values);

    % 5. 平均絶対誤差率 (Mean Absolute Error Rate)
    mean_absolute_error_rate = mean(abs(errors ./ true_values));
    
    % 6. 平均平方二乗誤差率 (Mean Squared Error Rate)
    mean_squared_error_rate = sqrt(mean((errors ./ true_values).^2));

    % 誤差評価指標を表示
    disp(['1. Mean Error: ', num2str(mean_error)])
    disp(['2. Mean Absolute Error: ', num2str(mean_absolute_error)])
    disp(['3. Root Mean Squared Error: ', num2str(mean_squared_error)])
    disp(['4. Mean Error Rate: ', num2str(mean_error_rate), '%'])
    disp(['5. Mean Absolute Error Rate: ', num2str(mean_absolute_error_rate), '%'])
    disp(['6. Mean Squared Error Rate: ', num2str(mean_squared_error_rate), '%'])

    % グラフ表示
    figure(1)
    subplot(211)
    plot(time, true_values, 'r', time, predicted_values, 'b')
    xlabel('Time[s]')
    legend('正解値', '予測値')
    
    subplot(212)
    plot(time, errors, 'g')
    xlabel('Time[s]')
    legend('誤差')
end