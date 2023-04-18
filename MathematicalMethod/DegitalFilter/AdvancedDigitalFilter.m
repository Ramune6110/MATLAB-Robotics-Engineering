function [] = AdvancedDigitalFilter()
    % 参考資料：https://detail-infomation.com/filter-butterworth-bessel-chebyshev-elliptic/
    clear;
    close all;
    clc;

    % サンプリング周期設定
    sampling_period = 0.001;

    % 時間軸を生成（データ長さ1000）
    time = 0:sampling_period:(1000 - 1) * sampling_period;

    % 信号生成 (合成正弦波)
    signal = 5 * sin(2 * pi * 5 * time) + 2 * sin(2 * pi * 15 * time) + 1 * sin(2 * pi * 30 * time);

    % フィルタ設定
    Fs = 1 / sampling_period; % サンプリング周波数
    low_cutoff = 10; % ローパスフィルタのカットオフ周波数

    % フィルタ設定
    filter_order = 4; % フィルタの次数

    % バターワースフィルタ
    [butter_b, butter_a] = butter(filter_order, low_cutoff/(Fs/2), 'low');
    filtered_butter = filtfilt(butter_b, butter_a, signal);

    % チェビシェフフィルタ
    cheby_ripple = 0.5; % リップル[dB]
    [cheby_b, cheby_a] = cheby1(filter_order, cheby_ripple, low_cutoff/(Fs/2), 'low');
    filtered_cheby = filtfilt(cheby_b, cheby_a, signal);

    % ベッセルフィルタ
    [bessel_b, bessel_a] = besself(filter_order, low_cutoff/(Fs/2), 'low');
    filtered_bessel = filtfilt(bessel_b, bessel_a, signal);

    % 結果の出力
    figure(1);
    subplot(4, 1, 1);
    plot(time, signal, 'r');
    title('Original Signal');

    subplot(4, 1, 2);
    plot(time, signal, 'r', time, filtered_butter, 'b');
    title('Butterworth Filtered Signal');

    subplot(4, 1, 3);
    plot(time, signal, 'r', time, filtered_cheby, 'b');
    title('Chebyshev Filtered Signal');

    subplot(4, 1, 4);
    plot(time, signal, 'r', time, filtered_bessel, 'b');
    title('Bessel Filtered Signal');
    
    figure(2);
    freqz(butter_b, butter_a)
    
    figure(3);
    freqz(cheby_b, cheby_a)
    
    figure(4);
    freqz(bessel_b, bessel_a)
end
