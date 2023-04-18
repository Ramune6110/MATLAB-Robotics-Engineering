function [] = BasicDigitalFilter()
    % 参考資料：https://www.ni.com/docs/ja-JP/bundle/diadem/page/genmaths/genmaths/calc_digitalfilter.htm
    %・ローパスフィルタ: 低い周波数を通過させ、高い周波数を減衰させます
    %・ハイパスフィルタ: 高い周波数を通過させ、低い周波数を減衰させます
    %・バンドパスフィルタ: 特定の帯域の周波数を通過させます
    %・バンドストップフィルタ: 特定の帯域の周波数を減衰させます
   
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
    high_cutoff = 20; % ハイパスフィルタのカットオフ周波数
    band_pass = [8, 25]; % バンドパスフィルタの周波数範囲
    band_stop = [12, 22]; % バンドストップフィルタの周波数範囲

    % ローパスフィルタ
    lpf = designfilt('lowpassiir', 'FilterOrder', 4, 'HalfPowerFrequency', low_cutoff/(Fs/2));
    filtered_lpf = filtfilt(lpf, signal);

    % ハイパスフィルタ
    hpf = designfilt('highpassiir', 'FilterOrder', 4, 'HalfPowerFrequency', high_cutoff/(Fs/2));
    filtered_hpf = filtfilt(hpf, signal);

    % バンドパスフィルタ
    bpf = designfilt('bandpassiir', 'FilterOrder', 4, 'HalfPowerFrequency1', band_pass(1)/(Fs/2), 'HalfPowerFrequency2', band_pass(2)/(Fs/2));
    filtered_bpf = filtfilt(bpf, signal);

    % バンドストップフィルタ
    bsf = designfilt('bandstopiir', 'FilterOrder', 4, 'HalfPowerFrequency1', band_stop(1)/(Fs/2), 'HalfPowerFrequency2', band_stop(2)/(Fs/2));
    filtered_bsf = filtfilt(bsf, signal);

    % グラフ表示
    figure(1);
    subplot(221)
    plot(time, signal, 'r', time, filtered_lpf, 'b');
    title('Difference between original and LPF');

    subplot(222)
    plot(time, signal, 'r', time, filtered_hpf, 'b');
    title('Difference between original and HPF');

    subplot(223)
    plot(time, signal, 'r', time, filtered_bpf, 'b');
    title('Difference between original and BPF');

    subplot(224)
    plot(time, signal, 'r', time, filtered_bsf, 'b');
    title('Difference between original and BSF');
    
    fvtool(lpf)

    fvtool(hpf)
    
    fvtool(bpf)
    
    fvtool(bsf)
end
