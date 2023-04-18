function [] = AdvancedDigitalFilter()
    % �Q�l�����Fhttps://detail-infomation.com/filter-butterworth-bessel-chebyshev-elliptic/
    clear;
    close all;
    clc;

    % �T���v�����O�����ݒ�
    sampling_period = 0.001;

    % ���Ԏ��𐶐��i�f�[�^����1000�j
    time = 0:sampling_period:(1000 - 1) * sampling_period;

    % �M������ (���������g)
    signal = 5 * sin(2 * pi * 5 * time) + 2 * sin(2 * pi * 15 * time) + 1 * sin(2 * pi * 30 * time);

    % �t�B���^�ݒ�
    Fs = 1 / sampling_period; % �T���v�����O���g��
    low_cutoff = 10; % ���[�p�X�t�B���^�̃J�b�g�I�t���g��

    % �t�B���^�ݒ�
    filter_order = 4; % �t�B���^�̎���

    % �o�^�[���[�X�t�B���^
    [butter_b, butter_a] = butter(filter_order, low_cutoff/(Fs/2), 'low');
    filtered_butter = filtfilt(butter_b, butter_a, signal);

    % �`�F�r�V�F�t�t�B���^
    cheby_ripple = 0.5; % ���b�v��[dB]
    [cheby_b, cheby_a] = cheby1(filter_order, cheby_ripple, low_cutoff/(Fs/2), 'low');
    filtered_cheby = filtfilt(cheby_b, cheby_a, signal);

    % �x�b�Z���t�B���^
    [bessel_b, bessel_a] = besself(filter_order, low_cutoff/(Fs/2), 'low');
    filtered_bessel = filtfilt(bessel_b, bessel_a, signal);

    % ���ʂ̏o��
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
