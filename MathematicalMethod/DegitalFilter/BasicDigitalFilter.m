function [] = BasicDigitalFilter()
    % �Q�l�����Fhttps://www.ni.com/docs/ja-JP/bundle/diadem/page/genmaths/genmaths/calc_digitalfilter.htm
    %�E���[�p�X�t�B���^: �Ⴂ���g����ʉ߂����A�������g�������������܂�
    %�E�n�C�p�X�t�B���^: �������g����ʉ߂����A�Ⴂ���g�������������܂�
    %�E�o���h�p�X�t�B���^: ����̑ш�̎��g����ʉ߂����܂�
    %�E�o���h�X�g�b�v�t�B���^: ����̑ш�̎��g�������������܂�
   
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
    high_cutoff = 20; % �n�C�p�X�t�B���^�̃J�b�g�I�t���g��
    band_pass = [8, 25]; % �o���h�p�X�t�B���^�̎��g���͈�
    band_stop = [12, 22]; % �o���h�X�g�b�v�t�B���^�̎��g���͈�

    % ���[�p�X�t�B���^
    lpf = designfilt('lowpassiir', 'FilterOrder', 4, 'HalfPowerFrequency', low_cutoff/(Fs/2));
    filtered_lpf = filtfilt(lpf, signal);

    % �n�C�p�X�t�B���^
    hpf = designfilt('highpassiir', 'FilterOrder', 4, 'HalfPowerFrequency', high_cutoff/(Fs/2));
    filtered_hpf = filtfilt(hpf, signal);

    % �o���h�p�X�t�B���^
    bpf = designfilt('bandpassiir', 'FilterOrder', 4, 'HalfPowerFrequency1', band_pass(1)/(Fs/2), 'HalfPowerFrequency2', band_pass(2)/(Fs/2));
    filtered_bpf = filtfilt(bpf, signal);

    % �o���h�X�g�b�v�t�B���^
    bsf = designfilt('bandstopiir', 'FilterOrder', 4, 'HalfPowerFrequency1', band_stop(1)/(Fs/2), 'HalfPowerFrequency2', band_stop(2)/(Fs/2));
    filtered_bsf = filtfilt(bsf, signal);

    % �O���t�\��
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
