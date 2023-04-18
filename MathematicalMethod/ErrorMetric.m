function [] = ErrorMetric()
    % �Q�l�����Fhttps://analysis-navi.com/?p=2875
    clear;
    close all;
    clc;
    
    % �T���v�����O�����ݒ�
    sampling_period = 0.1;

    % ���Ԏ��𐶐��i�f�[�^����100�j
    time = 0:sampling_period:(100 - 1) * sampling_period;

    % ���Ԏ��ɑ΂��鐳��l�Ɨ\���l�̎��n��f�[�^�𐶐�
    % �����ł̓T���v���f�[�^�Ƃ��ė������g�p
    rng(0); % �����̃V�[�h���Œ�i�Č����̂��߁j
    true_values = rand(1, 100) * 100;
    predicted_values = true_values + randn(1, 100) * 50; % �m�C�Y���������\���l

    % �덷���v�Z
    errors = true_values - predicted_values;

    % 1. ���ό덷 (Mean Error�FME)
    mean_error = mean(errors);

    % 2. ���ϐ�Ό덷 (Mean Absolute Error�FMAE)
    mean_absolute_error = mean(abs(errors));

    % 3. ���ϕ������덷 (Root Mean Squared Error�FRMSE)
    mean_squared_error = sqrt(mean(errors.^2));

    % 4. ���ό덷�� (Mean Error Rate�FMPE)
    mean_error_rate =  mean(errors ./ true_values);

    % 5. ���ϐ�Ό덷�� (Mean Absolute Error Rate)
    mean_absolute_error_rate = mean(abs(errors ./ true_values));
    
    % 6. ���ϕ������덷�� (Mean Squared Error Rate)
    mean_squared_error_rate = sqrt(mean((errors ./ true_values).^2));

    % �덷�]���w�W��\��
    disp(['1. Mean Error: ', num2str(mean_error)])
    disp(['2. Mean Absolute Error: ', num2str(mean_absolute_error)])
    disp(['3. Root Mean Squared Error: ', num2str(mean_squared_error)])
    disp(['4. Mean Error Rate: ', num2str(mean_error_rate), '%'])
    disp(['5. Mean Absolute Error Rate: ', num2str(mean_absolute_error_rate), '%'])
    disp(['6. Mean Squared Error Rate: ', num2str(mean_squared_error_rate), '%'])

    % �O���t�\��
    figure(1)
    subplot(211)
    plot(time, true_values, 'r', time, predicted_values, 'b')
    xlabel('Time[s]')
    legend('����l', '�\���l')
    
    subplot(212)
    plot(time, errors, 'g')
    xlabel('Time[s]')
    legend('�덷')
end