clear;
close all;
clc;

%% main
% SafeDivide�֐��̃e�X�g�P�[�X
testCases = [
    10, 2;    % �ʏ�̏��Z
    5, 0;     % ���ꂪ 0
    3, eps;   % ���ꂪ���ɏ������ieps �ɓ������j
    -4, -eps; % ���̐��Ŕ��ɏ���������
    0, 0;     % ���� 0
];

% ���ʂ̕\���p
fprintf('Numerator\tDenominator\tResult\n');
fprintf('----------\t------------\t------\n');

% �e�e�X�g�P�[�X�ɑ΂��� SafeDivide �֐������s
for i = 1:size(testCases, 1)
    numerator = testCases(i, 1);
    denominator = testCases(i, 2);
    result = SafeDivide(numerator, denominator);
    
    % ���ʂ̕\��
    fprintf('%10.2f\t%12.2e\t%6.2f\n', numerator, denominator, result);
end

% AND�֐��̃e�X�g
resultAnd = AND(true, false, true)% ��: false
resultAnd = AND(1, 0, 1)% ��: false

% OR�֐��̃e�X�g
resultOr = OR(true, false, false) % ��: true
resultOr = OR(1, 0, 0)

% NOT�֐��̃e�X�g
resultNot = NOT(true) % ��: false
resultNot = NOT(1)

% Switch�֐��̃e�X�g
result1 = Switch(10, 1, 20);  % input2 ����[���Ȃ̂ŁAinput1 (10) ���o��
result2 = Switch(10, 0, 20);  % input2 ���[���Ȃ̂ŁAinput3 (20) ���o��

% ���ʂ̕\��
disp(['result1: ', num2str(result1)]);
disp(['result2: ', num2str(result2)]);

% Multiport Switch�֐��̃e�X�g
result1 = MultiportSwitch(1, 10, 20, 30); % �C���f�b�N�X1�Ȃ̂ŁA�ŏ��̓��� (10) ���o��
result2 = MultiportSwitch(3, 10, 20, 30); % �C���f�b�N�X3�Ȃ̂ŁA3�Ԗڂ̓��� (30) ���o��

% ���ʂ̕\��
disp(['result1: ', num2str(result1)]);
disp(['result2: ', num2str(result2)]);

% �e�X�g�f�[�^
xData = [1, 2, 3, 4, 5];
yData = [10, 20, 30, 40, 50];

% �N�G���|�C���g
input = 2.5;

% ���`��Ԃ��g�p���ČĂяo��
outputLinear = oneDLookupTable(input, xData, yData, 'linear');

% �X�v���C����Ԃ��g�p���ČĂяo��
outputSpline = oneDLookupTable(input, xData, yData, 'spline');

% ���ʂ̕\��
disp(['Linear Interpolation Output: ', num2str(outputLinear)]);
disp(['Spline Interpolation Output: ', num2str(outputSpline)]);

% ���m�̃f�[�^�|�C���g
[xData, yData] = meshgrid(1:5, 1:5); % 5x5�O���b�h
zData = magic(5); % 5x5�̃}�W�b�N�X�N�G�A���f�[�^�l�Ƃ��Ďg�p

% �N�G���|�C���g
xQuery = 2.5;
yQuery = 3.5;

% 2-D Lookup Table�֐��̌Ăяo���i���`��ԁj
output = twoDLookupTable(xData, yData, zData, xQuery, yQuery, 'linear');

% ���ʂ̕\��
disp(['2-D Lookup Table Output: ', num2str(output)]);

% �T�`�����[�V�����̃e�X�g
x = -10:0.1:10; % �e�X�g�f�[�^
lowerLimit = -2; % �����l
upperLimit = 2;  % ����l

% �T�`�����[�V�����֐���K�p
y = Saturation(x, lowerLimit, upperLimit);

% ���ʂ̃v���b�g
plot(x, y);
grid on;
xlabel('Original Value');
ylabel('Saturated Value');
title('Saturation Example');

% �T�`�����[�V����Dynamic�֐��̃e�X�g
inputValues = -10:10; % �e�X�g�p�̓��͔͈�
dynamicLowerLimit = -5; % ���I�����l
dynamicUpperLimit = 5;  % ���I����l

% �֐��̓K�p�ƌ��ʂ̊i�[
outputValues = arrayfun(@(x) SaturationDynamic(x, dynamicLowerLimit, dynamicUpperLimit), inputValues);

% ���ʂ̃v���b�g
plot(inputValues, outputValues);
grid on;
xlabel('Input Value');
ylabel('Output Value');
title('Saturation Dynamic Example');

% Unit Delay�֐��̃e�X�g
for i = 1:5
    output = UnitDelay(i);
    disp(['Input: ', num2str(i), ', Output: ', num2str(output)]);
end

%% SafeDivide
function result = SafeDivide(numerator, denominator)
    % numerator: ���q
    % denominator: ����

    % ���ꂪ���ɏ������ꍇ�ɂ́Aeps�̋t�����g�p
    tolerance = eps; % MATLAB�Œ�`����Ă��镂�������_���̐��x
    if abs(denominator) < tolerance
        denominator = 1/tolerance;
    end

    % ���S�ȏ��Z�����s
    result = single(numerator ./ denominator);
end

%% AND
function result = AND(varargin)
    % �����̓��͂ɑ΂���AND���Z
    result = all([varargin{:}]);
end

%% OR
function result = OR(varargin)
    % �����̓��͂ɑ΂���OR���Z
    result = any([varargin{:}]);
end

%% NOT
function result = NOT(input)
    % �P��̓��͂ɑ΂���NOT���Z
    result = ~input;
end

%% Switch
function result = Switch(input1, input2, input3)
    % input1: 1�Ԗڂ̓���
    % input2: 2�Ԗڂ̓��́i���̓��͂Ɋ�Â��ĕ���j
    % input3: 3�Ԗڂ̓���

    % 2�Ԗڂ̓��͂���[���̏ꍇ��1�Ԗڂ̓��͂��A�����łȂ����3�Ԗڂ̓��͂��o��
    if input2 ~= 0
        result = input1;
    else
        result = input3;
    end
end

%% Multiport Switch
function result = MultiportSwitch(index, varargin)
    % index: ���̓C���f�b�N�X
    % varargin: �ϒ��̓��̓��X�g

    % ���̓��X�g����C���f�b�N�X�ɑΉ�������͂�I��
    if index >= 1 && index <= length(varargin)
        result = varargin{index};
    else
        error('Invalid index value.');
    end
end

%%  1-D Lookup Table 
function result = oneDLookupTable(u1, xData, yData, method)
    % u1: �N�G���|�C���g�i��Ԃ��������͒l�j
    % xData: ���m�̃f�[�^�|�C���g��X�l�i�e�[�u���̃C���f�b�N�X�j
    % yData: ���m�̃f�[�^�|�C���g��Y�l�i�e�[�u���̃f�[�^�j
    % method: ��ԕ��@�i��F'linear', 'spline', 'nearest'�j

    % ��ԕ��@���w�肳��Ă��Ȃ��ꍇ�́A�f�t�H���g�� 'linear' ���g�p
    if nargin < 4
        method = 'linear';
    end

    % �w�肳�ꂽ��ԕ��@���g�p���ďo�͂��v�Z
    result = interp1(xData, yData, u1, method, 'extrap');
end

%% 
function result = twoDLookupTable(xData, yData, zData, xQuery, yQuery, method)
    % xData, yData: ���m�̃f�[�^�|�C���g��X�l��Y�l�̃O���b�h
    % zData: X-Y�O���b�h���Z�l�i�f�[�^�l�j
    % xQuery, yQuery: �N�G���|�C���g�i��Ԃ�����X-Y�l�j
    % method: ��ԕ��@�i��F'linear', 'spline', 'cubic'�j

    % ��ԕ��@���w�肳��Ă��Ȃ��ꍇ�́A�f�t�H���g�� 'linear' ���g�p
    if nargin < 6
        method = 'linear';
    end

    % 2������Ԃ��g�p���ďo�͂��v�Z
    result = interp2(xData, yData, zData, xQuery, yQuery, method);
end

%% Saturation
function result = Saturation(x, lowerLimit, upperLimit)
    % x: �T�`�����[�V������K�p����l
    % lowerLimit: �����l
    % upperLimit: ����l

    % �����l�ŃT�`�����[�V����
    result = max(x, lowerLimit);
    % ����l�ŃT�`�����[�V����
    result = min(result, upperLimit);
end

%% Saturation Dynamic
function result = SaturationDynamic(input, lowerLimit, upperLimit)
    % input: �T�`�����[�V������K�p����l
    % lowerLimit: ���I�����l
    % upperLimit: ���I����l

    % ���͂𓮓I�ȉ����l�Ə���l�͈̔͂ɐ���
    result = min(max(input, lowerLimit), upperLimit);
end

%% Unit Delay
function result = UnitDelay(input)
    % �����ϐ����g�p���đO�̏�Ԃ�ێ�
    persistent previousInput;
    
    % �����Ăяo�����ɑO�̓��͂�0�ɐݒ�
    if isempty(previousInput)
        previousInput = 0;
    end

    % �O��̓��͂��o�͂ɐݒ�
    result = previousInput;

    % ���݂̓��͂�����̂��߂ɕێ�
    previousInput = input;
end
