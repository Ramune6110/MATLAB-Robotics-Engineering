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
