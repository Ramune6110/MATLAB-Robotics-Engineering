clear;
close all;
clc;

%% main
% SafeDivide関数のテストケース
testCases = [
    10, 2;    % 通常の除算
    5, 0;     % 分母が 0
    3, eps;   % 分母が非常に小さい（eps に等しい）
    -4, -eps; % 負の数で非常に小さい分母
    0, 0;     % 両方 0
];

% 結果の表示用
fprintf('Numerator\tDenominator\tResult\n');
fprintf('----------\t------------\t------\n');

% 各テストケースに対して SafeDivide 関数を実行
for i = 1:size(testCases, 1)
    numerator = testCases(i, 1);
    denominator = testCases(i, 2);
    result = SafeDivide(numerator, denominator);
    
    % 結果の表示
    fprintf('%10.2f\t%12.2e\t%6.2f\n', numerator, denominator, result);
end

% AND関数のテスト
resultAnd = AND(true, false, true)% 例: false
resultAnd = AND(1, 0, 1)% 例: false

% OR関数のテスト
resultOr = OR(true, false, false) % 例: true
resultOr = OR(1, 0, 0)

% NOT関数のテスト
resultNot = NOT(true) % 例: false
resultNot = NOT(1)

% Switch関数のテスト
result1 = Switch(10, 1, 20);  % input2 が非ゼロなので、input1 (10) を出力
result2 = Switch(10, 0, 20);  % input2 がゼロなので、input3 (20) を出力

% 結果の表示
disp(['result1: ', num2str(result1)]);
disp(['result2: ', num2str(result2)]);

% Multiport Switch関数のテスト
result1 = MultiportSwitch(1, 10, 20, 30); % インデックス1なので、最初の入力 (10) を出力
result2 = MultiportSwitch(3, 10, 20, 30); % インデックス3なので、3番目の入力 (30) を出力

% 結果の表示
disp(['result1: ', num2str(result1)]);
disp(['result2: ', num2str(result2)]);

% テストデータ
xData = [1, 2, 3, 4, 5];
yData = [10, 20, 30, 40, 50];

% クエリポイント
input = 2.5;

% 線形補間を使用して呼び出し
outputLinear = oneDLookupTable(input, xData, yData, 'linear');

% スプライン補間を使用して呼び出し
outputSpline = oneDLookupTable(input, xData, yData, 'spline');

% 結果の表示
disp(['Linear Interpolation Output: ', num2str(outputLinear)]);
disp(['Spline Interpolation Output: ', num2str(outputSpline)]);

% 既知のデータポイント
[xData, yData] = meshgrid(1:5, 1:5); % 5x5グリッド
zData = magic(5); % 5x5のマジックスクエアをデータ値として使用

% クエリポイント
xQuery = 2.5;
yQuery = 3.5;

% 2-D Lookup Table関数の呼び出し（線形補間）
output = twoDLookupTable(xData, yData, zData, xQuery, yQuery, 'linear');

% 結果の表示
disp(['2-D Lookup Table Output: ', num2str(output)]);

% サチュレーションのテスト
x = -10:0.1:10; % テストデータ
lowerLimit = -2; % 下限値
upperLimit = 2;  % 上限値

% サチュレーション関数を適用
y = Saturation(x, lowerLimit, upperLimit);

% 結果のプロット
plot(x, y);
grid on;
xlabel('Original Value');
ylabel('Saturated Value');
title('Saturation Example');

% サチュレーションDynamic関数のテスト
inputValues = -10:10; % テスト用の入力範囲
dynamicLowerLimit = -5; % 動的下限値
dynamicUpperLimit = 5;  % 動的上限値

% 関数の適用と結果の格納
outputValues = arrayfun(@(x) SaturationDynamic(x, dynamicLowerLimit, dynamicUpperLimit), inputValues);

% 結果のプロット
plot(inputValues, outputValues);
grid on;
xlabel('Input Value');
ylabel('Output Value');
title('Saturation Dynamic Example');

% Unit Delay関数のテスト
for i = 1:5
    output = UnitDelay(i);
    disp(['Input: ', num2str(i), ', Output: ', num2str(output)]);
end

%% SafeDivide
function result = SafeDivide(numerator, denominator)
    % numerator: 分子
    % denominator: 分母

    % 分母が非常に小さい場合には、epsの逆数を使用
    tolerance = eps; % MATLABで定義されている浮動小数点数の精度
    if abs(denominator) < tolerance
        denominator = 1/tolerance;
    end

    % 安全な除算を実行
    result = single(numerator ./ denominator);
end

%% AND
function result = AND(varargin)
    % 複数の入力に対するAND演算
    result = all([varargin{:}]);
end

%% OR
function result = OR(varargin)
    % 複数の入力に対するOR演算
    result = any([varargin{:}]);
end

%% NOT
function result = NOT(input)
    % 単一の入力に対するNOT演算
    result = ~input;
end

%% Switch
function result = Switch(input1, input2, input3)
    % input1: 1番目の入力
    % input2: 2番目の入力（この入力に基づいて分岐）
    % input3: 3番目の入力

    % 2番目の入力が非ゼロの場合は1番目の入力を、そうでなければ3番目の入力を出力
    if input2 ~= 0
        result = input1;
    else
        result = input3;
    end
end

%% Multiport Switch
function result = MultiportSwitch(index, varargin)
    % index: 入力インデックス
    % varargin: 可変長の入力リスト

    % 入力リストからインデックスに対応する入力を選択
    if index >= 1 && index <= length(varargin)
        result = varargin{index};
    else
        error('Invalid index value.');
    end
end

%%  1-D Lookup Table 
function result = oneDLookupTable(u1, xData, yData, method)
    % u1: クエリポイント（補間したい入力値）
    % xData: 既知のデータポイントのX値（テーブルのインデックス）
    % yData: 既知のデータポイントのY値（テーブルのデータ）
    % method: 補間方法（例：'linear', 'spline', 'nearest'）

    % 補間方法が指定されていない場合は、デフォルトで 'linear' を使用
    if nargin < 4
        method = 'linear';
    end

    % 指定された補間方法を使用して出力を計算
    result = interp1(xData, yData, u1, method, 'extrap');
end

%% 
function result = twoDLookupTable(xData, yData, zData, xQuery, yQuery, method)
    % xData, yData: 既知のデータポイントのX値とY値のグリッド
    % zData: X-Yグリッド上のZ値（データ値）
    % xQuery, yQuery: クエリポイント（補間したいX-Y値）
    % method: 補間方法（例：'linear', 'spline', 'cubic'）

    % 補間方法が指定されていない場合は、デフォルトで 'linear' を使用
    if nargin < 6
        method = 'linear';
    end

    % 2次元補間を使用して出力を計算
    result = interp2(xData, yData, zData, xQuery, yQuery, method);
end

%% Saturation
function result = Saturation(x, lowerLimit, upperLimit)
    % x: サチュレーションを適用する値
    % lowerLimit: 下限値
    % upperLimit: 上限値

    % 下限値でサチュレーション
    result = max(x, lowerLimit);
    % 上限値でサチュレーション
    result = min(result, upperLimit);
end

%% Saturation Dynamic
function result = SaturationDynamic(input, lowerLimit, upperLimit)
    % input: サチュレーションを適用する値
    % lowerLimit: 動的下限値
    % upperLimit: 動的上限値

    % 入力を動的な下限値と上限値の範囲に制限
    result = min(max(input, lowerLimit), upperLimit);
end

%% Unit Delay
function result = UnitDelay(input)
    % 持続変数を使用して前の状態を保持
    persistent previousInput;
    
    % 初期呼び出し時に前の入力を0に設定
    if isempty(previousInput)
        previousInput = 0;
    end

    % 前回の入力を出力に設定
    result = previousInput;

    % 現在の入力を次回のために保持
    previousInput = input;
end
