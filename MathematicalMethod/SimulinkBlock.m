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
