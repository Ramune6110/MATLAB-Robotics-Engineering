function simple_data_viewer()
% ウィンドウを作成
main_fig = figure('Name', 'シンプルデータビューア', 'NumberTitle', 'off', ...
    'MenuBar', 'none', 'ToolBar', 'none', 'Position', [100, 100, 600, 400]);

% ボタンを作成
load_button = uicontrol(main_fig, 'Style', 'pushbutton', 'String', 'ファイルを開く', ...
    'Position', [10, 360, 100, 30], 'Callback', @load_file);

% データ表示用の表を作成
data_table = uitable(main_fig, 'ColumnName', {'変数名', '最小値', '最大値', '平均値'}, ...
    'Position', [10, 10, 580, 340], 'ColumnWidth', {100, 150, 150, 150});

% データを格納する変数
loaded_data = [];

    function load_file(~, ~)
        % ファイルを選択
        [filename, pathname] = uigetfile({'*.csv;*.mat', 'CSV/MAT Files (*.csv, *.mat)'}, 'Select a CSV or MAT file');
        if isequal(filename, 0)
            return;
        end

        [~, ~, ext] = fileparts(filename);
        if strcmp(ext, '.csv')
            opts = detectImportOptions(fullfile(pathname, filename), 'FileType', 'text', 'Encoding', 'UTF-8');
            loaded_data = readtable(fullfile(pathname, filename), opts);
        elseif strcmp(ext, '.mat')
            loaded_data = load(fullfile(pathname, filename));
            loaded_data = struct2table(loaded_data);
        end

        % 表にデータを表示
        vars = loaded_data.Properties.VariableNames';
        data_table.Data = [vars, ...
            num2cell(min(loaded_data.Variables)), ...
            num2cell(max(loaded_data.Variables)), ...
            num2cell(mean(loaded_data.Variables))];
    end
end