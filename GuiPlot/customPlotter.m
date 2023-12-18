function customPlotter
    % メインウィンドウを作成
    main_fig = figure('Name', 'Custom Plotter', 'Position', [100, 100, 900, 600]);

    % CSVまたはMATファイルを読み込むボタン
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Load CSV/MAT', ...
        'Position', [10, 550, 120, 30], 'Callback', @load_file);

    % 変数の一覧表示
    data_table = uitable(main_fig, 'ColumnName', {'Variable', 'Value', 'Min', 'Max', 'Mean'}, ...
        'ColumnWidth', {100, 100, 100, 100, 100}, 'Data', cell(0, 5), 'Position', [10, 50, 520, 450]);

    % プロット設定
    uicontrol('Parent', main_fig, 'Style', 'text', 'String', 'X:', 'Position', [550, 540, 20, 20]);
    x_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', 'String', {'None'}, ...
        'Position', [570, 540, 120, 25]);

    uicontrol('Parent', main_fig, 'Style', 'text', 'String', 'Y:', 'Position', [700, 540, 20, 20]);
    y_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', 'String', {'None'}, ...
        'Position', [720, 540, 120, 25]);

    uicontrol('Parent', main_fig, 'Style', 'text', 'String', 'Z:', 'Position', [550, 500, 20, 20]);
    z_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', 'String', {'None'}, ...
        'Position', [570, 500, 120, 25]);

    uicontrol('Parent', main_fig, 'Style', 'text', 'String', 'Color:', 'Position', [700, 500, 40, 20]);
    color_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', 'String', {'r', 'g', 'b', 'c', 'm', 'y', 'k'}, ...
        'Position', [740, 500, 50, 25]);

    uicontrol('Parent', main_fig, 'Style', 'text', 'String', 'Marker:', 'Position', [800, 500, 50, 20]);
    marker_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', 'String', {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'}, ...
        'Position', [850, 500, 50, 25]);

    % プロットボタン
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot', ...
        'Position', [550, 450, 100, 30], 'Callback', @plot_data);

    % データを格納する変数
    loaded_data = [];

    % コールバック関数
    function load_file(~, ~)
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

        vars = loaded_data.Properties.VariableNames';
        data_table.Data = [vars];
%         data_table.Data = [vars, ...
%             repmat({''}, length(vars), 1), ...
%             num2cell(min(loaded_data.Variables)), ...
%             num2cell(max(loaded_data.Variables)), ...
%             num2cell(mean(loaded_data.Variables))];
%         x_popup.String = ['None', vars];
%         y_popup.String = ['None', vars];
%         z_popup.String = ['None', vars];
    end

    function plot_data(~, ~)
        x_var = x_popup.Value - 1;
        y_var = y_popup.Value - 1;
        z_var = z_popup.Value - 1;

        if x_var == 0 || y_var == 0
            warndlg('XおよびYデータを選択してください。', '警告');
            return;
        end

        color = color_popup.String{color_popup.Value};
        marker = marker_popup.String{marker_popup.Value};

        if z_var == 0 % 2次元プロット
            figure(gcf);
            hold on;
            plot(loaded_data{:, x_var}, loaded_data{:, y_var}, ['-', color, marker]);
            xlabel(loaded_data.Properties.VariableNames{x_var});
            ylabel(loaded_data.Properties.VariableNames{y_var});
            hold off;
        else % 3次元プロット
            figure(gcf);
            hold on;
            scatter3(loaded_data{:, x_var}, loaded_data{:, y_var}, loaded_data{:, z_var}, ...
                [], color, marker);
            xlabel(loaded_data.Properties.VariableNames{x_var});
            ylabel(loaded_data.Properties.VariableNames{y_var});
            zlabel(loaded_data.Properties.VariableNames{z_var});
            hold off;
        end
    end
end

