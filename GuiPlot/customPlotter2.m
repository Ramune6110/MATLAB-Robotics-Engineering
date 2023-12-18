function customPlotter2
    clear;
    close all;
    clc;
    
    % メインウィンドウを作成
    main_fig = figure('Name', 'Custom Plotter', 'Position', [100, 100, 900, 600]);

    % CSVまたはMATファイルを読み込むボタン
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Load CSV/MAT', ...
        'Position', [10, 550, 120, 30], 'Callback', @load_file);

    % 変数の一覧表示
    data_table = uitable(main_fig, 'ColumnName', {'Variable', 'Value'}, ...
        'ColumnWidth', {100, 100}, 'Data', cell(0, 2), 'Position', [10, 50, 200, 450], ...
        'CellSelectionCallback', @select_variable);

    % 軸データ設定ボタン
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'X Data Set', ...
        'Position', [220, 550, 100, 30], 'Callback', @(~,~)set_axis_data(1));
%     x_popup = main_fig.Number;
        x_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', 'String', {'None'}, ...
        'Position', [570, 540, 120, 25]);
    
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Y Data Set', ...
        'Position', [220, 510, 100, 30], 'Callback', @(~,~)set_axis_data(2));
%     y_popup = main_fig.Number;
    y_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', 'String', {'None'}, ...
    'Position', [720, 540, 120, 25]);
    
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Z Data Set', ...
        'Position', [220, 470, 100, 30], 'Callback', @(~,~)set_axis_data(3));
%     z_popup = main_fig.Number;
        z_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', 'String', {'None'}, ...
        'Position', [570, 500, 120, 25]);

%     uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'X Data Set', ...
%     'Position', [10, 510, 100, 30], 'Callback', {@(h,e)set_axis_data_callback(1)});
%     uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Y Data Set', ...
%         'Position', [10, 470, 100, 30], 'Callback', {@(h,e)set_axis_data_callback(2)});
%     uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Z Data Set', ...
%         'Position', [10, 430, 100, 30], 'Callback', {@(h,e)set_axis_data_callback(3)});

    % プロット設定
    uicontrol('Parent', main_fig, 'Style', 'text', 'String', 'Color:', ...
        'Position', [330, 550, 40, 20]);
    color_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', ...
        'String', {'r', 'g', 'b', 'c', 'm', 'y', 'k'}, ...
        'Position', [370, 550, 50, 25]);

    uicontrol('Parent', main_fig, 'Style', 'text', 'String', 'Marker:', ...
        'Position', [330, 510, 50, 20]);
    marker_popup = uicontrol('Parent', main_fig, 'Style', 'popupmenu', ...
        'String', {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'}, ...
        'Position', [370, 510, 50, 25]);

    % プロットボタン
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot', ...
        'Position', [330, 470, 100, 30], 'Callback', @plot_data);
    
    % Add menu to main figure
    fig_menu = uimenu(main_fig, 'Text', 'Figures');
    uimenu(fig_menu, 'Text', '二次元グラフ', 'Callback', @plot_2D_empty);
    uimenu(fig_menu, 'Text', '三次元グラフ', 'Callback', @plot_3D_empty);
    
        % 2Dプロットボタン
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot 2D', ...
        'Position', [520, 550, 100, 30], 'Callback', @plot_2D);
    % 3Dプロットボタン
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot 3D', ...
        'Position', [520, 510, 100, 30], 'Callback', @plot_3D);
    % データを格納する変数
    loaded_data = [];
    selected_variables = {0, 0, 0};
    axis_data = [];
    
    % コールバック関数
    function load_file(~, ~)
        [filename, pathname] = uigetfile({'*.csv;*.mat', 'CSV/MAT Files (*.csv, *.mat)'}, 'Select a CSV or MAT file');
        if isequal(filename, 0)
            return;
        end
        [~, ~, ext] = fileparts(filename);
        if strcmp(ext, '.csv')
            opts = detectImportOptions(fullfile(pathname, filename), 'TextType', 'string');
            opts.Encoding = 'UTF-8';
            loaded_data = readtable(fullfile(pathname, filename), opts);
        elseif strcmp(ext, '.mat')
            loaded_data = load(fullfile(pathname, filename));
            loaded_data = struct2table(loaded_data);
        end
        data_table.Data = [loaded_data.Properties.VariableNames', ...
            repmat({''}, length(loaded_data.Properties.VariableNames), 1)];
    end

    function select_variable(~, eventdata)
        if isempty(eventdata.Indices)
            return;
        end
        selected_variables{eventdata.Indices(1)} = eventdata.Indices(2);
    end

%     function set_axis_data(axis_idx)
%         if selected_variables{axis_idx} ~= 0
%             data_table.Data{selected_variables{axis_idx}, 2} = sprintf('Axis %s', 'XYZ'(axis_idx));
%         end
%     end
    
    function set_axis_data(axis_idx)
        if isempty(selected_variables)
            warndlg(sprintf('データを選択してください。'), 'Warning');
            return;
        end
        axis_data{axis_idx} = selected_variables(1);
%         axis_var_text(axis_idx).String = csv_data.Properties.VariableNames{axis_data{axis_idx}};
        %disp(sprintf('Axis %s: %s', 'XYZ'(axis_idx), csv_data.Properties.VariableNames{axis_data{axis_idx}}));
    end

    function set_axis_data_callback(axis_idx)
        set_axis_data(axis_idx);
    end


%     function plot_data(~, ~)
%         if any(cellfun(@isequal, selected_variables, {0, 0, 0}))
%             warndlg('Select data for all axes.', 'Warning');
%             return;
%         end
% 
%         x_data = loaded_data{:, selected_variables{1}};
%         y_data = loaded_data{:, selected_variables{2}};
%         z_data = loaded_data{:, selected_variables{3}};
%         color_options = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
%         marker_options = {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'};
% 
%         figure;
%         plot3(x_data, y_data, z_data, [color_options{color_popup.Value}, ...
%             marker_options{marker_popup.Value}], 'LineWidth', 1.5);
%         grid on;
%         xlabel(loaded_data.Properties.VariableNames{selected_variables{1}});
%         ylabel(loaded_data.Properties.VariableNames{selected_variables{2}});
%         zlabel(loaded_data.Properties.VariableNames{selected_variables{3}});
%     end
    function plot_data(~, ~)
%         if any(cellfun(@isequal, selected_variables, {0, 0, 0}))
%             warndlg('Select data for all axes.', 'Warning');
%             return;
%         end
        
        x_var = x_popup.Value - 1;
        y_var = y_popup.Value - 1;
        z_var = z_popup.Value - 1

        
%         x_data = loaded_data{:, selected_variables{1}};
%         y_data = loaded_data{:, selected_variables{2}};
%         z_data = loaded_data{:, selected_variables{3}};
%         color_options = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
%         marker_options = {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'};

        figure;
        hold on;
            
        if z_var == 0 % 2D plot
            x_data = loaded_data{:, selected_variables{1}};
            y_data = loaded_data{:, selected_variables{2}};
            color_options = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
            marker_options = {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'};
        
            plot(x_data, y_data, [color_options{color_popup.Value}, ...
                marker_options{marker_popup.Value}], 'LineWidth', 1.5);
            xlabel(loaded_data.Properties.VariableNames{selected_variables{1}});
            ylabel(loaded_data.Properties.VariableNames{selected_variables{2}});

        else % 3D plot
            x_data = loaded_data{:, selected_variables{1}};
            y_data = loaded_data{:, selected_variables{2}};
            z_data = loaded_data{:, selected_variables{3}};
            color_options = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
            marker_options = {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'};
        
            plot3(x_data, y_data, z_data, [color_options{color_popup.Value}, ...
                marker_options{marker_popup.Value}], 'LineWidth', 1.5);
            grid on;
            xlabel(loaded_data.Properties.VariableNames{selected_variables{1}});
            ylabel(loaded_data.Properties.VariableNames{selected_variables{2}});
            zlabel(loaded_data.Properties.VariableNames{selected_variables{3}});
        end

        grid on;
        hold off;
    end

    function plot_2D_empty(~, ~)
        figure;
        xlabel('X');
        ylabel('Y');
        grid on;
    end

    function plot_3D_empty(~, ~)
        figure;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        grid on;
        view(3);
    end

    function plot_2D(~, ~)
        if length(axis_data) < 2
            warndlg('Select at least two variables to plot.', 'Warning');
            return;
        end
        x_data = loaded_data{:, selected_variables{1}};
        y_data = loaded_data{:, selected_variables{2}};
        color_options = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
        marker_options = {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'};

        plot(x_data, y_data, [color_options{color_popup.Value}, ...
            marker_options{marker_popup.Value}], 'LineWidth', 1.5);
        xlabel(loaded_data.Properties.VariableNames{selected_variables{1}});
        ylabel(loaded_data.Properties.VariableNames{selected_variables{2}});
    end

    function plot_3D(~, ~)
        if length(axis_data) < 3
            warndlg('Select at least three variables to plot.', 'Warning');
            return;
        end
        x_data = loaded_data{:, selected_variables{1}};
        y_data = loaded_data{:, selected_variables{2}};
        z_data = loaded_data{:, selected_variables{3}};
        color_options = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
        marker_options = {'o', '+', '*', '.', 'x', 's', 'd', '^', 'v', '>', '<', 'p', 'h'};

        plot3(x_data, y_data, z_data, [color_options{color_popup.Value}, ...
            marker_options{marker_popup.Value}], 'LineWidth', 1.5);
        grid on;
        xlabel(loaded_data.Properties.VariableNames{selected_variables{1}});
        ylabel(loaded_data.Properties.VariableNames{selected_variables{2}});
        zlabel(loaded_data.Properties.VariableNames{selected_variables{3}});
    end

%     function plot_2D(~, ~)
%         if length(axis_data) < 2
%             warndlg('Select at least two variables to plot.', 'Warning');
%             return;
%         end
%         h = figure;
%         hold on;
%         % Add context menu
%         cm = uicontextmenu;
%         uimenu(cm, 'Text', 'Plot X and Y data', 'Callback', @plot_2D_data);
%         h.UIContextMenu = cm;
%     end
% 
%     function plot_3D(~, ~)
%         if length(axis_data) < 3
%             warndlg('Select at least three variables to plot.', 'Warning');
%             return;
%         end
%         h = figure;
%         hold on;
%         % Add context menu
%         cm = uicontextmenu;
%         uimenu(cm, 'Text', 'Plot X, Y, and Z data', 'Callback', @plot_3D_data);
%         h.UIContextMenu = cm;
%     end
end

