function custom_plotter
    % Create the main figure
    main_fig = figure('Name', 'Custom Plotter', 'Position', [100, 100, 800, 600]);

    % Load CSV button
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Load CSV', ...
        'Position', [10, 550, 100, 30], 'Callback', @load_csv);

    % Variables table
    data_table = uitable(main_fig, 'ColumnName', {'Variable', 'Value'}, ...
        'ColumnWidth', {100, 100}, 'Data', cell(0, 2), 'Position', [10, 50, 200, 450], ...
        'CellSelectionCallback', @select_variable);

    % Set axis buttons and variable name displays
    for i = 1:3
        uicontrol('Parent', main_fig, 'Style', 'pushbutton', ...
            'String', sprintf('%c Data Set', 'XYZ'(i)), ...
            'Position', [220, 550 - 40 * (i - 1), 100, 30], ...
            'Callback', {@set_axis_data, i});
        axis_var_text(i) = uicontrol('Parent', main_fig, 'Style', 'text', ...
            'String', '', 'Position', [330, 550 - 40 * (i - 1), 100, 30]);
    end

    % Plot button
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot', ...
        'Position', [450, 550, 100, 30], 'Callback', @plot_data);

    % Variables
    csv_data = [];
    selected_variables = {};
    axis_data = cell(1, 3);

    % Callbacks
    % (load_csv and select_variable remain unchanged)

    function set_axis_data(~, ~, axis_idx)
        if isempty(selected_variables)
            warndlg(sprintf('データを選択してください。'), 'Warning');
            return;
        end
        axis_data{axis_idx} = selected_variables(1);
        axis_var_text(axis_idx).String = csv_data.Properties.VariableNames{axis_data{axis_idx}};
    end

    function plot_data(~, ~)
        if isempty(axis_data{1}) || isempty(axis_data{2})
            warndlg('XデータとYデータを選択してください。', 'Warning');
            return;
        end

        figure;
        hold on;

        if isempty(axis_data{3}) % 2D plot
            plot(csv_data{:, axis_data{1}}, csv_data{:, axis_data{2}});
            xlabel(csv_data.Properties.VariableNames{axis_data{1}});
            ylabel(csv_data.Properties.VariableNames{axis_data{2}});
        else % 3D plot
            scatter3(csv_data{:, axis_data{1}}, ...
                csv_data{:, axis_data{2}}, ...
                csv_data{:, axis_data{3}});
            xlabel(csv_data.Properties.VariableNames{axis_data{1}});
            ylabel(csv_data.Properties.VariableNames
            ylabel(csv_data.Properties.VariableNames{axis_data{2}});
            zlabel(csv_data.Properties.VariableNames{axis_data{3}});
        end

        grid on;
        hold off;
    end
end
