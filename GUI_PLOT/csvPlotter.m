function csvPlotter
    % Create the main figure
    main_fig = figure('Name', 'CSV Plotter', 'Position', [100, 100, 800, 600]);

    % Load CSV button
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Load CSV', ...
        'Position', [10, 550, 100, 30], 'Callback', @load_csv);

    % Variables table
    data_table = uitable(main_fig, 'ColumnName', {'Variable', 'Value'}, ...
        'ColumnWidth', {100, 100}, 'Data', cell(0, 2), 'Position', [10, 50, 200, 450], ...
        'CellSelectionCallback', @select_variable);

    % Plot button
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot 2D', ...
        'Position', [220, 550, 100, 30], 'Callback', @plot_2D);
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot 3D', ...
        'Position', [220, 510, 100, 30], 'Callback', @plot_3D);

    % Variables
    csv_data = [];
    selected_variables = {};

    % Callbacks
    function load_csv(~, ~)
        [filename, pathname] = uigetfile('*.csv', 'Select CSV file');
        if isequal(filename, 0)
            return;
        end
        csv_data = readtable(fullfile(pathname, filename));
        data_table.Data = [csv_data.Properties.VariableNames', ...
            repmat({''}, length(csv_data.Properties.VariableNames), 1)];
    end

    function select_variable(~, eventdata)
        if isempty(eventdata.Indices)
            return;
        end
        selected_variables = eventdata.Indices(:, 1)';
    end

    function plot_2D(~, ~)
        if length(selected_variables) < 2
            warndlg('Select at least two variables to plot.', 'Warning');
            return;
        end

        figure;
        hold on;
        plot(csv_data{:, selected_variables(1)}, csv_data{:, selected_variables(2)});
%         for i = 1:length(selected_variables)
%             plot(csv_data{:, selected_variables(i)});
%         end
        grid on;
        hold off;
        legend(csv_data.Properties.VariableNames(selected_variables));
    end

    function plot_3D(~, ~)
        if length(selected_variables) < 3
            warndlg('Select at least three variables to plot.', 'Warning');
            return;
        end

        figure;
        scatter3(csv_data{:, selected_variables(1)}, ...
            csv_data{:, selected_variables(2)}, ...
            csv_data{:, selected_variables(3)});
        grid on;
        xlabel(csv_data.Properties.VariableNames{selected_variables(1)});
        ylabel(csv_data.Properties.VariableNames{selected_variables(2)});
        zlabel(csv_data.Properties.VariableNames{selected_variables(3)});
    end
end