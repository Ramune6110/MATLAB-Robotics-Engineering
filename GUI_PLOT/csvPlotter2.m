function csvPlotter2
    % ���C���E�B���h�E���쐬
    main_fig = figure('Name', 'CSV Plotter', 'Position', [100, 100, 800, 600]);

    % CSV�t�@�C����ǂݍ��ރ{�^��
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Load CSV', ...
        'Position', [10, 550, 100, 30], 'Callback', @load_csv);

    % �ϐ��e�[�u��
    data_table = uitable(main_fig, 'ColumnName', {'Variable', 'Value', 'Min', 'Max', 'Mean'}, ...
        'ColumnWidth', {100, 100, 100, 100, 100}, 'Data', cell(0, 5), 'Position', [10, 50, 500, 450], ...
        'CellSelectionCallback', @select_variable);

    % 2D�v���b�g�{�^��
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot 2D', ...
        'Position', [520, 550, 100, 30], 'Callback', @plot_2D);
    % 3D�v���b�g�{�^��
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Plot 3D', ...
        'Position', [520, 510, 100, 30], 'Callback', @plot_3D);

    % �ϐ�
    csv_data = [];
    selected_variables = {};

    % �R�[���o�b�N�֐�
    function load_csv(~, ~)
        [filename, pathname] = uigetfile('*.csv', 'CSV�t�@�C����I��');
        if isequal(filename, 0)
            return;
        end
        csv_data = readtable(fullfile(pathname, filename));
%         data_table.Data = [csv_data.Properties.VariableNames', ...
%             cellstr(num2str(csv_data{1, :}')), ...
%             num2cell(min(csv_data.Variables)), ...
%             num2cell(max(csv_data.Variables)), ...
%             num2cell(mean(csv_data.Variables))];
        
        data_table.Data = [csv_data.Properties.VariableNames',...
                           cellstr(num2str(csv_data{1, :}')),...
                           num2cell(min(csv_data.Variables))];

    end
    
%     function load_csv(~, ~)
%         [filename, pathname] = uigetfile('*.csv', 'Select CSV file');
%         if isequal(filename, 0)
%             return;
%         end
%         csv_data = readtable(fullfile(pathname, filename));
%         data_table.Data = [csv_data.Properties.VariableNames', ...
%             repmat({''}, length(csv_data.Properties.VariableNames), 1)];
%     end

    function select_variable(~, eventdata)
        if isempty(eventdata.Indices)
            return;
        end
        selected_variables = eventdata.Indices(:, 1)';
    end

    function plot_2D(~, ~)
        if length(selected_variables) < 2
            warndlg('2�ȏ�̕ϐ���I�����ăv���b�g���Ă��������B', '�x��');
            return;
        end

        figure;
        hold on;
        plot(csv_data{:, selected_variables(1)}, csv_data{:, selected_variables(2)});
        grid on;
        hold off;
        legend(csv_data.Properties.VariableNames(selected_variables));
    end

    function plot_3D(~, ~)
        if length(selected_variables) < 3
            warndlg('3�ȏ�̕ϐ���I�����ăv���b�g���Ă��������B', '�x��');
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
