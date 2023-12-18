function simple_data_viewer()
% �E�B���h�E���쐬
main_fig = figure('Name', '�V���v���f�[�^�r���[�A', 'NumberTitle', 'off', ...
    'MenuBar', 'none', 'ToolBar', 'none', 'Position', [100, 100, 600, 400]);

% �{�^�����쐬
load_button = uicontrol(main_fig, 'Style', 'pushbutton', 'String', '�t�@�C�����J��', ...
    'Position', [10, 360, 100, 30], 'Callback', @load_file);

% �f�[�^�\���p�̕\���쐬
data_table = uitable(main_fig, 'ColumnName', {'�ϐ���', '�ŏ��l', '�ő�l', '���ϒl'}, ...
    'Position', [10, 10, 580, 340], 'ColumnWidth', {100, 150, 150, 150});

% �f�[�^���i�[����ϐ�
loaded_data = [];

    function load_file(~, ~)
        % �t�@�C����I��
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

        % �\�Ƀf�[�^��\��
        vars = loaded_data.Properties.VariableNames';
        data_table.Data = [vars, ...
            num2cell(min(loaded_data.Variables)), ...
            num2cell(max(loaded_data.Variables)), ...
            num2cell(mean(loaded_data.Variables))];
    end
end