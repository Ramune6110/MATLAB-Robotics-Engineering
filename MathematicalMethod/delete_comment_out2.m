
function delete_comment_out2
    main_fig = figure('Name', '�R�����g�A�E�g�폜�c�[��', 'Position', [300, 150, 600, 370]);
    
    % Load CSV button
%     model_name;
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Load CSV', ...
        'Position', [10, 550, 100, 30], 'Callback', @load_csv);
    
    prvco = 0; % �O�񋭒����Ă����u���b�N/�T�u�V�X�e���̔ԍ�
    nocomsg = '** �R�����g�A�E�g����Ă���u���b�N/�T�u�V�X�e���͂���܂���B **';
    
    %Get all commented blocks/subsystems
    list = find_system(model_name, 'IncludeCommented', 'on', 'Commented', 'on');

    if isempty(list) % �R�����g�A�E�g���P���Ȃ��ꍇ
        list = nocomsg;
    end

    % �u���b�N�̋���/���������̐ݒ�
    set_param(0, 'HiliteAncestorsData',... 
                    struct('HiliteType', 'user1', ... 
                           'ForegroundColor', 'darkGreen', ... 
                           'BackgroundColor', 'lightBlue'));
    set_param(0, 'HiliteAncestorsData', ... 
                    struct('HiliteType', 'default', ... 
                           'ForegroundColor', 'black', ... 
                           'BackgroundColor', 'white'));

    uicontrol('Style', 'text', 'String', '�R�����g�A�E�g���ꂽ�u���b�N/�T�u�V�X�e�����ȉ��ɕ\��', 'Position', [35, 335, 250, 20]);
%     tgtbox = uicontrol('Style', 'listbox', 'String', list, 'Position', [35, 45, 525, 250], 'Callback', @go_to_commented_out);
    dlt_next  = uicontrol('Visible', 'off', 'Style', 'pushbutton', 'String', '�폜���Ď�������', 'Position', [35, 305, 110, 25], 'Callback', @delete_block_and_next_block);
    just_next = uicontrol('Visible', 'off', 'Style', 'pushbutton', 'String', '�폜�����Ɏ�������', 'Position', [170, 305, 110, 25], 'Callback', @go_next_block);
    
    % ���X�g��̃u���b�N/�T�u�V�X�e�����N���b�N���A���̃u���b�N/�T�u�V�X�e������������
    function go_to_commented_out(source, command)

        if ~strcmp(source.String, nocomsg) % �R�����g�A�E�g�����݂���ꍇ
            if prvco ~= 0 && ~strcmp(command, 'delete') 
                % �O�񋭒������u���b�N���A���̏�Ԃɖ߂�
                hilite_system(source.String(prvco), 'default');
            else
                % ����̃u���b�N/�T�u�V�X�e����������ԂɂȂ��ď��߂āA�폜�{�^���Ȃǂ�\������
                dlt_next.Visible  = 'on';
                just_next.Visible = 'on';
            end

            % �u���b�N������
            hilite_system(source.String(source.Value), 'user1');

            prvco = source.Value; % ���񋭒������u���b�N���A�O��l�Ƃ��ĕۑ�
        end
    end

    % ���݋������Ă���R�����g�A�E�g���폜���Ď��̃R�����g�A�E�g������
    function delete_block_and_next_block(~, ~)

        % ���X�g�̈�ԉ��ȊO�̃R�����g�A�E�g���������Ă��鎞
        if tgtbox.Value < size(tgtbox.String, 1)

            curco = tgtbox.Value; % ���X�g��ŋ������̃u���b�N/�T�u�V�X�e���ԍ���ۑ�
        else  % ��ԉ����������Ă��鎞
            curco = 1;
            tgtbox.Value = curco; % ���X�g�̍ŏ��ɖ߂�
        end

        delete_block(tgtbox.String(tgtbox.Value));

        % �R�����g�A�E�g���ꂽ�u���b�N/�T�u�V�X�e�����Ď擾�i�폜�����R�����g�A�E�g�����X�g���珜�����߁j
        list = find_system(model_name, 'IncludeCommented', 'on', 'Commented', 'on');

        if isempty(list) % �R�����g�A�E�g���P���Ȃ��ꍇ

            list = nocomsg;
        end
        tgtbox.String = list;    
        tgtbox.Value = curco;

        go_to_commented_out(tgtbox, 'delete'); % ���̃u���b�N/�T�u�V�X�e��������
    end

    % ���݋������Ă���R�����g�A�E�g���폜���Ȃ��Ŏ��̃R�����g�A�E�g������
    function go_next_block(~, ~)

        % ���X�g�̈�ԉ��ȊO�̃R�����g�A�E�g���������Ă��鎞
        if tgtbox.Value < size(tgtbox.String, 1)

            tgtbox.Value = tgtbox.Value + 1; % ���X�g��̎��̃u���b�N/�T�u�V�X�e��������
        else % ��ԉ����������Ă��鎞
            tgtbox.Value = 1; % ���X�g�̍ŏ��ɖ߂�
        end

        go_to_commented_out(tgtbox, 'go_next'); % ���̃u���b�N/�T�u�V�X�e��������
    end
    
    % Callbacks
    function load_csv(~, ~)
        [file, path] = uigetfile('*.slx', 'Select CSV file');
        model_name = fullfile(path, file);
        
        % Load the chosen Simulink model
        load_system(model_name);
    end
end
