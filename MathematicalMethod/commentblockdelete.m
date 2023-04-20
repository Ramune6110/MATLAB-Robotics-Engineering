clear;
close all;
clc;

% �܂�GUI���쐬���܂�
f = figure('Name', 'Simulink Comment Browser', 'NumberTitle', 'off', 'Position', [300, 150, 600, 370]);

uicontrol('Style', 'pushbutton', 'String', 'Open Simulink Model', 'Position', [40, 300, 150, 25], 'Callback', @openSimulinkModel);
uicontrol('Style', 'text', 'String', '�R�����g�A�E�g���ꂽ�u���b�N/�T�u�V�X�e�����ȉ��ɕ\��', 'Position', [35, 335, 250, 20]);

global tgtbox;
global dlt_next;
global just_next;
global commentBlocks;
tgtbox = uicontrol('Style', 'listbox', 'String', commentBlocks, 'Position', [35, 45, 525, 250], 'Callback', @go_to_commented_out);
dlt_next  = uicontrol('Visible', 'on', 'Style', 'pushbutton', 'String', 'Delete Selected Block', 'Position', [220, 300, 150, 25], 'Callback', @delete_block_and_next_block);
just_next = uicontrol('Visible', 'on', 'Style', 'pushbutton', 'String', 'Go to Next Block', 'Position', [400, 300, 150, 25], 'Callback', @go_next_block);

global prvco;
prvco = 0;

global nocomsg;
nocomsg = '** �R�����g�A�E�g����Ă���u���b�N/�T�u�V�X�e���͂���܂���B **';

% �u���b�N�̋���/���������̐ݒ�
set_param(0, 'HiliteAncestorsData',... 
                struct('HiliteType', 'user1', ... 
                       'ForegroundColor', 'darkGreen', ... 
                       'BackgroundColor', 'lightBlue'));
set_param(0, 'HiliteAncestorsData', ... 
                struct('HiliteType', 'default', ... 
                       'ForegroundColor', 'black', ... 
                       'BackgroundColor', 'white'));
                   
% 1. Simulink���f�����J���܂�
function openSimulinkModel(~, ~)
    [filename, pathname] = uigetfile('*.slx', 'Select a Simulink model');
    if ~isequal(filename, 0)
        global modelPath;
        global SimlinkName;
        
        modelPath = fullfile(pathname, filename);
        open_system(modelPath);

        L = strlength(filename);
        extension_length = 4;
        temp_str = '';

        for i = 1:L - extension_length
            temp_str = append(temp_str,filename(i));
        end

        SimlinkName = temp_str;

        % 2. �R�����g�A�E�g�u���b�N�̃��X�g���쐬���܂�
        updateBlockList();
    end
end

% 2. �R�����g�A�E�g�u���b�N�̃��X�g���X�V���܂�
function updateBlockList()
    global modelPath;
%     global blockList;
    global commentBlocks;
    global SimlinkName;

    % �R�����g�A�E�g���ꂽ�u���b�N/�T�u�V�X�e�����擾
    commentBlocks = find_system(SimlinkName, 'IncludeCommented', 'on', 'Commented', 'on')
    
    if isempty(commentBlocks) % �R�����g�A�E�g���P���Ȃ��ꍇ
        commentBlocks = nocomsg;
    end
end

% ���X�g��̃u���b�N/�T�u�V�X�e�����N���b�N���A���̃u���b�N/�T�u�V�X�e������������
function go_to_commented_out(source, command)
    global prvco;
    global nocomsg;
    global dlt_next;
    global just_next;
    
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

% ���݋������Ă���R�����g�A�E�g���폜���Ȃ��Ŏ��̃R�����g�A�E�g������
function go_next_block(~, ~)
    global tgtbox;
    
    % ���X�g�̈�ԉ��ȊO�̃R�����g�A�E�g���������Ă��鎞
    if tgtbox.Value < size(tgtbox.String, 1)

        tgtbox.Value = tgtbox.Value + 1; % ���X�g��̎��̃u���b�N/�T�u�V�X�e��������
    else % ��ԉ����������Ă��鎞
        tgtbox.Value = 1; % ���X�g�̍ŏ��ɖ߂�
    end

    go_to_commented_out(tgtbox, 'go_next'); % ���̃u���b�N/�T�u�V�X�e��������
end

% ���݋������Ă���R�����g�A�E�g���폜���Ď��̃R�����g�A�E�g������
function delete_block_and_next_block(~, ~)
    global tgtbox;
        
    % ���X�g�̈�ԉ��ȊO�̃R�����g�A�E�g���������Ă��鎞
    if tgtbox.Value < size(tgtbox.String, 1)

        curco = tgtbox.Value; % ���X�g��ŋ������̃u���b�N/�T�u�V�X�e���ԍ���ۑ�
    else  % ��ԉ����������Ă��鎞
        curco = 1;
        tgtbox.Value = curco; % ���X�g�̍ŏ��ɖ߂�
    end

    delete_block(tgtbox.String(tgtbox.Value));

    % �R�����g�A�E�g���ꂽ�u���b�N/�T�u�V�X�e�����Ď擾�i�폜�����R�����g�A�E�g�����X�g���珜�����߁j
    list = find_system('test_co', 'IncludeCommented', 'on', 'Commented', 'on');

    if isempty(list) % �R�����g�A�E�g���P���Ȃ��ꍇ

        list = nocomsg;
    end
    tgtbox.String = list;    
    tgtbox.Value = curco;

    go_to_commented_out(tgtbox, 'delete'); % ���̃u���b�N/�T�u�V�X�e��������
end
