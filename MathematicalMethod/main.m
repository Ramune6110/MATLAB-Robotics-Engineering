% �܂�GUI���쐬���܂�
f = figure('Name', 'Simulink Comment Browser', 'NumberTitle', 'off', 'Position', [100, 100, 400, 350]);
                       
uicontrol('Style', 'pushbutton', 'String', 'Open Simulink Model', 'Position', [20, 300, 150, 30], 'Callback', @openSimulinkModel);
uicontrol('Style', 'pushbutton', 'String', 'Delete Selected Block', 'Position', [200, 300, 150, 30], 'Callback', @deleteSelectedBlock);
uicontrol('Style', 'pushbutton', 'String', 'Go to Next Block', 'Position', [200, 250, 150, 30], 'Callback', @goToNextBlock);

global blockList;
blockList = uicontrol('Style', 'listbox', 'Position', [20, 20, 360, 250], 'Callback', @selectBlock);

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
    global blockList;
    global commentBlocks;
    global SimlinkName;
%         commentBlocks = find_system(SimlinkName, 'BlockType', 'SubSystem', 'Commented', 'on');

    % �R�����g�A�E�g���ꂽ�u���b�N/�T�u�V�X�e�����擾
    commentBlocks = find_system(SimlinkName, 'IncludeCommented', 'on', 'Commented', 'on');
    
    nocomsg = '** �R�����g�A�E�g����Ă���u���b�N/�T�u�V�X�e���͂���܂���B **';
    if isempty(commentBlocks) % �R�����g�A�E�g���P���Ȃ��ꍇ
        commentBlocks = nocomsg;
    end

    set(blockList, 'String', commentBlocks);
end

% 3. �I�������u���b�N�Ƀt�H�[�J�X�𓖂Ă܂�
function selectBlock(src, ~)
    global commentBlocks;
    index = get(src, 'Value');
    if ~isempty(commentBlocks) && index <= numel(commentBlocks)
        open_system(commentBlocks{index});
    end
end

% 5. �I�������u���b�N���폜���܂�
function deleteSelectedBlock(~, ~)
    global blockList;
    global commentBlocks;
    index = get(blockList, 'Value');
    if ~isempty(commentBlocks) && index <= numel(commentBlocks)
        delete_block(commentBlocks{index});
        updateBlockList();
        goToNextBlock();
    end
end

% 6. ���̃u���b�N�Ɉړ����܂�
function goToNextBlock(~, ~)
    global blockList;
    global commentBlocks;
    index = get(blockList, 'Value');
    if ~isempty(commentBlocks) && index < numel(commentBlocks)
        set(blockList, 'Value', index + 1);
        selectBlock(blockList);
    end
end
