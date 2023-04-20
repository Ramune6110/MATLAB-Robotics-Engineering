% まずGUIを作成します
f = figure('Name', 'Simulink Comment Browser', 'NumberTitle', 'off', 'Position', [100, 100, 400, 350]);
                       
uicontrol('Style', 'pushbutton', 'String', 'Open Simulink Model', 'Position', [20, 300, 150, 30], 'Callback', @openSimulinkModel);
uicontrol('Style', 'pushbutton', 'String', 'Delete Selected Block', 'Position', [200, 300, 150, 30], 'Callback', @deleteSelectedBlock);
uicontrol('Style', 'pushbutton', 'String', 'Go to Next Block', 'Position', [200, 250, 150, 30], 'Callback', @goToNextBlock);

global blockList;
blockList = uicontrol('Style', 'listbox', 'Position', [20, 20, 360, 250], 'Callback', @selectBlock);

% 1. Simulinkモデルを開きます
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

        % 2. コメントアウトブロックのリストを作成します
        updateBlockList();
    end
end

% 2. コメントアウトブロックのリストを更新します
function updateBlockList()
    global modelPath;
    global blockList;
    global commentBlocks;
    global SimlinkName;
%         commentBlocks = find_system(SimlinkName, 'BlockType', 'SubSystem', 'Commented', 'on');

    % コメントアウトされたブロック/サブシステムを取得
    commentBlocks = find_system(SimlinkName, 'IncludeCommented', 'on', 'Commented', 'on');
    
    nocomsg = '** コメントアウトされているブロック/サブシステムはありません。 **';
    if isempty(commentBlocks) % コメントアウトが１つもない場合
        commentBlocks = nocomsg;
    end

    set(blockList, 'String', commentBlocks);
end

% 3. 選択したブロックにフォーカスを当てます
function selectBlock(src, ~)
    global commentBlocks;
    index = get(src, 'Value');
    if ~isempty(commentBlocks) && index <= numel(commentBlocks)
        open_system(commentBlocks{index});
    end
end

% 5. 選択したブロックを削除します
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

% 6. 次のブロックに移動します
function goToNextBlock(~, ~)
    global blockList;
    global commentBlocks;
    index = get(blockList, 'Value');
    if ~isempty(commentBlocks) && index < numel(commentBlocks)
        set(blockList, 'Value', index + 1);
        selectBlock(blockList);
    end
end
