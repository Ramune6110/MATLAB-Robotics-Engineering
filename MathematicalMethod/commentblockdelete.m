clear;
close all;
clc;

% まずGUIを作成します
f = figure('Name', 'Simulink Comment Browser', 'NumberTitle', 'off', 'Position', [300, 150, 600, 370]);

uicontrol('Style', 'pushbutton', 'String', 'Open Simulink Model', 'Position', [40, 300, 150, 25], 'Callback', @openSimulinkModel);
uicontrol('Style', 'text', 'String', 'コメントアウトされたブロック/サブシステムを以下に表示', 'Position', [35, 335, 250, 20]);

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
nocomsg = '** コメントアウトされているブロック/サブシステムはありません。 **';

% ブロックの強調/強調解除の設定
set_param(0, 'HiliteAncestorsData',... 
                struct('HiliteType', 'user1', ... 
                       'ForegroundColor', 'darkGreen', ... 
                       'BackgroundColor', 'lightBlue'));
set_param(0, 'HiliteAncestorsData', ... 
                struct('HiliteType', 'default', ... 
                       'ForegroundColor', 'black', ... 
                       'BackgroundColor', 'white'));
                   
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
%     global blockList;
    global commentBlocks;
    global SimlinkName;

    % コメントアウトされたブロック/サブシステムを取得
    commentBlocks = find_system(SimlinkName, 'IncludeCommented', 'on', 'Commented', 'on')
    
    if isempty(commentBlocks) % コメントアウトが１つもない場合
        commentBlocks = nocomsg;
    end
end

% リスト上のブロック/サブシステムをクリック時、そのブロック/サブシステムを強調する
function go_to_commented_out(source, command)
    global prvco;
    global nocomsg;
    global dlt_next;
    global just_next;
    
    if ~strcmp(source.String, nocomsg) % コメントアウトが存在する場合
        if prvco ~= 0 && ~strcmp(command, 'delete') 
            % 前回強調したブロックを、元の状態に戻す
            hilite_system(source.String(prvco), 'default');
        else
            % 特定のブロック/サブシステムが強調状態になって初めて、削除ボタンなどを表示する
            dlt_next.Visible  = 'on';
            just_next.Visible = 'on';
        end

        % ブロックを強調
        hilite_system(source.String(source.Value), 'user1');

        prvco = source.Value; % 今回強調したブロックを、前回値として保存
    end
end

% 現在強調しているコメントアウトを削除しないで次のコメントアウトを強調
function go_next_block(~, ~)
    global tgtbox;
    
    % リストの一番下以外のコメントアウトを強調している時
    if tgtbox.Value < size(tgtbox.String, 1)

        tgtbox.Value = tgtbox.Value + 1; % リスト上の次のブロック/サブシステムを強調
    else % 一番下を強調している時
        tgtbox.Value = 1; % リストの最初に戻る
    end

    go_to_commented_out(tgtbox, 'go_next'); % 次のブロック/サブシステムを強調
end

% 現在強調しているコメントアウトを削除して次のコメントアウトを強調
function delete_block_and_next_block(~, ~)
    global tgtbox;
        
    % リストの一番下以外のコメントアウトを強調している時
    if tgtbox.Value < size(tgtbox.String, 1)

        curco = tgtbox.Value; % リスト上で強調中のブロック/サブシステム番号を保存
    else  % 一番下を強調している時
        curco = 1;
        tgtbox.Value = curco; % リストの最初に戻る
    end

    delete_block(tgtbox.String(tgtbox.Value));

    % コメントアウトされたブロック/サブシステムを再取得（削除したコメントアウトをリストから除くため）
    list = find_system('test_co', 'IncludeCommented', 'on', 'Commented', 'on');

    if isempty(list) % コメントアウトが１つもない場合

        list = nocomsg;
    end
    tgtbox.String = list;    
    tgtbox.Value = curco;

    go_to_commented_out(tgtbox, 'delete'); % 次のブロック/サブシステムを強調
end
