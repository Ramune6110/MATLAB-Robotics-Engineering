
function delete_comment_out2
    main_fig = figure('Name', 'コメントアウト削除ツール', 'Position', [300, 150, 600, 370]);
    
    % Load CSV button
%     model_name;
    uicontrol('Parent', main_fig, 'Style', 'pushbutton', 'String', 'Load CSV', ...
        'Position', [10, 550, 100, 30], 'Callback', @load_csv);
    
    prvco = 0; % 前回強調していたブロック/サブシステムの番号
    nocomsg = '** コメントアウトされているブロック/サブシステムはありません。 **';
    
    %Get all commented blocks/subsystems
    list = find_system(model_name, 'IncludeCommented', 'on', 'Commented', 'on');

    if isempty(list) % コメントアウトが１つもない場合
        list = nocomsg;
    end

    % ブロックの強調/強調解除の設定
    set_param(0, 'HiliteAncestorsData',... 
                    struct('HiliteType', 'user1', ... 
                           'ForegroundColor', 'darkGreen', ... 
                           'BackgroundColor', 'lightBlue'));
    set_param(0, 'HiliteAncestorsData', ... 
                    struct('HiliteType', 'default', ... 
                           'ForegroundColor', 'black', ... 
                           'BackgroundColor', 'white'));

    uicontrol('Style', 'text', 'String', 'コメントアウトされたブロック/サブシステムを以下に表示', 'Position', [35, 335, 250, 20]);
%     tgtbox = uicontrol('Style', 'listbox', 'String', list, 'Position', [35, 45, 525, 250], 'Callback', @go_to_commented_out);
    dlt_next  = uicontrol('Visible', 'off', 'Style', 'pushbutton', 'String', '削除して次を検索', 'Position', [35, 305, 110, 25], 'Callback', @delete_block_and_next_block);
    just_next = uicontrol('Visible', 'off', 'Style', 'pushbutton', 'String', '削除せずに次を検索', 'Position', [170, 305, 110, 25], 'Callback', @go_next_block);
    
    % リスト上のブロック/サブシステムをクリック時、そのブロック/サブシステムを強調する
    function go_to_commented_out(source, command)

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

    % 現在強調しているコメントアウトを削除して次のコメントアウトを強調
    function delete_block_and_next_block(~, ~)

        % リストの一番下以外のコメントアウトを強調している時
        if tgtbox.Value < size(tgtbox.String, 1)

            curco = tgtbox.Value; % リスト上で強調中のブロック/サブシステム番号を保存
        else  % 一番下を強調している時
            curco = 1;
            tgtbox.Value = curco; % リストの最初に戻る
        end

        delete_block(tgtbox.String(tgtbox.Value));

        % コメントアウトされたブロック/サブシステムを再取得（削除したコメントアウトをリストから除くため）
        list = find_system(model_name, 'IncludeCommented', 'on', 'Commented', 'on');

        if isempty(list) % コメントアウトが１つもない場合

            list = nocomsg;
        end
        tgtbox.String = list;    
        tgtbox.Value = curco;

        go_to_commented_out(tgtbox, 'delete'); % 次のブロック/サブシステムを強調
    end

    % 現在強調しているコメントアウトを削除しないで次のコメントアウトを強調
    function go_next_block(~, ~)

        % リストの一番下以外のコメントアウトを強調している時
        if tgtbox.Value < size(tgtbox.String, 1)

            tgtbox.Value = tgtbox.Value + 1; % リスト上の次のブロック/サブシステムを強調
        else % 一番下を強調している時
            tgtbox.Value = 1; % リストの最初に戻る
        end

        go_to_commented_out(tgtbox, 'go_next'); % 次のブロック/サブシステムを強調
    end
    
    % Callbacks
    function load_csv(~, ~)
        [file, path] = uigetfile('*.slx', 'Select CSV file');
        model_name = fullfile(path, file);
        
        % Load the chosen Simulink model
        load_system(model_name);
    end
end
