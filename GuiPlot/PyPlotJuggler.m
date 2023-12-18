classdef PyPlotJuggler < handle
    properties
        root
        figs
        data
        selected_fields
        time
        max_time
        args
        time_started
        field_table_label
        field_list
        slider_label_frame
        time_slider_label
        time_slider
        btn_frame
        new_fig_btn
        clear_list_btn
        load_file_btn
        start_time_btn
        stop_time_btn
        status_bar_str
        status_bar
        menubar
        idir
    end
    methods
        function obj = PyPlotJuggler(root, args)
            obj.root = root;
            obj.figs = {};
            obj.data = [];
            obj.selected_fields = {};
            obj.time = 0;
            obj.max_time = 200;
            obj.args = args;
            obj.time_started = false;
            obj.initialize_widgets();

            obj.root.Name = 'PyPlotJuggler';
            obj.root.CloseRequestFcn = @obj.on_closing;

            obj.setup_with_args();
        end

        % The remaining methods go here
        function initialize_widgets(obj)
            obj.setup_menubar();
            obj.setup_field_list();
            obj.setup_time_slider();
            obj.setup_buttons();
            obj.setup_status_bar();
        end

        function setup_menubar(obj)
            obj.menubar = uimenu(obj.root, 'Text', 'Action');
            uimenu(obj.menubar, 'Text', 'Load file', 'Callback', @obj.load_file);
            uimenu(obj.menubar, 'Text', 'Create figure', 'Callback', @obj.create_new_figure);
            uimenu(obj.menubar, 'Text', 'Clear selection', 'Callback', @obj.clear_selection);
            uimenu(obj.menubar, 'Text', 'Start time', 'Callback', @obj.start_time);
            uimenu(obj.menubar, 'Text', 'Stop time', 'Callback', @obj.stop_time);
            helpmenu = uimenu(obj.root, 'Text', 'Help');
            uimenu(helpmenu, 'Text', 'About', 'Callback', @obj.show_about_message);
        end
        
        function setup_field_list(obj)
            obj.field_table_label = uicontrol(obj.root, 'Style', 'text', 'String', 'Field table', ...
                'Units', 'normalized', 'Position', [0.05, 0.8, 0.2, 0.05]);

            obj.container = uipanel(obj.root, 'Units', 'normalized', 'Position', [0.05, 0.4, 0.9, 0.4]);
            obj.field_list = uitable(obj.container, 'Units', 'normalized', 'Position', [0, 0, 1, 1], ...
                'ColumnName', {'field name', 'value'}, 'CellSelectionCallback', @obj.on_select);
            obj.update_selection_list(0.0);
        end

        function setup_time_slider(obj)
            obj.slider_label_frame = uipanel(obj.root, 'Units', 'normalized', 'Position', [0.05, 0.3, 0.9, 0.05]);
            obj.time_slider_label = uicontrol(obj.slider_label_frame, 'Style', 'text', 'String', 'Time index slider:', ...
                'Units', 'normalized', 'Position', [0, 0.5, 0.3, 0.5]);
            
            obj.time_slider = uicontrol(obj.slider_label_frame, 'Style', 'slider', 'Min', 0, 'Max', obj.max_time, ...
                'Value', 0, 'Units', 'normalized', 'Position', [0.3, 0, 0.7, 1], ...
                'Callback', @obj.slider_changed);
        end
        
        function setup_buttons(obj)
            obj.btn_frame = uipanel(obj.root, 'Units', 'normalized', 'Position', [0.05, 0.2, 0.9, 0.1]);
            
            obj.new_fig_btn = uicontrol(obj.btn_frame, 'Style', 'pushbutton', 'String', 'Create figure', ...
                'Units', 'normalized', 'Position', [0, 0.5, 0.2, 0.5], 'Callback', @obj.create_new_figure);
            
            obj.clear_list_btn = uicontrol(obj.btn_frame, 'Style', 'pushbutton', 'String', 'Clear selection', ...
                'Units', 'normalized', 'Position', [0.2, 0.5, 0.2, 0.5], 'Callback', @obj.clear_selection);
            
            obj.load_file_btn = uicontrol(obj.btn_frame, 'Style', 'pushbutton', 'String', 'Load file', ...
                'Units', 'normalized', 'Position', [0.4, 0.5, 0.2, 0.5], 'Callback', @obj.load_file);
            
            obj.start_time_btn = uicontrol(obj.btn_frame, 'Style', 'pushbutton', 'String', 'Start time', ...
                'Units', 'normalized', 'Position', [0.6, 0.5, 0.2, 0.5], 'Callback', @obj.start_time);
            
            obj.stop_time_btn = uicontrol(obj.btn_frame, 'Style', 'pushbutton', 'String', 'Stop time', ...
                'Units', 'normalized', 'Position', [0.8, 0.5, 0.2, 0.5], 'Callback', @obj.stop_time);
        end

        function setup_status_bar(obj)
            obj.status_bar_str = '';
            obj.status_bar = uicontrol(obj.root, 'Style', 'text', 'String', obj.status_bar_str, ...
                'Units', 'normalized', 'Position', [0.05, 0.1, 0.9, 0.05]);
        end
        
                function on_select(obj, src, event)
            % �I�����ꂽ�t�B�[���h���X�V����
            obj.selected_fields = event.Indices(:, 1);
        end

        function slider_changed(obj, src, event)
            % ���Ԃ��X�V���A�I�����X�g�Ɛ}���X�V����
            obj.time = obj.time_slider.Value;
            obj.update_selection_list(obj.time);
            % �}�̍X�V�́AfigureManager�̃C���X�^���X���쐬���A������g�p���čs��
        end

        function create_new_figure(obj, src, event)
            % �V�����}���쐬����
            % ���̊֐��ł́AfigureManager�̃C���X�^���X���쐬���A������g�p���Đ}���쐬����
        end

        function clear_selection(obj, src, event)
            % ���X�g�̑I�����N���A����
            obj.field_list.Value = [];
        end

        function load_file(obj, src, event)
            % �t�@�C����ǂݍ��݁A�f�[�^��ݒ肷��
            % �����ł́Auigetfile���g�p���ăt�@�C����I�����A�f�[�^��ǂݍ���
            % �ǂݍ��񂾃f�[�^���g�p���āA�}��I�����X�g���X�V����
        end

        function start_time(obj, src, event)
            % ���Ԃ̐i�s���J�n����
            if obj.time_started
                return;
            end
            obj.time_started = true;
            obj.proceed_time();
        end

        function stop_time(obj, src, event)
            % ���Ԃ̐i�s���~����
            obj.time_started = false;
        end
        
    end
end
