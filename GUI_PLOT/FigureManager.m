classdef FigureManager < handle
    properties
        fig
        ax
        parent
        x
        x_field_names
        y
        y_field_names
        fig_num
    end
    methods
        function obj = FigureManager(parent, fnum)
            [obj.fig, obj.ax] = deal(figure, axes);
            obj.parent = parent;
            obj.x = {};
            obj.x_field_names = {};
            obj.y = {};
            obj.y_field_names = {};
            obj.fig_num = fnum;

            set(obj.fig, 'WindowButtonDownFcn', @obj.on_click);
            grid(obj.ax, 'on');
            pause(PLOT_DT);
        end

        function on_click(obj, ~, ~)
            if isempty(obj.parent.selected_fields)
                obj.parent.status_bar_str = 'Please click field';
            else
                obj.set_data(obj.parent.data, obj.parent.selected_fields);
                pause(PLOT_DT);
            end
            obj.parent.clear_selection();
            obj.parent.selected_fields = {};
        end

        function set_data(obj, data, field_names)
            if isempty(obj.x)
                for i = 1:length(field_names)
                    fn = field_names{i};
                    obj.x{end+1} = data.(fn);
                    obj.x_field_names{end+1} = fn;
                end
            elseif isempty(obj.y)
                for i = 1:length(field_names)
                    fn = field_names{i};
                    obj.y{end+1} = data.(fn);
                    obj.y_field_names{end+1} = fn;
                end
            end
            obj.plot();
        end

        function plot(obj)
            cla(obj.ax);

            if isempty(obj.y)
                for i = 1:length(obj.x)
                    time = 1:length(obj.x{i});
                    plot(obj.ax, time, obj.x{i}, 'DisplayName', obj.x_field_names{i});
                end
                xlabel(obj.ax, 'Time index');
                legend(obj.ax);
            elseif length(obj.x) == 1
                plot(obj.ax, obj.x{1}, obj.y{1}, '-r');
                xlabel(obj.ax, obj.x_field_names{1});
                ylabel(obj.ax, obj.y_field_names{1});
            end
            grid(obj.ax, 'on');
        end

        function plot_time_line(obj, time)
            obj.plot();
            if isempty(obj.y)
                axvline(obj.ax, time);

                for i = 1:length(obj.x)
                    plot(obj.ax, time, obj.x{i}(time), 'xk');
                    text(obj.ax, time, obj.x{i}(time), ...
                         [num2str(obj.x{i}(time), '%.3f'), ':', obj.x_field_names{i}]);
                end
            else
                plot(obj.ax, obj.x{1}(time), obj.y{1}(time), 'xk');
            end
        end
    end
end
