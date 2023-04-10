classdef CubicSpline1D < handle
    properties
        a
        b
        c
        d
        x
        y
        nx
    end

    methods (Access = public)
        function obj = CubicSpline1D(x, y)
            h = diff(x);
            if any(h < 0)
                error('x coordinates must be sorted in ascending order');
            end

            obj.a = y;
            obj.b = zeros(1, numel(y));
            obj.c = zeros(1, numel(y));
            obj.d = zeros(1, numel(y));
            obj.x = x;
            obj.y = y;
            obj.nx = numel(x);

            A = obj.calc_A(h);
            B = obj.calc_B(h);
            obj.c = A \ B';

            for i = 1:(obj.nx - 1)
                d = (obj.c(i + 1) - obj.c(i)) / (3 * h(i));
                b = 1 / h(i) * (obj.a(i + 1) - obj.a(i)) ...
                    - h(i) / 3 * (2 * obj.c(i) + obj.c(i + 1));
                obj.d(i) = d;
                obj.b(i) = b;
            end
        end

        function position = calc_position(obj, x)
            if x < obj.x(1) || x > obj.x(end)
                position = nan;
                return;
            end

            i = obj.search_index(x);
            dx = x - obj.x(i);
            position = obj.a(i) + obj.b(i) * dx + ...
                obj.c(i) * dx^2 + obj.d(i) * dx^3;
        end

        function dy = calc_first_derivative(obj, x)
            if x < obj.x(1) || x > obj.x(end)
                dy = nan;
                return;
            end

            i = obj.search_index(x);
            dx = x - obj.x(i);
            dy = obj.b(i) + 2 * obj.c(i) * dx + 3 * obj.d(i) * dx^2;
        end

        function ddy = calc_second_derivative(obj, x)
            if x < obj.x(1) || x > obj.x(end)
                ddy = nan;
                return;
            end

            i = obj.search_index(x);
            dx = x - obj.x(i);
            ddy = 2 * obj.c(i) + 6 * obj.d(i) * dx;
        end

        function idx = search_index(obj, x)
            [~, idx] = histc(x, obj.x);
            if idx == obj.nx
                idx = idx - 1;
            end
        end
        
        function A = calc_A(obj, h)
            % calc matrix A for spline coefficient c
            A = zeros(obj.nx);
            A(1, 1) = 1.0;
            for i = 1:(obj.nx - 1)
                if i ~= (obj.nx - 1)
                    A(i + 1, i + 1) = 2.0 * (h(i) + h(i + 1));
                end
                A(i + 1, i) = h(i);
                A(i, i + 1) = h(i);
            end

            A(1, 2) = 0.0;
            A(obj.nx, obj.nx - 1) = 0.0;
            A(obj.nx, obj.nx) = 1.0;
        end

        function B = calc_B(obj, h)
            % calc matrix B for spline coefficient c
            B = zeros(1, obj.nx);
            for i = 1:(obj.nx - 2)
                B(i + 1) = 3.0 * (obj.a(i + 2) - obj.a(i + 1)) / h(i + 1)...
                    - 3.0 * (obj.a(i + 1) - obj.a(i)) / h(i);
            end
        end
    end
end