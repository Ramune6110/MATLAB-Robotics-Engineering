% CubicSpline2D.m

classdef CubicSpline2D
    properties
        s
        ds
        sx
        sy
    end
    methods
        function obj = CubicSpline2D(x, y)
%             dx = diff(x);
%             dy = diff(y);
%             obj.ds = hypot(dx, dy);
%             obj.s = [0 cumsum(ds)];
            
            [obj.s, obj.ds] = obj.calc_s(x, y);
            obj.sx = CubicSpline1D(obj.s, x);
            obj.sy = CubicSpline1D(obj.s, y);
        end

        function [s, ds] = calc_s(obj, x, y)
            dx = diff(x);
            dy = diff(y);
            ds = hypot(dx, dy);
            s = [0 cumsum(ds)];
        end

        function [x, y] = calc_position(obj, s)
            x = calc_position(obj.sx, s);
            y = calc_position(obj.sy, s);
        end

        function k = calc_curvature(obj, s)
            dx = calc_first_derivative(obj.sx, s);
            ddx = calc_second_derivative(obj.sx, s);
            dy = calc_first_derivative(obj.sy, s);
            ddy = calc_second_derivative(obj.sy, s);
            k = (ddy * dx - ddx * dy) / ((dx^2 + dy^2)^(3/2));
        end

        function yaw = calc_yaw(obj, s)
            dx = calc_first_derivative(obj.sx, s);
            dy = calc_first_derivative(obj.sy, s);
            yaw = atan2(dy, dx);
        end
    end
end