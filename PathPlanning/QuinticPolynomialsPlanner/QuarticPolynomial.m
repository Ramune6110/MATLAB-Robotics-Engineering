% QuarticPolynomial.m

classdef QuarticPolynomial
    properties
        a0
        a1
        a2
        a3
        a4
    end
    
    methods
        function obj = QuarticPolynomial(xs, vxs, axs, vxe, axe, time)
            obj.a0 = xs;
            obj.a1 = vxs;
            obj.a2 = axs / 2.0;

            A = [3 * time ^ 2, 4 * time ^ 3;
                 6 * time, 12 * time ^ 2];
            b = [vxe - obj.a1 - 2 * obj.a2 * time;
                 axe - 2 * obj.a2];
            x = A \ b;

            obj.a3 = x(1);
            obj.a4 = x(2);
        end
        
        function xt = calc_point(obj, t)
            xt = obj.a0 + obj.a1 * t + obj.a2 * t .^ 2 + ...
                 obj.a3 * t .^ 3 + obj.a4 * t .^ 4;
        end
        
        function xt = calc_first_derivative(obj, t)
            xt = obj.a1 + 2 * obj.a2 * t + ...
                 3 * obj.a3 * t .^ 2 + 4 * obj.a4 * t .^ 3;
        end
        
        function xt = calc_second_derivative(obj, t)
            xt = 2 * obj.a2 + 6 * obj.a3 * t + 12 * obj.a4 * t .^ 2;
        end
        
        function xt = calc_third_derivative(obj, t)
            xt = 6 * obj.a3 + 24 * obj.a4 * t;
        end
    end
end