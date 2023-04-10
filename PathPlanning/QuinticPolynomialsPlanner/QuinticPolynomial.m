% QuinticPolynomial.m

classdef QuinticPolynomial
    properties
        a0
        a1
        a2
        a3
        a4
        a5
    end
    
    methods
        function obj = QuinticPolynomial(xs, vxs, axs, xe, vxe, axe, time)
            obj.a0 = xs;
            obj.a1 = vxs;
            obj.a2 = axs / 2.0;

            A = [time ^ 3, time ^ 4, time ^ 5;
                 3 * time ^ 2, 4 * time ^ 3, 5 * time ^ 4;
                 6 * time, 12 * time ^ 2, 20 * time ^ 3];
            b = [xe - obj.a0 - obj.a1 * time - obj.a2 * time ^ 2;
                 vxe - obj.a1 - 2 * obj.a2 * time;
                 axe - 2 * obj.a2];
            x = A \ b;

            obj.a3 = x(1);
            obj.a4 = x(2);
            obj.a5 = x(3);
        end
        
        function xt = calc_point(obj, t)
            xt = obj.a0 + obj.a1 * t + obj.a2 * t .^ 2 + ...
                 obj.a3 * t .^ 3 + obj.a4 * t .^ 4 + obj.a5 * t .^ 5;
        end
        
        function xt = calc_first_derivative(obj, t)
            xt = obj.a1 + 2 * obj.a2 * t + ...
                 3 * obj.a3 * t .^ 2 + 4 * obj.a4 * t .^ 3 + 5 * obj.a5 * t .^ 4;
        end
        
        function xt = calc_second_derivative(obj, t)
            xt = 2 * obj.a2 + 6 * obj.a3 * t + 12 * obj.a4 * t .^ 2 + 20 * obj.a5 * t .^ 3;
        end
        
        function xt = calc_third_derivative(obj, t)
            xt = 6 * obj.a3 + 24 * obj.a4 * t + 60 * obj.a5 * t .^ 2;
        end
    end
end