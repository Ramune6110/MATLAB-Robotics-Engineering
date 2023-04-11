% FrenetPath.m

classdef FrenetPath
    properties
        t
        d
        d_d
        d_dd
        d_ddd
        s
        s_d
        s_dd
        s_ddd
        cd
        cv
        cf
        x
        y
        yaw
        ds
        c
    end
    
    methods
        function obj = FrenetPath()
            obj.t = [];
            obj.d = [];
            obj.d_d = [];
            obj.d_dd = [];
            obj.d_ddd = [];
            obj.s = [];
            obj.s_d = [];
            obj.s_dd = [];
            obj.s_ddd = [];
            obj.cd = 0.0;
            obj.cv = 0.0;
            obj.cf = 0.0;
            obj.x = [];
            obj.y = [];
            obj.yaw = [];
            obj.ds = [];
            obj.c = [];
        end
    end
end