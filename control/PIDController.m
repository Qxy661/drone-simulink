classdef PIDController < handle
%PIDCONTROLLER  通用 PID 控制器
%   支持: 抗积分饱和、微分滤波、输出限幅
%
%   用法:
%     pid = PIDController(Kp, Ki, Kd, dt);
%     pid.set_limits(-10, 10);
%     output = pid.update(error);

    properties
        Kp          % 比例增益
        Ki          % 积分增益
        Kd          % 微分增益
        dt          % 时间步长

        integral    % 积分累积
        prev_error  % 上一次误差
        prev_deriv  % 上一次微分值 (用于滤波)

        out_min     % 输出下限
        out_max     % 输出上限
        int_max     % 积分限幅 (抗饱和)

        deriv_filt  % 微分滤波系数 (0~1, 0=无滤波)
    end

    methods
        function obj = PIDController(Kp, Ki, Kd, dt)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.dt = dt;

            obj.integral = 0;
            obj.prev_error = 0;
            obj.prev_deriv = 0;

            obj.out_min = -Inf;
            obj.out_max = Inf;
            obj.int_max = Inf;
            obj.deriv_filt = 0.1;  % 默认 10% 低通滤波
        end

        function set_limits(obj, out_min, out_max, int_max)
        %SET_LIMITS  设置输出限幅和积分限幅
            obj.out_min = out_min;
            obj.out_max = out_max;
            if nargin >= 4
                obj.int_max = int_max;
            end
        end

        function set_gains(obj, Kp, Ki, Kd)
        %SET_GAINS  更新增益
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
        end

        function output = update(obj, error)
        %UPDATE  计算 PID 输出
        %   输入:
        %     error - 标量误差
        %   输出:
        %     output - PID 控制输出

            % 比例项
            P = obj.Kp * error;

            % 积分项 (带抗饱和)
            obj.integral = obj.integral + error * obj.dt;
            obj.integral = max(-obj.int_max, min(obj.int_max, obj.integral));
            I = obj.Ki * obj.integral;

            % 微分项 (带低通滤波)
            if obj.dt > 0
                deriv_raw = (error - obj.prev_error) / obj.dt;
            else
                deriv_raw = 0;
            end
            deriv_filt = obj.deriv_filt * deriv_raw + ...
                         (1 - obj.deriv_filt) * obj.prev_deriv;
            D = obj.Kd * deriv_filt;

            obj.prev_error = error;
            obj.prev_deriv = deriv_filt;

            % 总输出
            output = P + I + D;

            % 输出限幅
            output = max(obj.out_min, min(obj.out_max, output));
        end

        function reset(obj)
        %RESET  重置积分和历史
            obj.integral = 0;
            obj.prev_error = 0;
            obj.prev_deriv = 0;
        end
    end
end
