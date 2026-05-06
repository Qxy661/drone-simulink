classdef MotorModel < handle
%MOTORMODEL  电机模型
%   推力: F = k_t * omega^2
%   反扭矩: M = k_d * omega^2
%   一阶惯性响应: omega_dot = (1/tau) * (omega_cmd - omega)

    properties
        params      % 电机参数
        omega       % 当前转速 [4x1] (rad/s)
    end

    methods
        function obj = MotorModel(motor_params)
            obj.params = motor_params;
            obj.omega = zeros(4, 1);
        end

        function reset(obj, omega_init)
        %RESET  重置电机转速
            if nargin < 2
                obj.omega = zeros(4, 1);
            else
                obj.omega = omega_init(:);
            end
        end

        function [F, M] = update(obj, omega_cmd, dt)
        %UPDATE  更新电机状态并输出推力/力矩
        %   输入:
        %     omega_cmd - 4x1 期望转速 (rad/s)
        %     dt        - 时间步长 (s)
        %   输出:
        %     F - 4x1 各电机推力 (N)
        %     M - 4x1 各电机反扭矩 (N·m)

            % 限幅
            omega_cmd = max(obj.params.omega_min, ...
                           min(obj.params.omega_max, omega_cmd(:)));

            % 一阶惯性响应
            alpha = dt / obj.params.tau_m;
            obj.omega = obj.omega + alpha * (omega_cmd - obj.omega);

            % 推力和反扭矩
            F = obj.params.k_t * obj.omega.^2;
            M = obj.params.k_d * obj.omega.^2;
        end

        function [F, M] = steady(obj, omega_cmd)
        %STEADY  稳态推力/力矩 (忽略动态响应)
            omega_cmd = max(obj.params.omega_min, ...
                           min(obj.params.omega_max, omega_cmd(:)));
            F = obj.params.k_t * omega_cmd.^2;
            M = obj.params.k_d * omega_cmd.^2;
        end
    end
end
