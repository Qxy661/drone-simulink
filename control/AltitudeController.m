classdef AltitudeController < handle
%ALTITUDECONTROLLER  固定翼高度控制器
%   通过升降舵和油门控制高度
%
%   控制策略:
%     - 升降舵: 控制俯仰角 -> 影响爬升率
%     - 油门: 控制速度 (维持空速)
%
%   参考: Beard & McLain, Ch. 6

    properties
        pitch_pid       % 俯仰角 PID
        alt_pid         % 高度 PID
        throttle_pid    % 油门 PID (空速保持)
        params          % 控制器参数
        fw_params       % 固定翼参数
    end

    methods
        function obj = AltitudeController(ctrl_params, fw_params, dt)
            obj.params = ctrl_params;
            obj.fw_params = fw_params;

            % 高度 -> 俯仰角
            obj.alt_pid = PIDController(...
                ctrl_params.fw.Kp_alt, 0, ctrl_params.fw.Kd_alt, dt);
            obj.alt_pid.set_limits(-30*pi/180, 30*pi/180);  % 最大俯仰角

            % 俯仰角 -> 升降舵
            obj.pitch_pid = PIDController(...
                ctrl_params.fw.Kp_pitch, ...
                ctrl_params.fw.Ki_pitch, ...
                ctrl_params.fw.Kd_pitch, dt);
            obj.pitch_pid.set_limits(-fw_params.elevator_max, ...
                                      fw_params.elevator_max, 0.5);

            % 空速 -> 油门
            obj.throttle_pid = PIDController(...
                ctrl_params.fw.Kp_throttle, ...
                ctrl_params.fw.Ki_throttle, ...
                ctrl_params.fw.Kd_throttle, dt);
            obj.throttle_pid.set_limits(0, 1);  % 油门 0~1
        end

        function [elevator, throttle] = update(obj, alt_des, alt_cur, ...
                                               pitch_cur, V_cur, V_des)
        %UPDATE  高度控制
        %   输入:
        %     alt_des   - 期望高度 [m]
        %     alt_cur   - 当前高度 [m]
        %     pitch_cur - 当前俯仰角 [rad]
        %     V_cur     - 当前空速 [m/s]
        %     V_des     - 期望空速 [m/s]
        %   输出:
        %     elevator  - 升降舵偏转 [rad]
        %     throttle  - 油门 (0~1)

            % 高度误差 -> 期望俯仰角
            alt_err = alt_des - alt_cur;
            pitch_des = obj.alt_pid.update(alt_err);

            % 俯仰角误差 -> 升降舵
            pitch_err = pitch_des - pitch_cur;
            elevator = obj.pitch_pid.update(pitch_err);

            % 空速误差 -> 油门
            V_err = V_des - V_cur;
            throttle = obj.throttle_pid.update(V_err);
        end

        function reset(obj)
        %RESET  重置所有积分器
            obj.alt_pid.reset();
            obj.pitch_pid.reset();
            obj.throttle_pid.reset();
        end
    end
end
