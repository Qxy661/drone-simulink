classdef HeadingController < handle
%HEADINGCONTROLLER  固定翼航向控制器
%   通过副翼和方向舵控制航向
%
%   控制策略:
%     - 副翼: 控制滚转角 -> 转弯
%     - 方向舵: 协调转弯 (消除侧滑)
%
%   协调转弯:
%     tan(phi) = V^2 / (g * R)
%     其中 R 为转弯半径

    properties
        roll_pid        % 滚转 PID
        heading_pid     % 航向 PID
        yaw_pid         % 偏航 PID (方向舵)
        params          % 控制器参数
        fw_params       % 固定翼参数
    end

    methods
        function obj = HeadingController(ctrl_params, fw_params, dt)
            obj.params = ctrl_params;
            obj.fw_params = fw_params;

            % 航向 -> 滚转角
            obj.heading_pid = PIDController(...
                ctrl_params.fw.Kp_heading, ...
                ctrl_params.fw.Ki_heading, ...
                ctrl_params.fw.Kd_heading, dt);
            obj.heading_pid.set_limits(-45*pi/180, 45*pi/180);  % 最大滚转角

            % 滚转角 -> 副翼
            obj.roll_pid = PIDController(...
                ctrl_params.fw.Kp_roll, ...
                ctrl_params.fw.Ki_roll, ...
                ctrl_params.fw.Kd_roll, dt);
            obj.roll_pid.set_limits(-fw_params.aileron_max, ...
                                     fw_params.aileron_max, 0.3);

            % 侧滑角 -> 方向舵 (协调转弯)
            obj.yaw_pid = PIDController(...
                ctrl_params.fw.Kp_yaw, 0, ...
                ctrl_params.fw.Kd_yaw, dt);
            obj.yaw_pid.set_limits(-fw_params.rudder_max, ...
                                    fw_params.rudder_max);
        end

        function [aileron, rudder] = update(obj, heading_des, heading_cur, ...
                                            roll_cur, roll_rate, beta)
        %UPDATE  航向控制
        %   输入:
        %     heading_des - 期望航向 [rad]
        %     heading_cur - 当前航向 [rad]
        %     roll_cur    - 当前滚转角 [rad]
        %     roll_rate   - 滚转角速度 [rad/s]
        %     beta        - 侧滑角 [rad]
        %   输出:
        %     aileron  - 副翼偏转 [rad]
        %     rudder   - 方向舵偏转 [rad]

            % 航向误差 -> 期望滚转角
            heading_err = heading_des - heading_cur;
            % 归一化到 [-pi, pi]
            heading_err = mod(heading_err + pi, 2*pi) - pi;
            roll_des = obj.heading_pid.update(heading_err);

            % 滚转角误差 -> 副翼
            roll_err = roll_des - roll_cur;
            aileron = obj.roll_pid.update(roll_err);

            % 侧滑角 -> 方向舵 (协调转弯)
            rudder = obj.yaw_pid.update(-beta);
        end

        function reset(obj)
        %RESET  重置所有积分器
            obj.roll_pid.reset();
            obj.heading_pid.reset();
            obj.yaw_pid.reset();
        end
    end
end
