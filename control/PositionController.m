classdef PositionController < handle
%POSITIONCONTROLLER  位置环控制器 (外环)
%   输入: 位置误差
%   输出: 期望姿态 + 推力增量
%
%   位置环 -> [phi_des, theta_des, delta_T]
%   姿态环 -> 力矩
%   混控器 -> 电机转速

    properties
        x_pid           % X 轴 PID
        y_pid           % Y 轴 PID
        z_pid           % Z 轴 PID
        params          % 控制器参数
        hover_thrust    % 悬停推力 [N]
    end

    methods
        function obj = PositionController(ctrl_params, quad_params, motor_params, dt)
            obj.params = ctrl_params;

            % 悬停推力 = m*g
            obj.hover_thrust = quad_params.mass * quad_params.g;

            % 创建各轴 PID
            obj.x_pid = PIDController(...
                ctrl_params.pos.Kp_x, ...
                ctrl_params.pos.Ki_x, ...
                ctrl_params.pos.Kd_x, dt);
            obj.x_pid.set_limits(-10, 10, 5);

            obj.y_pid = PIDController(...
                ctrl_params.pos.Kp_y, ...
                ctrl_params.pos.Ki_y, ...
                ctrl_params.pos.Kd_y, dt);
            obj.y_pid.set_limits(-10, 10, 5);

            obj.z_pid = PIDController(...
                ctrl_params.pos.Kp_z, ...
                ctrl_params.pos.Ki_z, ...
                ctrl_params.pos.Kd_z, dt);
            obj.z_pid.set_limits(-ctrl_params.pos.max_vel_z, ...
                                  ctrl_params.pos.max_vel_z, 5);
        end

        function [att_des, thrust] = update(obj, pos_des, pos_cur, vel_cur)
        %UPDATE  位置环控制
        %   输入:
        %     pos_des - 3x1 期望位置 [x_d; y_d; z_d]
        %     pos_cur - 3x1 当前位置 [x; y; z]
        %     vel_cur - 3x1 当前速度 [vx; vy; vz]
        %   输出:
        %     att_des - 3x1 期望姿态 [phi_d; theta_d; psi_d]
        %     thrust  - 总推力 [N]

            % 位置误差
            err = pos_des(:) - pos_cur(:);

            % PID 计算
            ax = obj.x_pid.update(err(1));
            ay = obj.y_pid.update(err(2));
            az = obj.z_pid.update(err(3));

            % 期望倾斜角 (从加速度命令转换)
            % phi_d = atan2(ay, g)
            % theta_d = -atan2(ax, g)
            g = 9.81;
            max_tilt = obj.params.pos.max_tilt;

            phi_des = atan2(ay, g + az);
            theta_des = -atan2(ax, g + az);

            % 限幅
            phi_des = max(-max_tilt, min(max_tilt, phi_des));
            theta_des = max(-max_tilt, min(max_tilt, theta_des));
            psi_des = 0;  % 偏航角由外部设定

            att_des = [phi_des; theta_des; psi_des];

            % 推力 = 悬停推力 + 垂直加速度补偿
            thrust = obj.hover_thrust + obj.params.mass * az;
            thrust = max(0, thrust);  % 不能为负
        end

        function set_yaw(obj, psi_des)
        %SET_YAW  设置偏航角目标
            % 保存为属性供 update 使用
            obj.y_pid.set_gains(obj.params.pos.Kp_y, obj.params.pos.Ki_y, obj.params.pos.Kd_y);
        end

        function reset(obj)
        %RESET  重置所有积分器
            obj.x_pid.reset();
            obj.y_pid.reset();
            obj.z_pid.reset();
        end
    end
end
