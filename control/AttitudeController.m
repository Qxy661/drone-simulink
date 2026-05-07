classdef AttitudeController < handle
%ATTITUDECONTROLLER  姿态环控制器
%   输入: 姿态误差 (期望姿态 - 当前姿态)
%   输出: 力矩命令 [tau_phi; tau_theta; tau_psi]
%
%   使用 PD 控制 (姿态环通常不需要积分)

    properties
        roll_pid        % 滚转 PID
        pitch_pid       % 俯仰 PID
        yaw_pid         % 偏航 PID
        params          % 控制器参数
    end

    methods
        function obj = AttitudeController(ctrl_params, dt)
            obj.params = ctrl_params;

            % 创建各轴 PID
            obj.roll_pid = PIDController(...
                ctrl_params.att.Kp_phi, ...
                ctrl_params.att.Ki_phi, ...
                ctrl_params.att.Kd_phi, dt);

            obj.pitch_pid = PIDController(...
                ctrl_params.att.Kp_theta, ...
                ctrl_params.att.Ki_theta, ...
                ctrl_params.att.Kd_theta, dt);

            obj.yaw_pid = PIDController(...
                ctrl_params.att.Kp_psi, ...
                ctrl_params.att.Ki_psi, ...
                ctrl_params.att.Kd_psi, dt);

            % 设置力矩限幅
            max_torque = ctrl_params.att.max_torque;
            obj.roll_pid.set_limits(-max_torque, max_torque, ctrl_params.anti_windup);
            obj.pitch_pid.set_limits(-max_torque, max_torque, ctrl_params.anti_windup);
            obj.yaw_pid.set_limits(-max_torque, max_torque, ctrl_params.anti_windup);
        end

        function torques = update(obj, att_des, att_cur, rates_cur)
        %UPDATE  姿态环控制
        %   输入:
        %     att_des   - 3x1 期望姿态 [phi_d; theta_d; psi_d] (rad)
        %     att_cur   - 3x1 当前姿态 [phi; theta; psi] (rad)
        %     rates_cur - 3x1 当前角速度 [p; q; r] (rad/s)
        %   输出:
        %     torques - 3x1 力矩命令 [tau_phi; tau_theta; tau_psi]
        %
        %   架构: P(角度误差) - D(角速度) + I(角度误差积分)
        %   微分项直接使用陀螺仪测量, 避免数值微分噪声

            % 姿态误差
            err = att_des(:) - att_cur(:);
            err = mod(err + pi, 2*pi) - pi;

            % P 项
            P = [obj.params.att.Kp_phi * err(1);
                 obj.params.att.Kp_theta * err(2);
                 obj.params.att.Kp_psi * err(3)];

            % I 项 (通过 PID 对象维护积分)
            I_term = [obj.params.att.Ki_phi * obj.roll_pid.integral;
                      obj.params.att.Ki_theta * obj.pitch_pid.integral;
                      obj.params.att.Ki_psi * obj.yaw_pid.integral];
            % 更新积分
            obj.roll_pid.integral = obj.roll_pid.integral + err(1) * obj.roll_pid.dt;
            obj.pitch_pid.integral = obj.pitch_pid.integral + err(2) * obj.pitch_pid.dt;
            obj.yaw_pid.integral = obj.yaw_pid.integral + err(3) * obj.yaw_pid.dt;
            % 积分限幅
            obj.roll_pid.integral = max(-obj.roll_pid.int_max, min(obj.roll_pid.int_max, obj.roll_pid.integral));
            obj.pitch_pid.integral = max(-obj.pitch_pid.int_max, min(obj.pitch_pid.int_max, obj.pitch_pid.integral));
            obj.yaw_pid.integral = max(-obj.yaw_pid.int_max, min(obj.yaw_pid.int_max, obj.yaw_pid.integral));
            I_term = [obj.params.att.Ki_phi * obj.roll_pid.integral;
                      obj.params.att.Ki_theta * obj.pitch_pid.integral;
                      obj.params.att.Ki_psi * obj.yaw_pid.integral];

            % D 项 (直接用陀螺仪角速度, 负反馈阻尼)
            D = [-obj.params.att.Kd_phi * rates_cur(1);
                 -obj.params.att.Kd_theta * rates_cur(2);
                 -obj.params.att.Kd_psi * rates_cur(3)];

            torques = P + I_term + D;

            % 输出限幅
            max_torque = obj.params.att.max_torque;
            torques = max(-max_torque, min(max_torque, torques));
        end

        function reset(obj)
        %RESET  重置所有积分器
            obj.roll_pid.reset();
            obj.pitch_pid.reset();
            obj.yaw_pid.reset();
        end
    end
end
