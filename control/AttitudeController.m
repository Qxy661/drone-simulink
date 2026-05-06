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

            % 姿态误差
            err = att_des(:) - att_cur(:);

            % 角度归一化到 [-pi, pi]
            err = mod(err + pi, 2*pi) - pi;

            % PID 计算 (用角度误差作为比例, 角速度作为微分)
            tau_phi = obj.roll_pid.update(err(1));
            tau_theta = obj.pitch_pid.update(err(2));
            tau_psi = obj.yaw_pid.update(err(3));

            % 补偿角速度项 (角速度阻尼)
            tau_phi = tau_phi - obj.params.att.Kd_phi * rates_cur(1);
            tau_theta = tau_theta - obj.params.att.Kd_theta * rates_cur(2);
            tau_psi = tau_psi - obj.params.att.Kd_psi * rates_cur(3);

            torques = [tau_phi; tau_theta; tau_psi];
        end

        function reset(obj)
        %RESET  重置所有积分器
            obj.roll_pid.reset();
            obj.pitch_pid.reset();
            obj.yaw_pid.reset();
        end
    end
end
