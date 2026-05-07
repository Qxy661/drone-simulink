classdef TransitionController < handle
%TRANSITIONCONTROLLER  eVTOL 过渡控制器
%   管理悬停↔巡航过渡:
%     1. 悬停模式: 倾转角=0°, 类四旋翼控制
%     2. 过渡模式: 倾转角渐变, 混合控制
%     3. 巡航模式: 倾转角=90°, 类固定翼控制
%
%   过渡策略:
%     - 速度调度: 基于空速决定倾转角
%     - 安全约束: 高度保持、最小速度

    properties
        pos_ctrl        % 位置控制器 (悬停模式)
        alt_ctrl        % 高度控制器 (巡航模式)
        head_ctrl       % 航向控制器 (巡航模式)
        params          % 控制器参数
        evtol_params    % eVTOL 参数

        transition_speed  % 过渡目标速度 [m/s]
        hover_speed       % 悬停模式速度阈值 [m/s]
        cruise_speed      % 巡航模式速度阈值 [m/s]
    end

    methods
        function obj = TransitionController(ctrl_params, evtol_params, dt)
            obj.params = ctrl_params;
            obj.evtol_params = evtol_params;

            % 创建子控制器
            obj.pos_ctrl = PositionController(ctrl_params, evtol_params, ...
                                              struct('k_t', evtol_params.k_t), dt);
            obj.alt_ctrl = AltitudeController(ctrl_params, evtol_params, dt);
            obj.head_ctrl = HeadingController(ctrl_params, evtol_params, dt);

            % 过渡速度阈值
            obj.hover_speed = 5;          % 悬停模式上限 [m/s]
            obj.transition_speed = 15;    % 过渡目标速度 [m/s]
            obj.cruise_speed = 20;        % 巡航模式下限 [m/s]
        end

        function [omega_cmd, tilt_cmd, mode] = update(obj, state, ...
                pos_des, heading_des, V_des, dt)
        %UPDATE  过渡控制主函数
        %   输入:
        %     state       - 12x1 当前状态
        %     pos_des     - 3x1 期望位置
        %     heading_des - 期望航向 [rad]
        %     V_des       - 期望空速 [m/s]
        %     dt          - 时间步长
        %   输出:
        %     omega_cmd - 4x1 电机转速命令
        %     tilt_cmd  - 4x1 倾转角命令
        %     mode      - 当前模式

            % 当前状态
            pos_cur = state(1:3);
            vel_cur = state(4:6);
            att_cur = state(7:9);
            rates_cur = state(10:12);

            % 空速
            R = euler_to_rotation(att_cur(1), att_cur(2), att_cur(3));
            V_body = R' * vel_cur;
            V = norm(V_body);

            % 确定飞行模式
            if V < obj.hover_speed
                mode = 'hover';
            elseif V > obj.cruise_speed
                mode = 'cruise';
            else
                mode = 'transition';
            end

            % 根据模式选择控制策略
            switch mode
                case 'hover'
                    [omega_cmd, tilt_cmd] = obj.hover_control(...
                        state, pos_des, heading_des);

                case 'transition'
                    [omega_cmd, tilt_cmd] = obj.transition_control(...
                        state, pos_des, heading_des, V_des);

                case 'cruise'
                    [omega_cmd, tilt_cmd] = obj.cruise_control(...
                        state, pos_des, heading_des, V_des);
            end
        end

        function [omega_cmd, tilt_cmd] = hover_control(obj, state, pos_des, ~)
        %HOVER_control  悬停模式控制 (类四旋翼)
            pos_cur = state(1:3);
            vel_cur = state(4:6);
            att_cur = state(7:9);
            rates_cur = state(10:12);

            % 位置环
            [att_des, thrust] = obj.pos_ctrl.update(pos_des, pos_cur, vel_cur);

            % 姿态环
            torques = obj.attitude_control(att_des, att_cur, rates_cur);

            % 混控
            omega_cmd = obj.mixer(thrust, torques);
            tilt_cmd = zeros(4, 1);
        end

        function [omega_cmd, tilt_cmd] = transition_control(obj, state, ...
                pos_des, heading_des, V_des)
        %TRANSITION_control  过渡模式控制 (混合)
            pos_cur = state(1:3);
            vel_cur = state(4:6);
            att_cur = state(7:9);
            rates_cur = state(10:12);

            % 计算倾转角 (基于速度)
            R = euler_to_rotation(att_cur(1), att_cur(2), att_cur(3));
            V_body = R' * vel_cur;
            V = norm(V_body);

            % 倾转角调度: 0° @ hover_speed → 90° @ cruise_speed
            tilt_ratio = (V - obj.hover_speed) / (obj.cruise_speed - obj.hover_speed);
            tilt_ratio = max(0, min(1, tilt_ratio));
            tilt_angle = tilt_ratio * pi/2;

            % 悬停推力 (随倾转角减小)
            [att_des_hover, thrust_hover] = obj.pos_ctrl.update(...
                pos_des, pos_cur, vel_cur);

            % 巡航推力 (随倾转角增大)
            alt_cur = -state(3);
            [elevator, throttle_cruise] = obj.alt_ctrl.update(...
                -pos_des(3), alt_cur, att_cur(2), V, V_des);

            % 混合
            thrust = (1 - tilt_ratio) * thrust_hover + ...
                      tilt_ratio * throttle_cruise * obj.evtol_params.mass * obj.evtol_params.g;

            % 姿态环
            torques = obj.attitude_control(att_des_hover, att_cur, rates_cur);

            % 混控
            omega_cmd = obj.mixer(thrust, torques);
            tilt_cmd = ones(4, 1) * tilt_angle;
        end

        function [omega_cmd, tilt_cmd] = cruise_control(obj, state, ...
                pos_des, heading_des, V_des)
        %CRUISE_control  巡航模式控制 (类固定翼)
            pos_cur = state(1:3);
            vel_cur = state(4:6);
            att_cur = state(7:9);

            % 高度控制
            alt_cur = -state(3);
            R = euler_to_rotation(att_cur(1), att_cur(2), att_cur(3));
            V_body = R' * vel_cur;
            V = norm(V_body);

            [elevator, throttle] = obj.alt_ctrl.update(...
                -pos_des(3), alt_cur, att_cur(2), V, V_des);

            % 航向控制
            [alpha, beta, ~] = wind_frame_transform(V_body);
            [aileron, rudder] = obj.head_ctrl.update(...
                heading_des, att_cur(3), att_cur(1), state(10), beta);

            % 巡航推力 (沿机头)
            T = throttle * obj.evtol_params.mass * obj.evtol_params.g;

            % 电机转速 (倾转 90°, 推力沿 X 轴)
            omega_single = sqrt(T / (obj.evtol_params.n_rotors * obj.evtol_params.k_t));
            omega_cmd = ones(4, 1) * omega_single;

            % 舵面效果通过转速差模拟 (简化)
            % 实际应有独立舵面
            tilt_cmd = ones(4, 1) * pi/2;
        end

        function torques = attitude_control(obj, att_des, att_cur, rates_cur)
        %ATTITUDE_control  简化姿态控制 (使用统一参数)
            err = att_des(:) - att_cur(:);
            err = mod(err + pi, 2*pi) - pi;

            Kp = [obj.params.att.Kp_phi; obj.params.att.Kp_theta; obj.params.att.Kp_psi];
            Kd = [obj.params.att.Kd_phi; obj.params.att.Kd_theta; obj.params.att.Kd_psi];

            torques = Kp .* err - Kd .* rates_cur;
            max_torque = obj.params.att.max_torque;
            torques = max(-max_torque, min(max_torque, torques));
        end

        function omega_cmd = mixer(obj, thrust, torques)
        %MIXER  简化混控器
            kt = obj.evtol_params.k_t;
            kd = obj.evtol_params.k_d;

            % 简化: 均分推力 + 力矩修正
            thrust = max(0, thrust);
            hover_omega = sqrt(thrust / (obj.evtol_params.n_rotors * kt));

            omega_cmd = ones(4, 1) * hover_omega;

            % 力矩修正 (需要 hover_omega > 0)
            if hover_omega > 1
                omega_cmd(1) = omega_cmd(1) + torques(3) / (4 * kd * hover_omega);
                omega_cmd(2) = omega_cmd(2) - torques(3) / (4 * kd * hover_omega);
                omega_cmd(3) = omega_cmd(3) - torques(3) / (4 * kd * hover_omega);
                omega_cmd(4) = omega_cmd(4) + torques(3) / (4 * kd * hover_omega);
            end

            omega_cmd = max(0, min(obj.evtol_params.omega_max, omega_cmd));
        end

        function reset(obj)
        %RESET  重置所有控制器
            obj.pos_ctrl.reset();
            obj.alt_ctrl.reset();
            obj.head_ctrl.reset();
        end
    end
end
