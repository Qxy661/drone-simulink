classdef ModularDynamics < handle
%MODULARDYNAMICS  模块化无人机动力学
%   支持可配置旋翼拓扑、故障注入、自适应混控
%
%   拓扑配置:
%     - 旋翼数量 (3~8)
%     - 旋翼位置和方向
%     - 推力/扭矩系数
%
%   故障注入:
%     - 电机效率下降 (0~1)
%     - 电机完全失效 (0)
%     - 传感器噪声增大
%
%   自适应控制:
%     - 故障后重新计算混控矩阵
%     - 伪逆分配

    properties
        config          % 拓扑配置
        state           % 12维状态
        omega           % 旋翼转速 [nx1]
        fault           % 故障状态
        mixer           % 混控矩阵
        mixer_pinv      % 伪逆混控矩阵
        energy_used     % 能耗
        sim_dt          % 仿真步长 [s]
    end

    methods
        function obj = ModularDynamics(config)
        %MODULARDYNAMICS  构造函数
        %   config - 结构体:
        %     .n_rotors    - 旋翼数量
        %     .positions   - nx3 旋翼位置矩阵
        %     .directions  - nx1 旋转方向 (+1/-1)
        %     .k_t         - 推力系数
        %     .k_d         - 反扭矩系数
        %     .tau_m       - 电机时间常数
        %     .omega_max   - 最大转速
        %     .mass        - 质量
        %     .I           - 惯量矩阵
        %     .g           - 重力加速度

            obj.config = config;
            obj.state = zeros(12, 1);
            obj.omega = zeros(config.n_rotors, 1);
            obj.energy_used = 0;
            obj.sim_dt = 0.001;

            % 初始化故障状态 (无故障)
            obj.fault = struct();
            obj.fault.efficiency = ones(config.n_rotors, 1);  % 0~1
            obj.fault.failed = false(config.n_rotors, 1);     % true=失效

            % 计算混控矩阵
            obj.compute_mixer();
        end

        function reset(obj, pos, vel, att, rates)
        %RESET  重置状态
            obj.state(1:3) = pos(:);
            obj.state(4:6) = vel(:);
            obj.state(7:9) = att(:);
            obj.state(10:12) = rates(:);
            obj.omega = zeros(obj.config.n_rotors, 1);
            obj.energy_used = 0;
        end

        function compute_mixer(obj)
        %COMPUTE_mixER  计算混控矩阵
            n = obj.config.n_rotors;
            kt = obj.config.k_t;
            kd = obj.config.k_d;

            obj.mixer = zeros(4, n);
            for i = 1:n
                pos = obj.config.positions(i, :)';
                dir = obj.config.directions(i);

                % 推力贡献 (沿 Z 轴)
                obj.mixer(1, i) = kt;

                % 滚转力矩 (Y * Fz)
                obj.mixer(2, i) = pos(2) * kt;

                % 俯仰力矩 (-X * Fz)
                obj.mixer(3, i) = -pos(1) * kt;

                % 偏航力矩 (反扭矩)
                obj.mixer(4, i) = dir * kd;
            end

            % 伪逆 (用于故障后重分配)
            obj.mixer_pinv = pinv(obj.mixer);
        end

        function inject_fault(obj, rotor_id, efficiency)
        %INJECT_fault  注入电机故障
        %   rotor_id  - 旋翼编号 (1~n)
        %   efficiency - 效率 (0=完全失效, 1=正常)
            if rotor_id >= 1 && rotor_id <= obj.config.n_rotors
                obj.fault.efficiency(rotor_id) = max(0, min(1, efficiency));
                obj.fault.failed(rotor_id) = (efficiency == 0);
            end
        end

        function state_dot = dynamics(obj, t, state, omega_cmd)
        %DYNAMICS  模块化动力学
            vel = state(4:6);
            att = state(7:9);
            omega_body = state(10:12);

            phi = att(1); theta = att(2); psi = att(3);
            R = euler_to_rotation(phi, theta, psi);

            % 更新转速 (一阶惯性 + 故障效率)
            dt = obj.sim_dt;
            alpha = dt / obj.config.tau_m;
            omega_ideal = obj.omega + alpha * (omega_cmd(:) - obj.omega);

            % 应用故障
            for i = 1:obj.config.n_rotors
                if obj.fault.failed(i)
                    obj.omega(i) = 0;
                else
                    obj.omega(i) = omega_ideal(i) * obj.fault.efficiency(i);
                end
            end
            obj.omega = max(0, min(obj.config.omega_max, obj.omega));

            % 计算推力和力矩
            F_body = [0; 0; 0];
            M_body = [0; 0; 0];

            for i = 1:obj.config.n_rotors
                F_i = obj.config.k_t * obj.omega(i)^2;
                pos_i = obj.config.positions(i, :)';
                dir_i = obj.config.directions(i);

                F_body = F_body + [0; 0; -F_i];
                M_body = M_body + cross(pos_i, [0; 0; -F_i]) + ...
                         [0; 0; dir_i * obj.config.k_d * obj.omega(i)^2];
            end

            % 平动方程
            F_world = R * F_body;
            gravity = [0; 0; -obj.config.mass * obj.config.g];
            accel = (F_world + gravity) / obj.config.mass;

            % 转动方程
            I = obj.config.I;
            gyro_torque = cross(omega_body, I * omega_body);
            omega_dot = I \ (M_body - gyro_torque);

            % 姿态导数
            sp = sin(phi); cp = cos(phi);
            tt = tan(theta); ct = cos(theta);
            W = [1, sp*tt, cp*tt; 0, cp, -sp; 0, sp/ct, cp/ct];
            att_dot = W * omega_body;

            state_dot = zeros(12, 1);
            state_dot(1:3) = vel;
            state_dot(4:6) = accel;
            state_dot(7:9) = att_dot;
            state_dot(10:12) = omega_dot;

            % 能耗
            P = sum(obj.config.k_t * obj.omega.^3) / 0.85;
            obj.energy_used = obj.energy_used + P * obj.sim_dt / 3600;
        end

        function omega_cmd = adaptive_mix(obj, commands)
        %ADAPTIVE_mix  自适应混控 (故障后重分配)
        %   输入:
        %     commands - 4x1 [T; tau_phi; tau_theta; tau_psi]
        %   输出:
        %     omega_cmd - nx1 旋翼转速命令

            % 考虑故障效率的混控矩阵
            eff = obj.fault.efficiency;
            active = ~obj.fault.failed;

            % 调整混控矩阵 (失效电机列置零)
            M_adj = obj.mixer;
            for i = 1:obj.config.n_rotors
                if obj.fault.failed(i)
                    M_adj(:, i) = 0;
                else
                    M_adj(:, i) = M_adj(:, i) * eff(i);
                end
            end

            % 伪逆重分配
            omega_sq = pinv(M_adj) * commands(:);
            omega_sq = max(0, omega_sq);
            omega_cmd = sqrt(omega_sq);
        end

        function n_active = count_active(obj)
        %COUNT_active  计算活跃电机数
            n_active = sum(~obj.fault.failed);
        end
    end
end
