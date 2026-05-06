classdef EvtolDynamics < DroneDynamics
%EVTOLDYNAMICS  倾转旋翼 eVTOL 动力学模型
%   构型: 4 倾转旋翼 + 固定翼
%   模式: 悬停 (tilt=0°) → 过渡 → 巡航 (tilt=90°)
%
%   状态: [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r] (12维)
%   额外状态: tilt_angle (4个旋翼倾转角)
%
%   参考:
%     - Rotorcraft aerodynamics + fixed-wing aerodynamics
%     - Tilt-rotor transition dynamics

    properties
        aero            % AeroModel 对象 (巡航气动)
        tilt_angles     % 4x1 当前倾转角 [rad] (0=悬停, pi/2=巡航)
        tilt_cmd        % 4x1 期望倾转角 [rad]
        omega           % 4x1 当前电机转速 [rad/s]
        mode            % 'hover', 'transition', 'cruise'
        energy_used     % 累计能耗 [Wh]
    end

    methods
        function obj = EvtolDynamics(evtol_params, aero_params)
            obj@DroneDynamics(evtol_params);
            obj.aero = AeroModel(aero_params, evtol_params);
            obj.tilt_angles = zeros(4, 1);
            obj.tilt_cmd = zeros(4, 1);
            obj.omega = zeros(4, 1);
            obj.mode = 'hover';
            obj.energy_used = 0;
        end

        function reset(obj, pos, vel, att, rates, tilt_init)
        %RESET  重置状态
            reset@DroneDynamics(obj, pos, vel, att, rates);
            if nargin >= 6
                obj.tilt_angles = tilt_init(:);
            else
                obj.tilt_angles = zeros(4, 1);
            end
            obj.omega = zeros(4, 1);
            obj.energy_used = 0;
        end

        function set_tilt_cmd(obj, tilt_cmd)
        %SET_TILT_CMD  设置倾转角命令
            obj.tilt_cmd = max(obj.params.tilt_min, ...
                               min(obj.params.tilt_max, tilt_cmd(:)));
        end

        function set_mode(obj, mode)
        %SET_mode  设置飞行模式
            switch mode
                case 'hover'
                    obj.set_tilt_cmd([0; 0; 0; 0]);
                case 'cruise'
                    obj.set_tilt_cmd([pi/2; pi/2; pi/2; pi/2]);
                case 'transition'
                    % 由外部设置具体的倾转角
            end
            obj.mode = mode;
        end

        function state_dot = dynamics(obj, t, state, omega_cmd, ~)
        %DYNAMICS  eVTOL 状态微分方程
        %   输入:
        %     state    - 12x1 [pos; vel; att; rates]
        %     omega_cmd - 4x1 电机转速命令 [rad/s]
        %     ~        - 不使用
        %   输出:
        %     state_dot - 12x1 状态导数

            % 提取状态
            vel = state(4:6);
            att = state(7:9);
            omega_body = state(10:12);

            phi = att(1);
            theta = att(2);
            psi = att(3);

            % 旋转矩阵
            R = obj.euler_to_rotation(phi, theta, psi);

            % 机体速度
            V_body = R' * vel;

            % 更新倾转角 (一阶惯性)
            obj.update_tilt(t);

            % 计算各旋翼推力和力矩
            [F_rotors, M_rotors] = obj.compute_rotor_forces(omega_cmd);

            % 计算气动力 (巡航模式贡献)
            [F_aero, M_aero] = obj.compute_aero(V_body, omega_body);

            % === 总外力 (机体系) ===
            F_body = F_rotors + F_aero;

            % === 总力矩 (机体系) ===
            M_body = M_rotors + M_aero;

            % === 平动方程 ===
            F_world = R * F_body;
            gravity = [0; 0; -obj.params.mass * obj.params.g];
            accel = (F_world + gravity) / obj.params.mass;

            % === 转动方程 ===
            I = obj.params.I;
            gyro_torque = cross(omega_body, I * omega_body);
            omega_dot = I \ (M_body - gyro_torque);

            % === 姿态导数 ===
            W = obj.angular_velocity_matrix(phi, theta);
            att_dot = W * omega_body;

            % === 组装状态导数 ===
            state_dot = zeros(12, 1);
            state_dot(1:3) = vel;
            state_dot(4:6) = accel;
            state_dot(7:9) = att_dot;
            state_dot(10:12) = omega_dot;

            % 计算能耗
            obj.compute_energy(omega_cmd, t);
        end

        function update_tilt(obj, ~)
        %UPDATE_TILT  更新倾转角 (一阶惯性响应)
            dt = 0.001;  % 假设固定步长
            alpha = dt / obj.params.tau_tilt;
            obj.tilt_angles = obj.tilt_angles + alpha * (obj.tilt_cmd - obj.tilt_angles);

            % 限幅
            obj.tilt_angles = max(obj.params.tilt_min, ...
                                  min(obj.params.tilt_max, obj.tilt_angles));
        end

        function [F_rotors, M_rotors] = compute_rotor_forces(obj, omega_cmd)
        %COMPUTE_ROTOR_forces  计算旋翼推力和力矩
        %   旋翼推力方向随倾转角变化:
        %     tilt=0: 推力沿 body Z (悬停)
        %     tilt=90°: 推力沿 body X (巡航)

            F_rotors = zeros(3, 1);
            M_rotors = zeros(3, 1);

            % 更新电机转速 (一阶惯性)
            dt = 0.001;
            alpha = dt / obj.params.tau_m;
            obj.omega = obj.omega + alpha * (omega_cmd(:) - obj.omega);
            obj.omega = max(0, min(obj.params.omega_max, obj.omega));

            for i = 1:obj.params.n_rotors
                % 旋翼推力 (沿旋翼轴方向)
                F_mag = obj.params.k_t * obj.omega(i)^2;

                % 推力方向 (在机体系中)
                % 倾转角: 0=沿Z轴, pi/2=沿X轴
                tilt = obj.tilt_angles(i);
                F_dir = [sin(tilt); 0; -cos(tilt)];  % Z 向下为正

                % 旋翼推力
                F_i = F_mag * F_dir;

                % 旋翼位置
                pos_i = obj.params.motor_pos(i, :)';

                % 旋翼力矩: 推力矩 + 反扭矩
                M_thrust = cross(pos_i, F_i);
                M_torque = obj.params.k_d * obj.omega(i)^2 * ...
                           [0; 0; obj.params.motor_dir(i)];

                F_rotors = F_rotors + F_i;
                M_rotors = M_rotors + M_thrust + M_torque;
            end
        end

        function [F_aero, M_aero] = compute_aero(obj, V_body, omega)
        %COMPUTE_aero  计算气动贡献
        %   仅在巡航模式下显著

            V = norm(V_body);
            if V < 1.0
                F_aero = zeros(3, 1);
                M_aero = zeros(3, 1);
                return;
            end

            % 使用平均倾转角作为气动迎角参考
            avg_tilt = mean(obj.tilt_angles);

            % 简化气动模型
            qbar = 0.5 * obj.params.rho * V^2 * obj.params.wing_area;

            % 迎角 (简化)
            alpha = atan2(V_body(3), V_body(1)) + avg_tilt - pi/2;

            % 升力和阻力
            CL = obj.params.CL0 + obj.params.CL_alpha * alpha;
            CD = obj.params.CD0 + CL^2 / (pi * obj.params.e_oswald * obj.params.AR);

            L = qbar * CL;
            D = qbar * CD;

            % 气动力 (机体系, 简化)
            F_aero = [-D; 0; -L];  % 阻力向前, 升力向上

            % 气动力矩 (简化)
            M_aero = zeros(3, 1);
        end

        function compute_energy(obj, omega_cmd, t)
        %COMPUTE_energy  计算能耗
            dt = 0.001;

            % 功率 = 推力 * 速度 / 效率
            % 简化: P = k_t * omega^3 * (1/eta)
            eta = obj.params.motor_efficiency * obj.params.prop_efficiency;
            P = sum(obj.params.k_t * obj.omega.^3) / eta;

            % 累计能耗 (Wh)
            obj.energy_used = obj.energy_used + P * dt / 3600;
        end

        function [alpha, beta, V] = get_aero_state(obj)
        %GET_AERO_state  获取气流角
            R = obj.euler_to_rotation(obj.state(7), obj.state(8), obj.state(9));
            V_body = R' * obj.state(4:6);
            V = norm(V_body);
            if V < 0.1
                alpha = 0; beta = 0; return;
            end
            alpha = atan2(V_body(3), V_body(1));
            beta = asin(max(-1, min(1, V_body(2) / V)));
        end
    end

    methods (Static)
        function W = angular_velocity_matrix(phi, theta)
            sp = sin(phi); cp = cos(phi);
            tt = tan(theta); ct = cos(theta);
            W = [1, sp*tt, cp*tt;
                 0, cp,    -sp;
                 0, sp/ct,  cp/ct];
        end
    end
end
