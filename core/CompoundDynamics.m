classdef CompoundDynamics < DroneDynamics
%COMPOUNDDYNAMICS  复合翼动力学模型
%   构型: 4 升力旋翼 (固定朝上) + 1 推进器 (机头方向) + 固定翼
%
%   状态: [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r] (12维)
%   额外状态: lift_omega (4个升力旋翼转速), push_omega (推进器转速)
%
%   模式:
%     hover   - 仅升力旋翼 (低速/悬停)
%     transition - 升力旋翼 + 推进器 + 部分气动
%     cruise  - 推进器为主 + 气动升力, 升力旋翼辅助

    properties
        aero            % AeroModel 对象
        lift_omega      % 4x1 升力旋翼转速 [rad/s]
        push_omega      % 1x1 推进器转速 [rad/s]
        surfaces        % 舵面偏转
        mode            % 'hover', 'transition', 'cruise'
        energy_used     % 累计能耗 [Wh]
        sim_dt          % 仿真步长 [s]
    end

    methods
        function obj = CompoundDynamics(compound_params, aero_params)
            obj@DroneDynamics(compound_params);
            obj.aero = AeroModel(aero_params, compound_params);
            obj.lift_omega = zeros(4, 1);
            obj.push_omega = 0;
            obj.surfaces = struct('elevator', 0, 'aileron', 0, 'rudder', 0);
            obj.mode = 'hover';
            obj.energy_used = 0;
            obj.sim_dt = 0.001;
        end

        function reset(obj, pos, vel, att, rates)
        %RESET  重置状态
            reset@DroneDynamics(obj, pos, vel, att, rates);
            obj.lift_omega = zeros(4, 1);
            obj.push_omega = 0;
            obj.energy_used = 0;
        end

        function state_dot = dynamics(obj, t, state, commands, ~)
        %DYNAMICS  复合翼状态微分方程
        %   输入:
        %     state    - 12x1 [pos; vel; att; rates]
        %     commands - 结构体:
        %       .lift_cmd - 4x1 升力旋翼转速命令
        %       .push_cmd - 推进器转速命令
        %       .elevator - 升降舵偏转
        %       .aileron  - 副翼偏转
        %       .rudder   - 方向舵偏转
        %   输出:
        %     state_dot - 12x1 状态导数

            % 提取状态
            vel = state(4:6);
            att = state(7:9);
            omega = state(10:12);

            phi = att(1); theta = att(2); psi = att(3);

            % 旋转矩阵
            R = obj.euler_to_rotation(phi, theta, psi);
            V_body = R' * vel;

            % 更新电机转速 (一阶惯性)
            dt = obj.sim_dt;
            lift_alpha = dt / obj.params.lift_tau;
            push_alpha = dt / obj.params.push_tau;

            obj.lift_omega = obj.lift_omega + lift_alpha * (commands.lift_cmd(:) - obj.lift_omega);
            obj.lift_omega = max(0, min(obj.params.lift_omega_max, obj.lift_omega));

            obj.push_omega = obj.push_omega + push_alpha * (commands.push_cmd - obj.push_omega);
            obj.push_omega = max(0, min(obj.params.push_omega_max, obj.push_omega));

            % 计算升力旋翼推力和力矩
            [F_lift, M_lift] = obj.compute_lift_forces();

            % 计算推进器推力
            F_push = obj.compute_push_force();

            % 计算气动力
            [F_aero, M_aero] = obj.compute_aero(V_body, omega);

            % 总外力和力矩
            F_body = F_lift + F_push + F_aero;
            M_body = M_lift + M_aero;

            % 平动方程
            F_world = R * F_body;
            gravity = [0; 0; -obj.params.mass * obj.params.g];
            accel = (F_world + gravity) / obj.params.mass;

            % 转动方程
            I = obj.params.I;
            gyro_torque = cross(omega, I * omega);
            omega_dot = I \ (M_body - gyro_torque);

            % 姿态导数
            W = obj.angular_velocity_matrix(phi, theta);
            att_dot = W * omega;

            % 组装
            state_dot = zeros(12, 1);
            state_dot(1:3) = vel;
            state_dot(4:6) = accel;
            state_dot(7:9) = att_dot;
            state_dot(10:12) = omega_dot;

            % 能耗
            obj.compute_energy();
        end

        function [F_lift, M_lift] = compute_lift_forces(obj)
        %COMPUTE_LIFT_forces  计算升力旋翼推力和力矩
            F_lift = zeros(3, 1);
            M_lift = zeros(3, 1);

            for i = 1:obj.params.n_lift
                F_mag = obj.params.lift_kt * obj.lift_omega(i)^2;
                F_i = [0; 0; -F_mag];  % 向下推力 (Z 向下为正)

                pos_i = obj.params.lift_pos(i, :)';
                M_thrust = cross(pos_i, F_i);
                M_torque = obj.params.lift_kd * obj.lift_omega(i)^2 * ...
                           [0; 0; obj.params.lift_dir(i)];

                F_lift = F_lift + F_i;
                M_lift = M_lift + M_thrust + M_torque;
            end
        end

        function F_push = compute_push_force(obj)
        %COMPUTE_PUSH_force  计算推进器推力
            F_mag = obj.params.push_kt * obj.push_omega^2;
            F_push = [F_mag; 0; 0];  % 沿机头 X 轴
        end

        function [F_aero, M_aero] = compute_aero(obj, V_body, omega)
        %COMPUTE_aero  计算气动力
            V = norm(V_body);
            if V < 1.0
                F_aero = zeros(3, 1);
                M_aero = zeros(3, 1);
                return;
            end

            qbar = 0.5 * obj.params.rho * V^2 * obj.params.wing_area;

            alpha = atan2(V_body(3), V_body(1));
            beta = asin(max(-1, min(1, V_body(2) / V)));

            CL = obj.params.CL0 + obj.params.CL_alpha * alpha;
            CD = obj.params.CD0 + CL^2 / (pi * obj.params.e_oswald * obj.params.AR);

            L = qbar * CL;
            D = qbar * CD;
            Y = qbar * (-0.3) * beta;

            % 气动力 (机体系)
            ca = cos(alpha); sa = sin(alpha);
            F_aero = [-D*ca - L*sa; Y; D*sa - L*ca];

            % 气动力矩 (简化)
            p_hat = omega(1) * obj.params.wing_span / (2 * V);
            q_hat = omega(2) * obj.params.chord / (2 * V);

            M_roll = qbar * obj.params.wing_area * obj.params.wing_span * (-0.5 * p_hat);
            M_pitch = qbar * obj.params.wing_area * obj.params.chord * (-1.0 * alpha + q_hat * (-15));
            M_yaw = qbar * obj.params.wing_area * obj.params.wing_span * (0.06 * beta);

            M_aero = [M_roll; M_pitch; M_yaw];
        end

        function compute_energy(obj)
        %COMPUTE_energy  计算能耗
            dt = obj.sim_dt;
            eta = obj.params.motor_efficiency * obj.params.prop_efficiency;

            P_lift = sum(obj.params.lift_kt * obj.lift_omega.^3) / eta;
            P_push = obj.params.push_kt * obj.push_omega^3 / eta;

            obj.energy_used = obj.energy_used + (P_lift + P_push) * dt / 3600;
        end

        function [alpha, beta, V] = get_aero_state(obj)
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
