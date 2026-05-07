classdef QuadcopterDynamics < DroneDynamics
%QUADCOPTERDYNAMICS  四旋翼 6DOF 动力学模型 (高保真版)
%   状态: [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r]  (12 维, Euler)
%   或:   [x,y,z, vx,vy,vz, qw,qx,qy,qz, p,q,r]   (13 维, 四元数)
%
%   高保真物理效应:
%     - 地面效应 (ground effect)
%     - 旋翼阻力 (H-force / rotor drag)
%     - 电池电压衰减
%     - 涡环状态 (VRS) 检测与推力损失
%     - 前飞升力修正 (translational lift)
%     - 四元数姿态表示 (避免万向锁)
%
%   参考:
%     [1] Beard & McLain, "Small Unmanned Aircraft", Ch. 3
%     [2] PX4 Dev Guide - Rotor Dynamics
%     [3] Leishman, "Principles of Helicopter Aerodynamics"

    properties
        use_quaternion = false   % 是否使用四元数模式
        battery_voltage          % 当前电池电压 [V]
        battery_capacity         % 电池容量 [mAh]
        V_nominal                % 标称电压 [V]
    end

    methods
        function obj = QuadcopterDynamics(quad_params)
            obj@DroneDynamics(quad_params);
            obj.use_quaternion = false;
            % 电池参数 (可选)
            if isfield(quad_params, 'V_nominal')
                obj.V_nominal = quad_params.V_nominal;
                obj.battery_voltage = quad_params.V_nominal;
            else
                obj.V_nominal = 14.8;  % 4S LiPo
                obj.battery_voltage = 14.8;
            end
            if isfield(quad_params, 'battery_capacity')
                obj.battery_capacity = quad_params.battery_capacity;
            else
                obj.battery_capacity = 5000;  % mAh
            end
        end

        function reset(obj, pos, vel, att, rates)
        %RESET  重置状态
            if obj.use_quaternion
                % att 作为四元数 [qw;qx;qy;qz] 或欧拉角 [phi;theta;psi]
                if length(att) == 3
                    q = quaternion_ops('from_euler', att(1), att(2), att(3));
                else
                    q = quaternion_ops('normalize', att(:));
                end
                obj.state = zeros(13, 1);
                obj.state(1:3) = pos(:);
                obj.state(4:6) = vel(:);
                obj.state(7:10) = q(:);
                obj.state(11:13) = rates(:);
            else
                obj.state = zeros(12, 1);
                obj.state(1:3) = pos(:);
                obj.state(4:6) = vel(:);
                obj.state(7:9) = att(:);
                obj.state(10:12) = rates(:);
            end
        end

        function state_dot = dynamics(obj, ~, state, forces, moments)
        %DYNAMICS  四旋翼状态微分方程 (Euler 模式)
        %   输入:
        %     state   - 12x1 [pos; vel; att; rates]
        %     forces  - 3x1 外力 (机体系)
        %     moments - 3x1 外力矩 (机体系)
        %   输出:
        %     state_dot - 12x1 状态导数

            % 提取状态
            vel = state(4:6);
            att = state(7:9);
            omega = state(10:12);

            phi = att(1); theta = att(2); psi = att(3);

            % 旋转矩阵 (body -> world)
            R = obj.euler_to_rotation(phi, theta, psi);

            % === 平动方程 ===
            F_world = R * forces;
            gravity = [0; 0; -obj.params.mass * obj.params.g];
            accel = (F_world + gravity) / obj.params.mass;

            % === 转动方程 ===
            I = obj.params.I;
            gyro_torque = cross(omega, I * omega);
            omega_dot = I \ (moments - gyro_torque);

            % === 姿态导数 ===
            W = obj.angular_velocity_matrix(phi, theta);
            att_dot = W * omega;

            % === 组装状态导数 ===
            state_dot = zeros(12, 1);
            state_dot(1:3) = vel;
            state_dot(4:6) = accel;
            state_dot(7:9) = att_dot;
            state_dot(10:12) = omega_dot;
        end

        function state_dot = dynamics_quat(obj, ~, state, forces, moments)
        %DYNAMICS_QUAT  四旋翼状态微分方程 (四元数模式)
        %   输入:
        %     state   - 13x1 [pos; vel; quat; rates]
        %     forces  - 3x1 外力 (机体系)
        %     moments - 3x1 外力矩 (机体系)
        %   输出:
        %     state_dot - 13x1 状态导数
        %
        %   优势: 无万向锁, 全姿态域有效, 数值稳定性好

            vel = state(4:6);
            q = state(7:10);
            omega = state(11:13);

            % 旋转矩阵
            R = quaternion_ops('to_rotation', q);

            % 平动
            F_world = R * forces;
            gravity = [0; 0; -obj.params.mass * obj.params.g];
            accel = (F_world + gravity) / obj.params.mass;

            % 转动
            I = obj.params.I;
            gyro_torque = cross(omega, I * omega);
            omega_dot = I \ (moments - gyro_torque);

            % 四元数导数: q_dot = 0.5 * q ⊗ [0; omega]
            q_dot = quaternion_ops('derivative', q, omega);

            state_dot = zeros(13, 1);
            state_dot(1:3) = vel;
            state_dot(4:6) = accel;
            state_dot(7:10) = q_dot;
            state_dot(11:13) = omega_dot;
        end

        function [pos, vel, att, rates, R] = get_state(obj)
        %GET_STATE  获取当前状态 (兼容两种模式)
            if obj.use_quaternion
                pos = obj.state(1:3);
                vel = obj.state(4:6);
                q = obj.state(7:10);
                rates = obj.state(11:13);
                R = quaternion_ops('to_rotation', q);
                [att(1), att(2), att(3)] = quaternion_ops('to_euler', q);
                att = att(:);
            else
                pos = obj.state(1:3);
                vel = obj.state(4:6);
                att = obj.state(7:9);
                rates = obj.state(10:12);
                R = obj.euler_to_rotation(att(1), att(2), att(3));
            end
        end

        function f_ge = ground_effect_factor(obj, z)
        %GROUND_EFFECT_FACTOR  地面效应推力增益因子
        %   f_ge = 1 / (1 - (R/(4*z))^2)
        %   z: 距地面高度 (正值, z>0 表示在地面上方)
        %   当 z -> 0 时, f_ge -> 很大 (需钳位)
        %
        %   参考: Leishman, "Principles of Helicopter Aerodynamics", Ch. 3
            R_prop = obj.params.arm_length;  % 旋翼半径近似
            z = max(z, 0.05);  % 避免除零, 最小 5cm
            ratio = R_prop / (4 * z);
            f_ge = 1 / (1 - ratio^2);
            f_ge = min(f_ge, 1.5);  % 钳位: 最多增加 50%
        end

        function F_drag = rotor_drag(obj, omega_rotors, V_body)
        %ROTOR_DRAG  旋翼阻力 (H-force)
        %   每个旋翼在前飞时产生的水平阻力
        %   F_drag_i = k_drag * omega_i * V_body_xy
        %
        %   参考: PX4 Dev Guide - Rotor Drag Model
            if ~isfield(obj.params, 'k_drag')
                if isfield(obj.params, 'k_t')
                    k_drag = 0.1 * obj.params.k_t;
                else
                    k_drag = 1.5e-6;  % 默认值
                end
            else
                k_drag = obj.params.k_drag;
            end

            n = length(omega_rotors);
            F_drag = zeros(3, 1);
            for i = 1:n
                % 旋翼在机体系中的位置
                r_i = obj.params.positions(i, :)';
                % 该旋翼处的相对速度
                V_local = V_body + cross([0;0;omega_rotors(i)], r_i);
                % H-force (简化: 与转速和前飞速度成正比)
                F_drag = F_drag - k_drag * abs(omega_rotors(i)) * V_local;
            end
        end

        function ct_eff = battery_thrust_coeff(obj, k_t_nominal)
        %BATTERY_THRUST_COEFF  电池电压修正后的推力系数
        %   C_T_eff = C_T_nominal * (V_batt / V_nominal)^2
        %   参考: PX4 Battery Model
            if nargin < 2
                if isfield(obj.params, 'k_t')
                    k_t_nominal = obj.params.k_t;
                else
                    k_t_nominal = 1.5e-5;  % 默认推力系数
                end
            end
            ratio = obj.battery_voltage / obj.V_nominal;
            ct_eff = k_t_nominal * ratio^2;
        end

        function [vrs_active, loss_factor] = detect_vortex_ring(obj, V_body_z, omega_avg)
        %DETECT_VORTEX_RING  涡环状态检测
        %   VRS 条件: 下降速度 > 0.5 * 悬停诱导速度
        %   参考: Leishman, Ch. 3; PX4 VRS avoidance
        %
        %   输入:
        %     V_body_z - 机体系 Z 轴速度 (正 = 向下)
        %     omega_avg - 平均旋翼转速
        %   输出:
        %     vrs_active - 是否处于 VRS
        %     loss_factor - 推力损失因子 [0.5, 1.0]

            % 悬停诱导速度估算: v_i = sqrt(T / (2*rho*A))
            % 简化: v_i ≈ k * omega_avg
            v_hover_induced = 0.01 * omega_avg;  % 经验近似

            vrs_active = false;
            loss_factor = 1.0;

            if V_body_z > 0  % 下降
                v_descent = V_body_z;
                if v_descent > 0.5 * v_hover_induced && v_descent < 2.0 * v_hover_induced
                    % 处于 VRS 区间
                    vrs_active = true;
                    % 推力损失: 在 1.0~1.5 倍诱导速度区间最大损失 ~50%
                    normalized = v_descent / v_hover_induced;
                    if normalized < 1.0
                        loss_factor = 1.0 - 0.5 * (normalized - 0.5) / 0.5;
                    else
                        loss_factor = 0.5 + 0.5 * (normalized - 1.0) / 1.0;
                    end
                    loss_factor = max(0.5, min(1.0, loss_factor));
                end
            end
        end

        function f_tl = translational_lift(obj, V_xy)
        %TRANSLATIONAL_LIFT  前飞升力修正
        %   前飞时旋翼效率增加 (入流比增大)
        %   f_tl = 1 + k * (V_xy / V_tip)^2
        %   V_xy: 水平速度幅值
        %
        %   参考: Leishman, Ch. 1
            V_tip = 100;  % 叶尖速度参考 [m/s], 简化
            k_tl = 0.25;  % 修正系数
            f_tl = 1 + k_tl * (V_xy / V_tip)^2;
            f_tl = min(f_tl, 1.3);  % 最多增加 30%
        end
    end

    methods (Static)
        function W = angular_velocity_matrix(phi, theta)
        %ANGULAR_VELOCITY_MATRIX  角速度 -> 欧拉角导数转换矩阵
            sp = sin(phi);   cp = cos(phi);
            tt = tan(theta); ct = cos(theta);

            W = [1, sp*tt, cp*tt;
                 0, cp,    -sp;
                 0, sp/ct,  cp/ct];
        end
    end
end
