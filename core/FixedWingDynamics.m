classdef FixedWingDynamics < DroneDynamics
%FIXEDWINGDYNAMICS  固定翼 6DOF 动力学模型
%   状态: [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r]
%   参考: Beard & McLain, "Small Unmanned Aircraft", Ch. 4
%
%   与四旋翼动力学的区别:
%     - 气动力替代旋翼推力
%     - 舵面替代力矩直接控制
%     - 不能悬停 (最低速度约束)

    properties
        aero        % AeroModel 对象
        surfaces    % 舵面偏转
    end

    methods
        function obj = FixedWingDynamics(fw_params, aero_params)
            obj@DroneDynamics(fw_params);
            obj.aero = AeroModel(aero_params, fw_params);
            obj.surfaces = struct('elevator', 0, 'aileron', 0, 'rudder', 0);
        end

        function set_surfaces(obj, elevator, aileron, rudder)
        %SET_SURFACES  设置舵面偏转
            obj.surfaces.elevator = max(-obj.params.elevator_max, ...
                min(obj.params.elevator_max, elevator));
            obj.surfaces.aileron = max(-obj.params.aileron_max, ...
                min(obj.params.aileron_max, aileron));
            obj.surfaces.rudder = max(-obj.params.rudder_max, ...
                min(obj.params.rudder_max, rudder));
        end

        function state_dot = dynamics(obj, ~, state, F_thrust, ~)
        %DYNAMICS  固定翼状态微分方程
        %   输入:
        %     state    - 12x1 [pos; vel; att; rates]
        %     F_thrust - 3x1 推力 (机体系, 通常沿 X 轴)
        %     ~        - 不使用 (力矩由气动计算)
        %   输出:
        %     state_dot - 12x1 状态导数

            % 提取状态
            vel = state(4:6);              % 世界系速度
            att = state(7:9);              % 姿态 [phi, theta, psi]
            omega = state(10:12);          % 角速度 [p, q, r]

            phi = att(1);
            theta = att(2);

            % 旋转矩阵 (body -> world)
            R = obj.euler_to_rotation(phi, theta, att(3));

            % 机体速度 (从世界系转换)
            V_body = R' * vel;

            % 气动力和力矩
            [F_aero, M_aero] = obj.aero.compute(V_body, omega, obj.surfaces);

            % 总外力 (机体系): 推力 + 气动力
            F_body = F_thrust(:) + F_aero;

            % === 平动方程 ===
            F_world = R * F_body;
            gravity = [0; 0; -obj.params.mass * obj.params.g];
            accel = (F_world + gravity) / obj.params.mass;

            % === 转动方程 ===
            I = obj.params.I;
            gyro_torque = cross(omega, I * omega);
            omega_dot = I \ (M_aero - gyro_torque);

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

        function [alpha, beta, V] = get_aero_state(obj)
        %GET_AERO_state  获取当前气流角
            R = obj.euler_to_rotation(obj.state(7), obj.state(8), obj.state(9));
            V_body = R' * obj.state(4:6);
            [alpha, beta, V] = obj.aero.wind_angles(V_body);
        end
    end

    methods (Static)
        function W = angular_velocity_matrix(phi, theta)
        %ANGULAR_VELOCITY_MATRIX  角速度 -> 欧拉角导数转换矩阵
            sp = sin(phi);
            cp = cos(phi);
            tt = tan(theta);
            ct = cos(theta);

            W = [1, sp*tt, cp*tt;
                 0, cp,    -sp;
                 0, sp/ct,  cp/ct];
        end
    end
end
