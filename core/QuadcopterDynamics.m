classdef QuadcopterDynamics < DroneDynamics
%QUADCOPTERDYNAMICS  四旋翼 6DOF 动力学模型
%   状态: [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r]
%   参考: Beard & McLain, "Small Unmanned Aircraft", Ch. 3

    methods
        function obj = QuadcopterDynamics(quad_params)
            obj@DroneDynamics(quad_params);
        end

        function state_dot = dynamics(obj, ~, state, forces, moments)
        %DYNAMICS  四旋翼状态微分方程
        %   输入:
        %     state   - 12x1 [pos; vel; att; rates]
        %     forces  - 3x1 外力 (机体系)
        %     moments - 3x1 外力矩 (机体系)
        %   输出:
        %     state_dot - 12x1 状态导数

            % 提取状态
            vel = state(4:6);              % 速度 (世界系)
            att = state(7:9);              % 姿态 [phi, theta, psi]
            omega = state(10:12);          % 角速度 [p, q, r]

            phi = att(1);
            theta = att(2);
            psi = att(3);

            % 旋转矩阵 (body -> world)
            R = obj.euler_to_rotation(phi, theta, psi);

            % === 平动方程 ===
            % m * a = R * F_body + [0; 0; -m*g]
            F_world = R * forces;
            gravity = [0; 0; -obj.params.mass * obj.params.g];
            accel = (F_world + gravity) / obj.params.mass;

            % === 转动方程 ===
            % I * omega_dot = M - omega x (I * omega)
            I = obj.params.I;
            gyro_torque = cross(omega, I * omega);
            omega_dot = I \ (moments - gyro_torque);

            % === 姿态导数 ===
            % 从角速度到欧拉角导数的转换矩阵
            W = obj.angular_velocity_matrix(phi, theta);
            att_dot = W * omega;

            % === 组装状态导数 ===
            state_dot = zeros(12, 1);
            state_dot(1:3) = vel;          % 位置导数 = 速度
            state_dot(4:6) = accel;        % 速度导数 = 加速度
            state_dot(7:9) = att_dot;      % 姿态导数
            state_dot(10:12) = omega_dot;  % 角速度导数
        end
    end

    methods (Static)
        function W = angular_velocity_matrix(phi, theta)
        %ANGULAR_VELOCITY_MATRIX  角速度 -> 欧拉角导数转换矩阵
        %   att_dot = W * [p; q; r]
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
