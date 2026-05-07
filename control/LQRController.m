classdef LQRController < handle
%LQRCONTROLLER  LQR (线性二次调节器) 姿态控制器
%   基于最优控制理论, 最小化状态误差和控制能量的加权和
%
%   状态: x = [φ, θ, ψ, p, q, r]ᵀ
%   控制: u = [τ_φ, τ_θ, τ_ψ]ᵀ
%
%   线性化模型 (悬停点):
%     ẋ = A·x + B·u
%     A = [0₃ₓ₃  I₃;  0₃ₓ₃  0₃ₓ₃]  (简化: 忽略陀螺耦合)
%     B = [0₃ₓ₃;  I⁻¹]
%
%   性能指标:
%     J = ∫(xᵀQx + uᵀRu) dt
%
%   最优反馈:
%     u = -K·x,  K = R⁻¹BᵀP
%     P 来自代数 Riccati 方程: AᵀP + PA - PBR⁻¹BᵀP + Q = 0
%
%   参考:
%     [1] Franklin, Powell, Emami-Naeini, "Feedback Control of Dynamic Systems"
%     [2] Beard & McLain, "Small Unmanned Aircraft", Ch. 6

    properties
        K           % 最优反馈增益矩阵 3x6
        Q           % 状态权重矩阵 6x6
        R           % 控制权重矩阵 3x3
        I_inv       % 惯量矩阵逆 3x3
        params      % 控制器参数
        dt          % 时间步长

        % 积分项 (可选, 用于消除稳态误差)
        use_integral = false
        integral     % 积分累积 3x1
        Ki           % 积分增益 3x1
        int_max      % 积分限幅
    end

    methods
        function obj = LQRController(quad_params, ctrl_params, dt)
        %LQRCONTROLLER  构造函数
        %   输入:
        %     quad_params  - 四旋翼参数 (含惯量 I)
        %     ctrl_params  - 控制器参数 (含 Q, R 权重)
        %     dt           - 时间步长

            obj.params = ctrl_params;
            obj.dt = dt;

            % 惯量矩阵逆
            I = quad_params.I;
            obj.I_inv = inv(I);

            % 默认 LQR 权重
            if isfield(ctrl_params, 'lqr')
                lqr_params = ctrl_params.lqr;
            else
                lqr_params = struct();
            end

            % 状态权重: [φ, θ, ψ, p, q, r]
            if isfield(lqr_params, 'Q')
                obj.Q = lqr_params.Q;
            else
                obj.Q = diag([10, 10, 5, 1, 1, 0.5]);
            end

            % 控制权重: [τ_φ, τ_θ, τ_ψ]
            if isfield(lqr_params, 'R')
                obj.R = lqr_params.R;
            else
                obj.R = diag([1, 1, 1]);
            end

            % 计算最优增益
            obj.K = obj.compute_lqr_gain();

            % 积分控制 (可选)
            if isfield(lqr_params, 'use_integral')
                obj.use_integral = lqr_params.use_integral;
            end
            if obj.use_integral
                obj.Ki = lqr_params.Ki;
                obj.integral = zeros(3, 1);
                if isfield(lqr_params, 'int_max')
                    obj.int_max = lqr_params.int_max;
                else
                    obj.int_max = 5;
                end
            end
        end

        function K = compute_lqr_gain(obj)
        %COMPUTE_LQR_GAIN  计算 LQR 最优增益
        %   求解代数 Riccati 方程 (ARE):
        %     AᵀP + PA - PBR⁻¹BᵀP + Q = 0
        %   然后: K = R⁻¹BᵀP

            % 线性化系统 (悬停点)
            % ẋ = Ax + Bu
            % x = [φ, θ, ψ, p, q, r]
            % u = [τ_φ, τ_θ, τ_ψ]

            % A 矩阵: 姿态导数 = 角速度, 角速度导数 = I⁻¹·τ
            A = zeros(6, 6);
            A(1,4) = 1;  % φ_dot = p (小角度)
            A(2,5) = 1;  % θ_dot = q
            A(3,6) = 1;  % ψ_dot = r
            % 角速度导数由 B 矩阵处理

            % B 矩阵: [0₃ₓ₃; I⁻¹]
            B = zeros(6, 3);
            B(4:6, :) = obj.I_inv;

            % 求解 Riccati 方程
            % 使用 MATLAB 的 care 函数 (连续时间 ARE)
            % AᵀP + PA - PBR⁻¹BᵀP + Q = 0
            try
                [P, ~, K] = care(A, B, obj.Q, obj.R);
                K = -K;  % care 返回 K = R⁻¹BᵀP, 需要取负
            catch
                % 如果 care 不可用, 使用离散时间方法
                % 或简化的增益矩阵
                warning('care 函数不可用, 使用简化的 LQR 增益');
                K = obj.simplified_gain(A, B);
            end
        end

        function K = simplified_gain(obj, A, B)
        %SIMPLIFIED_GAIN  简化的 LQR 增益计算
        %   当 care 不可用时的备用方案
        %   使用 Bryson 规则: K = Bᵀ·Q / (Bᵀ·B·R)

            % 基于 Bryson 规则的简化增益
            K = zeros(3, 6);

            % 比例增益 (角度误差 → 力矩)
            K(1,1) = sqrt(obj.Q(1,1) / obj.R(1,1));  % φ
            K(2,2) = sqrt(obj.Q(2,2) / obj.R(2,2));  % θ
            K(3,3) = sqrt(obj.Q(3,3) / obj.R(3,3));  % ψ

            % 微分增益 (角速度 → 力矩)
            K(1,4) = sqrt(obj.Q(4,4) / obj.R(1,1));  % p
            K(2,5) = sqrt(obj.Q(5,5) / obj.R(2,2));  % q
            K(3,6) = sqrt(obj.Q(6,6) / obj.R(3,3));  % r
        end

        function torques = update(obj, att_des, att_cur, rates_cur)
        %UPDATE  LQR 姿态控制
        %   输入:
        %     att_des   - 3x1 期望姿态 [φ_d; θ_d; ψ_d]
        %     att_cur   - 3x1 当前姿态 [φ; θ; ψ]
        %     rates_cur - 3x1 当前角速度 [p; q; r]
        %   输出:
        %     torques   - 3x1 力矩命令 [τ_φ; τ_θ; τ_ψ]

            % 状态误差
            att_err = att_des(:) - att_cur(:);
            att_err = mod(att_err + pi, 2*pi) - pi;

            % 状态向量 x = [att_err; rates_cur]
            x = [att_err; rates_cur(:)];

            % LQR 最优控制: u = -K·x
            torques = -obj.K * x;

            % 积分项 (可选, 消除稳态误差)
            if obj.use_integral
                obj.integral = obj.integral + att_err * obj.dt;
                obj.integral = max(-obj.int_max, min(obj.int_max, obj.integral));
                torques = torques + obj.Ki .* obj.integral;
            end

            % 输出限幅
            max_torque = obj.params.att.max_torque;
            torques = max(-max_torque, min(max_torque, torques));
        end

        function set_weights(obj, Q, R)
        %SET_WEIGHTS  更新 LQR 权重矩阵并重新计算增益
        %   输入:
        %     Q - 6x6 状态权重矩阵
        %     R - 3x3 控制权重矩阵
            obj.Q = Q;
            obj.R = R;
            obj.K = obj.compute_lqr_gain();
        end

        function reset(obj)
        %RESET  重置积分器
            if obj.use_integral
                obj.integral = zeros(3, 1);
            end
        end
    end
end
