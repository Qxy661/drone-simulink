classdef Mixer < handle
%MIXER  四旋翼混控器
%   将 [总推力, 滚转力矩, 俯仰力矩, 偏航力矩] 转换为 4 个电机转速
%   X 型布局:
%     1(CCW)    2(CW)
%         \   /
%     ---- + ----
%         /   \
%     4(CW)    3(CCW)

    properties
        mixer_matrix    % 4x4 混控矩阵
        mixer_inv       % 逆矩阵
        params          % 参数引用
    end

    methods
        function obj = Mixer(quad_params, motor_params)
            obj.params = struct('quad', quad_params, 'motor', motor_params);

            l = quad_params.arm_length;
            kt = motor_params.k_t;
            kd = motor_params.k_d;

            % 混控矩阵: [T; tau_phi; tau_theta; tau_psi] = M * [w1^2; w2^2; w3^2; w4^2]
            obj.mixer_matrix = [
                kt,       kt,       kt,       kt;
                0,        l*kt,     0,       -l*kt;
               -l*kt,     0,        l*kt,     0;
                kd,      -kd,       kd,      -kd
            ];

            obj.mixer_inv = inv(obj.mixer_matrix);
        end

        function omega_sq = mix(obj, commands)
        %MIX  混控: 力/力矩命令 -> 电机转速平方
        %   输入:
        %     commands - 4x1 [T; tau_phi; tau_theta; tau_psi]
        %   输出:
        %     omega_sq - 4x1 各电机转速平方 (rad/s)^2

            omega_sq = obj.mixer_inv * commands(:);

            % 限幅: 转速平方不能为负
            omega_sq = max(0, omega_sq);
        end

        function omega = mix_to_speed(obj, commands)
        %MIX_TO_SPEED  混控并开方得到转速
            omega_sq = obj.mix(commands);
            omega = sqrt(omega_sq);
        end

        function commands = forward_mix(obj, omega)
        %FORWARD_MIX  正向计算: 电机转速 -> 力/力矩 (用于验证)
            omega_sq = omega(:).^2;
            commands = obj.mixer_matrix * omega_sq;
        end
    end
end
