classdef AeroModel < handle
%AEROMODEL  气动模型
%   计算升力、阻力、侧力及力矩
%   参考: Beard & McLain, "Small Unmanned Aircraft", Ch. 4
%
%   气动力在气流坐标系 (wind frame) 中计算:
%     升力 L 垂直于来流
%     阻力 D 平行于来流 (向后)
%     侧力 Y 垂直于 L 和 D
%
%   然后转换到机体系 (body frame) 施加

    properties
        params      % 气动系数
        fw_params   % 固定翼物理参数
    end

    methods
        function obj = AeroModel(aero_params, fw_params)
            obj.params = aero_params;
            obj.fw_params = fw_params;
        end

        function [F_aero, M_aero] = compute(obj, V_body, omega, surfaces)
        %COMPUTE  计算气动力和力矩
        %   输入:
        %     V_body   - 3x1 机体速度 [u; v; w] (m/s)
        %     omega    - 3x1 角速度 [p; q; r] (rad/s)
        %     surfaces - 结构体:
        %       .elevator - 升降舵偏转 [rad]
        %       .aileron  - 副翼偏转 [rad]
        %       .rudder   - 方向舵偏转 [rad]
        %   输出:
        %     F_aero - 3x1 气动力 (机体系) [N]
        %     M_aero - 3x1 气动力矩 (机体系) [N·m]

            u = V_body(1);
            v = V_body(2);
            w = V_body(3);
            p = omega(1);
            q = omega(2);
            r = omega(3);

            % 气流角
            [alpha, beta, V] = obj.wind_angles(V_body);

            % 动压
            qbar = 0.5 * obj.fw_params.rho * V^2;
            S = obj.fw_params.wing_area;
            b = obj.fw_params.wing_span;
            c = obj.fw_params.chord;

            % 舵面偏转
            de = surfaces.elevator;
            da = surfaces.aileron;
            dr = surfaces.rudder;

            % 无量纲角速度
            p_hat = p * b / (2 * V);   % 滚转角速度系数
            q_hat = q * c / (2 * V);   % 俯仰角速度系数
            r_hat = r * b / (2 * V);   % 偏航角速度系数

            % === 升力系数 ===
            CL = obj.CL_alpha(alpha) + obj.params.CL_de * de + obj.params.Cm_q * q_hat;

            % === 阻力系数 ===
            CD = obj.CD_alpha(CL, alpha);

            % === 侧力系数 ===
            CY = obj.params.CY_beta * beta + obj.params.Cl_dr * dr;

            % === 气动力 (气流坐标系) ===
            L = qbar * S * CL;   % 升力
            D = qbar * S * CD;   % 阻力
            Y = qbar * S * CY;   % 侧力

            % 转换到机体系
            F_aero = obj.wind_to_body([L; D; Y], alpha, beta);

            % === 气动力矩 ===
            % 滚转力矩
            Cl = obj.params.Cl_beta * beta + obj.params.Cl_p * p_hat + ...
                 obj.params.Cl_da * da + obj.params.Cl_dr * dr;
            M_roll = qbar * S * b * Cl;

            % 俯仰力矩
            Cm = obj.params.Cm0 + obj.params.Cm_alpha * alpha + ...
                 obj.params.Cm_de * de + obj.params.Cm_q * q_hat;
            M_pitch = qbar * S * c * Cm;

            % 偏航力矩
            Cn = obj.params.Cn_beta * beta + obj.params.Cn_r * r_hat + ...
                 obj.params.Cn_dr * dr + obj.params.Cn_da * da;
            M_yaw = qbar * S * b * Cn;

            M_aero = [M_roll; M_pitch; M_yaw];
        end

        function [alpha, beta, V] = wind_angles(obj, V_body)
        %WIND_angles  计算迎角和侧滑角
        %   alpha = atan2(w, u)     (迎角)
        %   beta  = asin(v / V)     (侧滑角)

            u = V_body(1);
            v = V_body(2);
            w = V_body(3);

            V = norm(V_body);
            if V < 0.1
                alpha = 0;
                beta = 0;
                return;
            end

            alpha = atan2(w, u);
            beta = asin(max(-1, min(1, v / V)));
        end

        function CL = CL_alpha(obj, alpha)
        %CL_ALPHA  升力系数随迎角变化 (C1 连续失速模型)
        %   使用平滑混合函数替代分段线性, 确保 CL 和 dCL/dalpha 连续
        %   参考: Gazebo LiftDrag 插件; Selig 等人低速翼型数据拟合
            a = obj.params;

            % 线性区 (附着流)
            CL_linear = a.CL0 + a.CL_alpha * alpha;

            % 失速后模型 (简化的后失速特性)
            CL_stall = a.CL0 + a.CL_alpha * a.alpha_stall;
            slope_post = -0.5 * a.CL_alpha;  % 后失速斜率 (负值)
            CL_post = CL_stall + slope_post * (alpha - a.alpha_stall);

            % 平滑混合: 在 stall 附近用 sigmoid 过渡
            % 过渡宽度 (越小越接近 C0 连续的分段)
            delta = deg2rad(3);  % ~3° 过渡带
            x = (alpha - a.alpha_stall) / delta;
            % C1 连续 sigmoid: smoothstep-like 权重
            % w = 0 in linear region, w = 1 in post-stall region
            if x <= -1
                w = 0;
            elseif x >= 1
                w = 1;
            else
                w = 0.5 + 0.75*x - 0.25*x^3;  % Hermite smoothstep
            end

            CL = (1 - w) * CL_linear + w * CL_post;
            CL = max(a.CL_min, min(a.CL_max, CL));
        end

        function CD = CD_alpha(obj, CL, alpha)
        %CD_ALPHA  阻力系数 (含诱导阻力)
            a = obj.params;
            % 诱导阻力: CD_i = CL^2 / (pi * e * AR)
            CD_i = CL^2 / (pi * a.e_oswald * obj.fw_params.AR);
            CD = a.CD0 + CD_i + a.CD_alpha * alpha;
        end

        function F_body = wind_to_body(~, F_wind, alpha, beta)
        %WIND_TO_BODY  气流坐标系力 -> 机体系
        %   气流系: X_w = 来流方向, Z_w = 向下, Y_w = 右
        %   机体系: X_b = 前, Y_b = 右, Z_b = 下

            ca = cos(alpha);
            sa = sin(alpha);
            cb = cos(beta);
            sb = sin(beta);

            % 气流系到机体系旋转矩阵
            R_w2b = [ca*cb,  -ca*sb,  -sa;
                     sb,      cb,      0;
                     sa*cb,  -sa*sb,   ca];

            F_body = R_w2b * F_wind;
        end

        function [alpha_trim, de_trim, T_trim] = trim(obj, V, gamma)
        %TRIM  配平计算
        %   输入:
        %     V     - 配平速度 [m/s]
        %     gamma - 爬升角 [rad] (0=平飞)
        %   输出:
        %     alpha_trim - 配平迎角 [rad]
        %     de_trim    - 配平升降舵 [rad]
        %     T_trim     - 配平推力 [N]

            a = obj.params;
            fw = obj.fw_params;

            % 平飞: L = mg*cos(gamma), T = D + mg*sin(gamma)
            L_needed = fw.mass * fw.g * cos(gamma);
            T_drag = fw.mass * fw.g * sin(gamma);

            qbar = 0.5 * fw.rho * V^2 * fw.wing_area;

            % 求配平迎角: CL = L / qbar*S
            CL_trim = L_needed / qbar;
            alpha_trim = (CL_trim - a.CL0) / a.CL_alpha;

            % 配平阻力
            CD_trim = obj.CD_alpha(CL_trim, alpha_trim);
            D_trim = qbar * CD_trim;

            % 配平推力
            T_trim = D_trim + T_drag;
            T_trim = max(fw.T_min, min(fw.T_max, T_trim));

            % 配平升降舵 (消除俯仰力矩: Cm = 0)
            Cm_aero = a.Cm0 + a.Cm_alpha * alpha_trim;
            de_trim = -Cm_aero / a.Cm_de;
            de_trim = max(-fw.elevator_max, min(fw.elevator_max, de_trim));
        end
    end
end
