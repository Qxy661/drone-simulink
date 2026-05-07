classdef WindModel < handle
%WINDMODEL  风扰模型 (高保真版)
%   常值风 + Dryden 湍流模型
%
%   Dryden 模型: MIL-F-8785C 标准
%   通过成形滤波器将白噪声转换为符合大气湍流频谱的风速序列
%
%   参考:
%     [1] MIL-F-8785C, Flying Qualities of Piloted Airplanes
%     [2] Beard & McLain, "Small Unmanned Aircraft", Ch. 4
%     [3] "Dryden Wind Turbulence Model" - MathWorks Aerospace Blockset

    properties
        wind_speed      % 常值风速 [3x1] (m/s, 世界系)
        gust_state      % 阵风状态 [3x1] (一阶马尔科夫)
        params          % 参数
        turbulence_type % 'none', 'markov', 'dryden'
        dryden_state    % Dryden 滤波器状态
        V_airspeed      % 当前空速 (用于 Dryden 参数计算)
        altitude        % 当前高度 (用于湍流强度)
    end

    methods
        function obj = WindModel()
            obj.wind_speed = zeros(3, 1);
            obj.gust_state = zeros(3, 1);
            obj.turbulence_type = 'markov';
            obj.dryden_state = zeros(6, 1);  % 6 个滤波器状态
            obj.V_airspeed = 0;
            obj.altitude = 0;
        end

        function set_constant(obj, wind_vec)
        %SET_CONSTANT  设置常值风
            obj.wind_speed = wind_vec(:);
        end

        function set_turbulence_type(obj, type)
        %SET_TURBULENCE_TYPE  设置湍流模型类型
        %   type: 'none', 'markov', 'dryden'
            obj.turbulence_type = type;
        end

        function wind = update(obj, dt)
        %UPDATE  更新风速
        %   输出:
        %     wind - 3x1 当前风速 [m/s] (世界系)

            switch obj.turbulence_type
                case 'none'
                    gust = zeros(3, 1);
                case 'markov'
                    gust = obj.update_markov(dt);
                case 'dryden'
                    gust = obj.update_dryden(dt);
                otherwise
                    gust = zeros(3, 1);
            end

            wind = obj.wind_speed + gust;
        end

        function gust = update_markov(obj, dt)
        %UPDATE_MARKOV  一阶马尔科夫阵风
            tau_gust = 2.0;
            sigma_gust = 0.5;

            noise = sigma_gust * randn(3, 1);
            alpha = dt / tau_gust;
            obj.gust_state = (1 - alpha) * obj.gust_state + alpha * noise;
            gust = obj.gust_state;
        end

        function gust = update_dryden(obj, dt)
        %UPDATE_DRYDEN  Dryden 湍流模型 (MIL-F-8785C)
        %   三轴湍流通过独立的成形滤波器生成
        %
        %   传递函数 (简化形式):
        %     u(s)/n(s) = sigma_u * sqrt(2*V/(pi*L_u)) / (1 + (L_u/V)*s)
        %     v(s)/n(s) = sigma_v * sqrt(V/(pi*L_v)) * (1 + sqrt(3)*(L_v/V)*s) / (1 + (L_v/V)*s)^2
        %     w(s)/n(s) = sigma_w * sqrt(V/(pi*L_w)) * (1 + sqrt(3)*(L_w/V)*s) / (1 + (L_w/V)*s)^2
        %
        %   这里用离散状态空间实现

            V = max(obj.V_airspeed, 5);  % 最小 5 m/s 防止除零
            h = max(obj.altitude, 10);   % 最小 10m

            % 湍流强度 (MIL-F-8785C, 低空轻度湍流)
            sigma = obj.turbulence_intensity(h);

            % 湍流长度尺度
            if h > 530  % > 530 ft (~160m)
                L_u = 530; L_v = 530; L_w = 530;
            else
                L_u = h / (0.177 + 0.000823*h)^1.2;  % ft
                L_v = L_u;
                L_w = h;
            end
            % 转换为米 (MIL-F-8785C 使用英制)
            L_u = L_u * 0.3048;
            L_v = L_v * 0.3048;
            L_w = L_w * 0.3048;

            % 状态更新 (离散化的一阶滤波器)
            % 轴 1: u (纵向)
            tau_u = L_u / V;
            a_u = exp(-dt / tau_u);
            K_u = sigma.u * sqrt(2 * V / (pi * L_u));
            obj.dryden_state(1) = a_u * obj.dryden_state(1) + K_u * (1 - a_u) * randn;

            % 轴 2: v (横向)
            tau_v = L_v / V;
            a_v = exp(-dt / tau_v);
            K_v = sigma.v * sqrt(V / (pi * L_v));
            obj.dryden_state(2) = a_v * obj.dryden_state(2) + K_v * (1 - a_v) * randn;
            obj.dryden_state(3) = a_v * obj.dryden_state(3) + K_v * sqrt(3) * (1 - a_v) * randn;

            % 轴 3: w (垂直)
            tau_w = L_w / V;
            a_w = exp(-dt / tau_w);
            K_w = sigma.w * sqrt(V / (pi * L_w));
            obj.dryden_state(4) = a_w * obj.dryden_state(4) + K_w * (1 - a_w) * randn;
            obj.dryden_state(5) = a_w * obj.dryden_state(5) + K_w * sqrt(3) * (1 - a_w) * randn;

            gust = [obj.dryden_state(1);
                    obj.dryden_state(2) + obj.dryden_state(3);
                    obj.dryden_state(4) + obj.dryden_state(5)];
        end

        function sigma = turbulence_intensity(obj, h)
        %TURBULENCE_INTENSITY  湍流强度 (MIL-F-8785C)
        %   输入: h - 高度 [ft]
        %   输出: sigma - 结构体 {u, v, w} [m/s]
            if h < 1000
                % 低空轻度
                W20 = 15;  % 20ft 高度风速 [kts], 轻度湍流
                sigma_u = 0.1 * W20 * 0.5144;  % kts -> m/s
                sigma_v = sigma_u;
                sigma_w = sigma_u;
            else
                % 高空 (简化)
                sigma_u = 1.0;
                sigma_v = 1.0;
                sigma_w = 0.7;
            end
            sigma = struct('u', sigma_u, 'v', sigma_v, 'w', sigma_w);
        end
    end
end
