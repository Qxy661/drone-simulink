classdef WindModel < handle
%WINDMODEL  风扰模型
%   常值风 + 一阶马尔科夫阵风

    properties
        wind_speed      % 当前风速 [3x1] (m/s, 世界系)
        gust_state      % 阵风状态 [3x1]
        params          % 参数
    end

    methods
        function obj = WindModel()
            obj.wind_speed = zeros(3, 1);
            obj.gust_state = zeros(3, 1);
        end

        function set_constant(obj, wind_vec)
        %SET_CONSTANT  设置常值风
            obj.wind_speed = wind_vec(:);
        end

        function wind = update(obj, dt)
        %UPDATE  更新风速 (常值 + 阵风)
        %   输出:
        %     wind - 3x1 当前风速 [m/s]

            % 一阶马尔科夫阵风
            % tau * gust_dot + gust = sigma * white_noise
            tau_gust = 2.0;  % 阵风时间常数 [s]
            sigma_gust = 0.5; % 阵风强度 [m/s]

            noise = sigma_gust * randn(3, 1);
            alpha = dt / tau_gust;
            obj.gust_state = (1 - alpha) * obj.gust_state + alpha * noise;

            wind = obj.wind_speed + obj.gust_state;
        end
    end
end
