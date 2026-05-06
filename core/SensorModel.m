classdef SensorModel < handle
%SENSORMODEL  传感器模型
%   为真实状态添加噪声和零偏

    properties
        params      % 传感器参数
        accel_bias  % 当前加速度计零偏
        gyro_bias   % 当前陀螺仪零偏
    end

    methods
        function obj = SensorModel(sensor_params)
            obj.params = sensor_params;
            obj.accel_bias = zeros(3, 1);
            obj.gyro_bias = zeros(3, 1);
        end

        function reset(obj)
        %RESET  重置零偏
            obj.accel_bias = zeros(3, 1);
            obj.gyro_bias = zeros(3, 1);
        end

        function [accel_meas, gyro_meas] = imu_measure(obj, true_accel, true_gyro, dt)
        %IMU_MEASURE  IMU 测量 (加速度计 + 陀螺仪)
        %   输入:
        %     true_accel - 3x1 真实加速度 (机体系)
        %     true_gyro  - 3x1 真实角速度 (机体系)
        %     dt         - 时间步长 (s)
        %   输出:
        %     accel_meas - 3x1 测量加速度
        %     gyro_meas  - 3x1 测量角速度

            % 零偏漂移
            obj.accel_bias = obj.accel_bias + ...
                obj.params.accel_bias_drift * dt * randn(3, 1);
            obj.gyro_bias = obj.gyro_bias + ...
                obj.params.gyro_bias_drift * dt * randn(3, 1);

            % 测量 = 真实值 + 零偏 + 噪声
            accel_meas = true_accel + obj.accel_bias + ...
                obj.params.accel_noise * randn(3, 1);
            gyro_meas = true_gyro + obj.gyro_bias + ...
                obj.params.gyro_noise * randn(3, 1);
        end

        function z_baro = baro_measure(obj, true_alt)
        %BARO_MEASURE  气压计测量
            z_baro = true_alt + obj.params.baro_bias + ...
                obj.params.baro_noise * randn();
        end
    end
end
