% SENSOR_PARAMS  传感器模型参数
%   用于向仿真添加噪声和延迟

%% IMU (加速度计 + 陀螺仪)
sensor.accel_noise = 0.02;       % 加速度噪声标准差 [m/s²]
sensor.accel_bias = 0.05;        % 加速度计零偏 [m/s²]
sensor.accel_bias_drift = 0.001; % 零偏漂移率 [m/s²/s]

sensor.gyro_noise = 0.01;        % 陀螺仪噪声标准差 [rad/s]
sensor.gyro_bias = 0.01;         % 陀螺仪零偏 [rad/s]
sensor.gyro_bias_drift = 0.0005; % 零偏漂移率 [rad/s/s]

%% 气压计
sensor.baro_noise = 0.1;         % 气压计噪声 [m]
sensor.baro_bias = 0.0;          % 气压计零偏 [m]

%% GPS (可选, 用于室外)
sensor.gps_noise_xy = 1.5;       % GPS 水平噪声 [m]
sensor.gps_noise_z = 3.0;        % GPS 垂直噪声 [m]
sensor.gps_rate = 5;             % GPS 更新率 [Hz]

%% 采样时间
sensor.imu_dt = 0.004;           % IMU 采样周期 [s] (250Hz)
sensor.baro_dt = 0.05;           % 气压计采样周期 [s] (20Hz)
