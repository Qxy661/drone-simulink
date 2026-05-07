function [t_log, state_log, cmd_log] = run_quad_sim(traj, duration, wind_cfg, fidelity_cfg)
%RUN_QUAD_SIM  四旋翼仿真主函数
%
%   [t, state, cmd] = run_quad_sim(traj)
%   [t, state, cmd] = run_quad_sim(traj, 20)
%   [t, state, cmd] = run_quad_sim(traj, 20, wind_cfg)
%   [t, state, cmd] = run_quad_sim(traj, 20, wind_cfg, fidelity_cfg)
%
%   输入:
%     traj          - TrajectoryGenerator 对象
%     duration      - 仿真时长 [s] (默认 20)
%     wind_cfg      - 风扰配置 (可选)
%       .constant   - 常值风 [3x1]
%       .type       - 'none', 'markov', 'dryden'
%     fidelity_cfg  - 高保真配置 (可选)
%       .ground_effect     - 是否启用地面效应 (默认 false)
%       .battery_decay     - 是否启用电池衰减 (默认 false)
%       .translational_lift - 是否启用前飞升力修正 (默认 false)
%       .use_quaternion    - 是否使用四元数 (默认 false)
%   输出:
%     t_log     - 时间向量 [Nx1]
%     state_log - 状态矩阵 [Nx12] 或 [Nx13] (四元数模式)
%     cmd_log   - 命令矩阵 [Nx4] [T, tau_phi, tau_theta, tau_psi]

    % 加载参数 (需先运行 init_project)
    quad_params;
    motor_params;
    controller_params;
    sensor_params;
    sim_params;

    if nargin < 2 || isempty(duration)
        duration = sim.t_end;
    end

    % 高保真配置
    if nargin < 4
        fidelity_cfg = struct();
    end
    if ~isfield(fidelity_cfg, 'ground_effect'), fidelity_cfg.ground_effect = false; end
    if ~isfield(fidelity_cfg, 'battery_decay'), fidelity_cfg.battery_decay = false; end
    if ~isfield(fidelity_cfg, 'translational_lift'), fidelity_cfg.translational_lift = false; end
    if ~isfield(fidelity_cfg, 'use_quaternion'), fidelity_cfg.use_quaternion = false; end

    % 创建模块
    dynamics = QuadcopterDynamics(quad);
    dynamics.use_quaternion = fidelity_cfg.use_quaternion;
    motors = MotorModel(motor);
    mixer = Mixer(quad, motor);
    sensor = SensorModel(sensor);
    pos_ctrl = PositionController(ctrl, quad, motor, sim.dt);
    att_ctrl = AttitudeController(ctrl, sim.dt);
    wind = WindModel();

    % 风扰配置
    if nargin >= 3 && ~isempty(wind_cfg)
        if isfield(wind_cfg, 'constant')
            wind.set_constant(wind_cfg.constant);
        end
        if isfield(wind_cfg, 'type')
            wind.set_turbulence_type(wind_cfg.type);
        end
    end

    % 初始化状态
    dynamics.reset(sim.init_pos, sim.init_vel, sim.init_att, sim.init_rates);
    motors.reset();

    % 预分配日志
    N = ceil(duration / sim.dt);
    state_size = 13 * fidelity_cfg.use_quaternion + 12 * ~fidelity_cfg.use_quaternion;
    t_log = zeros(N, 1);
    state_log = zeros(N, state_size);
    cmd_log = zeros(N, 4);

    % 记录初始状态
    t_log(1) = 0;
    state_log(1, :) = dynamics.state';

    fprintf('开始仿真: duration=%.1fs, dt=%.4fs\n', duration, sim.dt);
    if fidelity_cfg.ground_effect, fprintf('  [高保真] 地面效应已启用\n'); end
    if fidelity_cfg.battery_decay, fprintf('  [高保真] 电池衰减已启用\n'); end
    if fidelity_cfg.translational_lift, fprintf('  [高保真] 前飞升力修正已启用\n'); end
    if fidelity_cfg.use_quaternion, fprintf('  [高保真] 四元数姿态已启用\n'); end

    for i = 2:N
        t = (i-1) * sim.dt;
        state = dynamics.state;

        % 轨迹生成
        [pos_des, vel_des] = traj.generate(t);

        % 风扰
        wind_vel = wind.update(sim.dt);

        % 位置环 (外环, 降频)
        if mod(i-1, round(1/(sim.dt * ctrl.pos_rate))) == 0
            [att_des, thrust] = pos_ctrl.update(pos_des, state(1:3), state(4:6));
        end

        % 姿态环 (内环, 降频)
        if mod(i-1, round(1/(sim.dt * ctrl.att_rate))) == 0
            if fidelity_cfg.use_quaternion
                % 四元数模式: 从状态提取欧拉角用于控制
                q = state(7:10);
                [phi, theta, psi] = quaternion_ops('to_euler', q);
                att = [phi; theta; psi];
                rates = state(11:13);
            else
                att = state(7:9);
                rates = state(10:12);
            end
            torques = att_ctrl.update(att_des, att, rates);
        end

        % 混控器
        commands = [thrust; torques];
        omega_cmd = mixer.mix_to_speed(commands);

        % 电池电压修正
        if fidelity_cfg.battery_decay
            % 简化: 线性衰减
            elapsed_frac = t / duration;
            dynamics.battery_voltage = dynamics.V_nominal * (1 - 0.2 * elapsed_frac);
            ct_eff = dynamics.battery_thrust_coeff();
            % 调整推力命令以补偿电压下降
            voltage_ratio = ct_eff / quad.k_t;
            if voltage_ratio < 1
                omega_cmd = omega_cmd / sqrt(voltage_ratio);
            end
        end

        % 电机更新
        [F_motors, M_motors] = motors.update(omega_cmd, sim.dt);

        % 地面效应修正
        if fidelity_cfg.ground_effect
            z = state(3);  % 高度 (z > 0 在地面以上)
            if z > 0 && z < 2  % 2m 以内有地面效应
                f_ge = dynamics.ground_effect_factor(z);
                F_motors = F_motors * f_ge;
            end
        end

        % 前飞升力修正
        if fidelity_cfg.translational_lift
            V_xy = norm(state(4:5));  % 水平速度
            if V_xy > 1  % 前飞速度 > 1 m/s
                f_tl = dynamics.translational_lift(V_xy);
                F_motors = F_motors * f_tl;
            end
        end

        % 总推力和力矩 (机体系)
        F_body = [0; 0; sum(F_motors)];

        % 力矩: 滚转、俯仰、偏航
        l = quad.arm_length;
        tau_phi = l * (F_motors(2) - F_motors(4));
        tau_theta = l * (F_motors(3) - F_motors(1));
        tau_psi = sum(M_motors .* quad.motor_directions');
        M_body = [tau_phi; tau_theta; tau_psi];

        % 加入风扰力 (简化: 直接加到机体系)
        if sim.wind_enable
            if fidelity_cfg.use_quaternion
                R = quaternion_ops('to_rotation', state(7:10));
            else
                R = euler_to_rotation(state(7), state(8), state(9));
            end
            F_wind = R' * wind_vel * 0.1;  % 简化风阻
            F_body = F_body + F_wind;
        end

        % 动力学积分 (RK4)
        if fidelity_cfg.use_quaternion
            k1 = dynamics.dynamics_quat(t, state, F_body, M_body);
            k2 = dynamics.dynamics_quat(t + sim.dt/2, state + sim.dt/2 * k1, F_body, M_body);
            k3 = dynamics.dynamics_quat(t + sim.dt/2, state + sim.dt/2 * k2, F_body, M_body);
            k4 = dynamics.dynamics_quat(t + sim.dt, state + sim.dt * k3, F_body, M_body);
            dynamics.state = state + sim.dt/6 * (k1 + 2*k2 + 2*k3 + k4);
            % 归一化四元数
            dynamics.state(7:10) = quaternion_ops('normalize', dynamics.state(7:10));
        else
            k1 = dynamics.dynamics(t, state, F_body, M_body);
            k2 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k1, F_body, M_body);
            k3 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k2, F_body, M_body);
            k4 = dynamics.dynamics(t + sim.dt, state + sim.dt * k3, F_body, M_body);
            dynamics.state = state + sim.dt/6 * (k1 + 2*k2 + 2*k3 + k4);
        end

        % 地面约束
        if sim.enable_ground
            if fidelity_cfg.use_quaternion
                if dynamics.state(3) < sim.ground_level
                    dynamics.state(3) = sim.ground_level;
                    dynamics.state(6) = max(0, dynamics.state(6));
                end
            else
                if dynamics.state(3) < sim.ground_level
                    dynamics.state(3) = sim.ground_level;
                    dynamics.state(6) = max(0, dynamics.state(6));
                end
            end
        end

        % 记录
        t_log(i) = t;
        state_log(i, :) = dynamics.state';
        cmd_log(i, :) = commands';
    end

    fprintf('仿真完成: %d 步\n', N);
end
