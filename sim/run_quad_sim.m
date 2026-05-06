function [t_log, state_log, cmd_log] = run_quad_sim(traj, duration, wind_cfg)
%RUN_QUAD_SIM  四旋翼仿真主函数
%
%   [t, state, cmd] = run_quad_sim(traj)
%   [t, state, cmd] = run_quad_sim(traj, 20)
%   [t, state, cmd] = run_quad_sim(traj, 20, wind_cfg)
%
%   输入:
%     traj     - TrajectoryGenerator 对象
%     duration - 仿真时长 [s] (默认 20)
%     wind_cfg - 风扰配置 (可选)
%   输出:
%     t_log     - 时间向量 [Nx1]
%     state_log - 状态矩阵 [Nx12]
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

    % 创建模块
    dynamics = QuadcopterDynamics(quad);
    motors = MotorModel(motor);
    mixer = Mixer(quad, motor);
    sensor = SensorModel(sensor);
    pos_ctrl = PositionController(ctrl, quad, motor, sim.dt);
    att_ctrl = AttitudeController(ctrl, sim.dt);
    wind = WindModel();

    if nargin >= 3 && ~isempty(wind_cfg)
        if isfield(wind_cfg, 'constant')
            wind.set_constant(wind_cfg.constant);
        end
    end

    % 初始化状态
    dynamics.reset(sim.init_pos, sim.init_vel, sim.init_att, sim.init_rates);
    motors.reset();

    % 预分配日志
    N = ceil(duration / sim.dt);
    t_log = zeros(N, 1);
    state_log = zeros(N, 12);
    cmd_log = zeros(N, 4);

    % 记录初始状态
    t_log(1) = 0;
    state_log(1, :) = dynamics.state';

    fprintf('开始仿真: duration=%.1fs, dt=%.4fs\n', duration, sim.dt);

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
            torques = att_ctrl.update(att_des, state(7:9), state(10:12));
        end

        % 混控器
        commands = [thrust; torques];
        omega_cmd = mixer.mix_to_speed(commands);

        % 电机更新
        [F_motors, M_motors] = motors.update(omega_cmd, sim.dt);

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
            R = euler_to_rotation(state(7), state(8), state(9));
            F_wind = R' * wind_vel * 0.1;  % 简化风阻
            F_body = F_body + F_wind;
        end

        % 动力学积分 (RK4)
        k1 = dynamics.dynamics(t, state, F_body, M_body);
        k2 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k1, F_body, M_body);
        k3 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k2, F_body, M_body);
        k4 = dynamics.dynamics(t + sim.dt, state + sim.dt * k3, F_body, M_body);
        dynamics.state = state + sim.dt/6 * (k1 + 2*k2 + 2*k3 + k4);

        % 地面约束
        if sim.enable_ground && dynamics.state(3) < sim.ground_level
            dynamics.state(3) = sim.ground_level;
            dynamics.state(6) = max(0, dynamics.state(6));
        end

        % 记录
        t_log(i) = t;
        state_log(i, :) = dynamics.state';
        cmd_log(i, :) = commands';
    end

    fprintf('仿真完成: %d 步\n', N);
end
