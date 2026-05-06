function test_dynamics()
%TEST_DYNAMICS  动力学模块单元测试
%
%   运行: init_project; test_dynamics

    init_project;
    fprintf('=== 动力学模块测试 ===\n\n');

    passed = 0;
    failed = 0;

    % 测试 1: 静止状态不受力应下落
    try
        dyn = QuadcopterDynamics(quad);
        dyn.reset([0;0;5], [0;0;0], [0;0;0], [0;0;0]);
        state_dot = dyn.dynamics(0, dyn.state, [0;0;0], [0;0;0]);
        assert(state_dot(6) < 0, '应受重力下落');
        assert(abs(state_dot(6) - (-quad.g)) < 0.01, '加速度应为 -g');
        fprintf('[PASS] 静止受重力下落\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 静止受重力下落: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 2: 悬停推力应平衡重力
    try
        dyn = QuadcopterDynamics(quad);
        dyn.reset([0;0;5], [0;0;0], [0;0;0], [0;0;0]);
        hover_thrust = quad.mass * quad.g;
        state_dot = dyn.dynamics(0, dyn.state, [0;0;hover_thrust], [0;0;0]);
        assert(abs(state_dot(6)) < 0.01, '垂直加速度应接近 0');
        fprintf('[PASS] 悬停推力平衡重力\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 悬停推力平衡重力: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 3: 旋转矩阵正交性
    try
        R = euler_to_rotation(0.1, 0.2, 0.3);
        assert(norm(R' * R - eye(3)) < 1e-10, 'R 应为正交矩阵');
        assert(abs(det(R) - 1) < 1e-10, 'det(R) 应为 1');
        fprintf('[PASS] 旋转矩阵正交性\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 旋转矩阵正交性: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 4: 旋转矩阵单位元
    try
        R = euler_to_rotation(0, 0, 0);
        assert(norm(R - eye(3)) < 1e-10, '零角度应为单位矩阵');
        fprintf('[PASS] 旋转矩阵单位元\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 旋转矩阵单位元: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 5: 混控器正反一致性
    try
        motors = MotorModel(motor);
        mixer_obj = Mixer(quad, motor);

        % 设定期望力/力矩
        hover_T = quad.mass * quad.g;
        commands = [hover_T; 0; 0; 0];

        % 混控 -> 转速 -> 正向计算
        omega = mixer_obj.mix_to_speed(commands);
        commands_back = mixer_obj.forward_mix(omega);

        assert(norm(commands - commands_back) < 0.01, '混控正反应一致');
        fprintf('[PASS] 混控器正反一致性\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 混控器正反一致性: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 6: 电机一阶响应
    try
        motors = MotorModel(motor);
        motors.reset();

        omega_cmd = [500; 500; 500; 500];
        dt = motor.tau_m * 0.1;

        % 一步更新
        [F, M] = motors.update(omega_cmd, dt);

        % 应该未达到命令值
        assert(all(motors.omega < omega_cmd), '一阶响应不应瞬时到达');
        fprintf('[PASS] 电机一阶响应\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 电机一阶响应: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 7: 电机限幅
    try
        motors = MotorModel(motor);
        motors.reset();

        omega_cmd = [2000; 2000; 2000; 2000];  % 超过最大值
        [F, M] = motors.steady(omega_cmd);

        assert(all(motors.omega <= motor.omega_max), '转速应被限幅');
        fprintf('[PASS] 电机限幅\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 电机限幅: %s\n', e.message);
        failed = failed + 1;
    end

    fprintf('\n=== 结果: %d 通过, %d 失败 ===\n', passed, failed);
end
