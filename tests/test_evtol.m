function test_evtol()
%TEST_evtol  eVTOL 模块测试
%
%   运行: init_project; test_evtol

    init_project;
    fprintf('=== eVTOL 测试 ===\n\n');

    passed = 0;
    failed = 0;

    % 测试 1: 悬停转速计算
    try
        omega_hover = sqrt(evtol.mass * evtol.g / (evtol.n_rotors * evtol.k_t));
        F_hover = evtol.n_rotors * evtol.k_t * omega_hover^2;
        assert(abs(F_hover - evtol.mass * evtol.g) < 0.1, '悬停推力应等于重力');
        fprintf('[PASS] 悬停转速计算\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 悬停转速计算: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 2: 倾转角限幅
    try
        dyn = EvtolDynamics(evtol, aero);
        dyn.set_tilt_cmd([0; 0; 0; 0]);
        assert(all(dyn.tilt_cmd == 0), '悬停倾转角应为 0');

        dyn.set_tilt_cmd([pi; pi; pi; pi]);  % 超限
        assert(all(dyn.tilt_cmd <= evtol.tilt_max), '倾转角应限幅');
        fprintf('[PASS] 倾转角限幅\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 倾转角限幅: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 3: 悬停模式推力方向
    try
        dyn = EvtolDynamics(evtol, aero);
        dyn.tilt_angles = zeros(4, 1);  % 悬停
        dyn.omega = ones(4, 1) * evtol.omega_hover;

        omega_cmd = ones(4, 1) * evtol.omega_hover;
        [F, M] = dyn.compute_rotor_forces(omega_cmd);

        % 推力应主要沿 Z 轴 (向下)
        assert(F(3) < 0, '悬停推力应向下');
        assert(abs(F(1)) < abs(F(3)) * 0.01, '悬停时 X 推力应接近 0');
        fprintf('[PASS] 悬停模式推力方向\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 悬停模式推力方向: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 4: 巡航模式推力方向
    try
        dyn = EvtolDynamics(evtol, aero);
        dyn.tilt_angles = ones(4, 1) * pi/2;  % 巡航
        dyn.omega = ones(4, 1) * evtol.omega_hover;

        omega_cmd = ones(4, 1) * evtol.omega_hover;
        [F, M] = dyn.compute_rotor_forces(omega_cmd);

        % 推力应主要沿 X 轴 (向前)
        assert(F(1) > 0, '巡航推力应向前');
        assert(abs(F(3)) < abs(F(1)) * 0.01, '巡航时 Z 推力应接近 0');
        fprintf('[PASS] 巡航模式推力方向\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 巡航模式推力方向: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 5: 过渡模式推力方向 (45°)
    try
        dyn = EvtolDynamics(evtol, aero);
        dyn.tilt_angles = ones(4, 1) * pi/4;  % 45°
        dyn.omega = ones(4, 1) * evtol.omega_hover;

        omega_cmd = ones(4, 1) * evtol.omega_hover;
        [F, M] = dyn.compute_rotor_forces(omega_cmd);

        % 推力应在 X 和 Z 之间
        assert(F(1) > 0, '过渡模式应有向前推力');
        assert(F(3) < 0, '过渡模式应有向下推力');
        fprintf('[PASS] 过渡模式推力方向\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 过渡模式推力方向: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 6: 能耗累积
    try
        dyn = EvtolDynamics(evtol, aero);
        dyn.reset([0;0;-50], [0;0;0], [0;0;0], [0;0;0]);
        assert(dyn.energy_used == 0, '初始能耗应为 0');

        % 模拟一步
        omega_cmd = ones(4, 1) * evtol.omega_hover;
        dyn.dynamics(0, dyn.state, omega_cmd, []);
        assert(dyn.energy_used > 0, '运行后能耗应增加');
        fprintf('[PASS] 能耗累积\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 能耗累积: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 7: 倾转速率限制
    try
        dyn = EvtolDynamics(evtol, aero);
        dyn.tilt_angles = zeros(4, 1);
        dyn.tilt_cmd = ones(4, 1) * pi/2;

        % 多步更新
        for i = 1:100
            dyn.update_tilt(i * 0.001);
        end

        % 不应瞬间到达
        assert(all(dyn.tilt_angles < pi/2), '倾转角不应瞬间到达');
        fprintf('[PASS] 倾转速率限制\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 倾转速率限制: %s\n', e.message);
        failed = failed + 1;
    end

    fprintf('\n=== 结果: %d 通过, %d 失败 ===\n', passed, failed);
end
