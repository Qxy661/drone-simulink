function test_compound()
%TEST_compound  复合翼模块测试
%
%   运行: init_project; test_compound

    init_project;
    fprintf('=== 复合翼测试 ===\n\n');

    passed = 0;
    failed = 0;

    % 测试 1: 悬停转速计算
    try
        omega_hover = sqrt(cw.mass * cw.g / (cw.n_lift * cw.lift_kt));
        F_hover = cw.n_lift * cw.lift_kt * omega_hover^2;
        assert(abs(F_hover - cw.mass * cw.g) < 0.1, '悬停推力应等于重力');
        fprintf('[PASS] 悬停转速计算\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 悬停转速计算: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 2: 升力旋翼推力方向
    try
        dyn = CompoundDynamics(cw, aero);
        dyn.lift_omega = ones(4, 1) * cw.omega_hover;
        [F, M] = dyn.compute_lift_forces();
        assert(F(3) < 0, '升力旋翼推力应向下 (Z负)');
        assert(abs(F(1)) < 1e-3, 'X方向推力应为0');
        fprintf('[PASS] 升力旋翼推力方向\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 升力旋翼推力方向: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 3: 推进器推力方向
    try
        dyn = CompoundDynamics(cw, aero);
        dyn.push_omega = 1000;
        F = dyn.compute_push_force();
        assert(F(1) > 0, '推进器推力应向前 (X正)');
        assert(abs(F(2)) < 1e-3, 'Y方向推力应为0');
        fprintf('[PASS] 推进器推力方向\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 推进器推力方向: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 4: 独立控制升力和推力
    try
        dyn = CompoundDynamics(cw, aero);
        dyn.lift_omega = ones(4, 1) * cw.omega_hover;
        dyn.push_omega = 500;

        [F_lift, ~] = dyn.compute_lift_forces();
        F_push = dyn.compute_push_force();

        % 升力沿Z, 推力沿X, 互不干扰
        assert(abs(F_lift(1)) < 1, '升力不应有X分量');
        assert(abs(F_push(3)) < 1, '推力不应有Z分量');
        fprintf('[PASS] 独立控制升力和推力\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 独立控制升力和推力: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 5: 气动升力
    try
        dyn = CompoundDynamics(cw, aero);
        V_body = [25; 0; 0];  % 前飞
        omega = [0; 0; 0];
        [F_aero, M_aero] = dyn.compute_aero(V_body, omega);
        % 应有向上的升力
        assert(F_aero(3) < 0, '气动升力应向上 (Z负)');
        fprintf('[PASS] 气动升力\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 气动升力: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 6: 能耗累积
    try
        dyn = CompoundDynamics(cw, aero);
        dyn.reset([0;0;-50], [0;0;0], [0;0;0], [0;0;0]);

        commands = struct('lift_cmd', ones(4,1)*cw.omega_hover, ...
                         'push_cmd', 500, 'elevator', 0, 'aileron', 0, 'rudder', 0);
        dyn.dynamics(0, dyn.state, commands, []);
        assert(dyn.energy_used > 0, '能耗应大于0');
        fprintf('[PASS] 能耗累积\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 能耗累积: %s\n', e.message);
        failed = failed + 1;
    end

    fprintf('\n=== 结果: %d 通过, %d 失败 ===\n', passed, failed);
end
