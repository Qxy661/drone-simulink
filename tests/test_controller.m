function test_controller()
%TEST_CONTROLLER  控制器模块单元测试
%
%   运行: init_project; test_controller

    init_project;
    fprintf('=== 控制器模块测试 ===\n\n');

    passed = 0;
    failed = 0;

    % 测试 1: PID 基本响应
    try
        pid = PIDController(1.0, 0, 0, 0.01);
        out = pid.update(1.0);
        assert(abs(out - 1.0) < 0.01, 'P=1 应输出 = 误差');
        fprintf('[PASS] PID 比例响应\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] PID 比例响应: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 2: PID 积分累积
    try
        pid = PIDController(0, 1.0, 0, 0.1);
        out = 0;
        for i = 1:10
            out = pid.update(1.0);
        end
        expected = 10 * 0.1 * 1.0;  % 10 步 * dt * error
        assert(abs(out - expected) < 0.01, '积分应累积');
        fprintf('[PASS] PID 积分累积\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] PID 积分累积: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 3: PID 抗饱和
    try
        pid = PIDController(0, 1.0, 0, 0.1);
        pid.set_limits(-5, 5, 10);
        for i = 1:100
            out = pid.update(1.0);
        end
        assert(abs(out) <= 5.01, '输出应被限幅');
        assert(abs(pid.integral) <= 10.01, '积分应被限幅');
        fprintf('[PASS] PID 抗饱和\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] PID 抗饱和: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 4: PID 重置
    try
        pid = PIDController(1.0, 1.0, 1.0, 0.1);
        for i = 1:10
            pid.update(1.0);
        end
        pid.reset();
        assert(pid.integral == 0, '重置后积分应为 0');
        assert(pid.prev_error == 0, '重置后历史应为 0');
        fprintf('[PASS] PID 重置\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] PID 重置: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 5: 姿态控制器输出
    try
        att_ctrl = AttitudeController(ctrl, 0.02);
        att_des = [0.1; 0; 0];  % 期望 0.1 rad 滚转
        att_cur = [0; 0; 0];
        rates_cur = [0; 0; 0];
        torques = att_ctrl.update(att_des, att_cur, rates_cur);
        assert(length(torques) == 3, '输出应为 3 维');
        assert(torques(1) > 0, '正滚转误差应产生正力矩');
        fprintf('[PASS] 姿态控制器输出\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 姿态控制器输出: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 6: 位置控制器输出
    try
        pos_ctrl = PositionController(ctrl, quad, motor, 0.01);
        pos_des = [0; 0; 5];
        pos_cur = [0; 0; 0];
        vel_cur = [0; 0; 0];
        [att_des, thrust] = pos_ctrl.update(pos_des, pos_cur, vel_cur);
        assert(length(att_des) == 3, '期望姿态应为 3 维');
        assert(thrust > 0, '推力应为正');
        assert(thrust > quad.mass * quad.g, '向上加速时推力应大于悬停');
        fprintf('[PASS] 位置控制器输出\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 位置控制器输出: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 7: 位置控制器倾斜限幅
    try
        pos_ctrl = PositionController(ctrl, quad, motor, 0.01);
        pos_des = [100; 0; 5];  % 远距离目标
        pos_cur = [0; 0; 5];
        vel_cur = [0; 0; 0];
        [att_des, ~] = pos_ctrl.update(pos_des, pos_cur, vel_cur);
        assert(abs(att_des(1)) <= ctrl.pos.max_tilt + 0.01, '倾斜角应被限幅');
        fprintf('[PASS] 位置控制器倾斜限幅\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 位置控制器倾斜限幅: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 8: 级联控制一致性 (悬停状态)
    try
        pos_ctrl = PositionController(ctrl, quad, motor, 0.01);
        att_ctrl = AttitudeController(ctrl, 0.02);

        % 悬停在目标位置
        [att_des, thrust] = pos_ctrl.update([0;0;5], [0;0;5], [0;0;0]);
        torques = att_ctrl.update(att_des, [0;0;0], [0;0;0]);

        % 期望姿态应接近零
        assert(norm(att_des) < 0.01, '悬停时期望姿态应接近零');
        % 推力应接近悬停
        assert(abs(thrust - quad.mass * quad.g) < 1, '悬停推力应接近 mg');
        fprintf('[PASS] 级联控制一致性\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 级联控制一致性: %s\n', e.message);
        failed = failed + 1;
    end

    fprintf('\n=== 结果: %d 通过, %d 失败 ===\n', passed, failed);
end
