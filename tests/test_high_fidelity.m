function test_high_fidelity()
%TEST_HIGH_FIDELITY  高保真物理模型测试
%   测试: 四元数、地面效应、旋翼阻力、电池、VRS、失速模型、Dryden 湍流
%
%   运行: init_project; test_high_fidelity

    init_project;
    fprintf('=== 高保真物理模型测试 ===\n\n');

    passed = 0;
    failed = 0;

    %% ---- 四元数运算测试 ----

    % 测试 1: 四元数归一化
    try
        q = [2; 0; 0; 0];
        qn = quaternion_ops('normalize', q);
        assert(abs(norm(qn) - 1) < 1e-10, '归一化后模应为 1');
        fprintf('[PASS] 四元数归一化\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数归一化: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 2: 四元数 <-> 欧拉角往返
    try
        phi = 0.3; theta = 0.2; psi = 0.5;
        q = quaternion_ops('from_euler', phi, theta, psi);
        [phi2, theta2, psi2] = quaternion_ops('to_euler', q);
        assert(abs(phi - phi2) < 1e-8, 'phi 往返误差');
        assert(abs(theta - theta2) < 1e-8, 'theta 往返误差');
        assert(abs(psi - psi2) < 1e-8, 'psi 往返误差');
        fprintf('[PASS] 四元数 <-> 欧拉角往返\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数 <-> 欧拉角往返: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 3: 四元数旋转矩阵正交性
    try
        q = quaternion_ops('from_euler', 0.5, -0.3, 1.2);
        R = quaternion_ops('to_rotation', q);
        assert(norm(R' * R - eye(3)) < 1e-10, 'R 应正交');
        assert(abs(det(R) - 1) < 1e-10, 'det(R) 应为 1');
        fprintf('[PASS] 四元数旋转矩阵正交性\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数旋转矩阵正交性: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 4: 四元数恒等旋转
    try
        q = [1; 0; 0; 0];
        R = quaternion_ops('to_rotation', q);
        assert(norm(R - eye(3)) < 1e-10, '单位四元数应为单位矩阵');
        fprintf('[PASS] 四元数恒等旋转\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数恒等旋转: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 5: 四元数乘法 (组合旋转)
    try
        q1 = quaternion_ops('from_euler', 0, 0, pi/4);
        q2 = quaternion_ops('from_euler', 0, 0, pi/4);
        q3 = quaternion_ops('multiply', q1, q2);
        [phi, theta, psi] = quaternion_ops('to_euler', q3);
        assert(abs(psi - pi/2) < 1e-8, '两次 45° 旋转应得 90°');
        fprintf('[PASS] 四元数乘法组合旋转\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数乘法组合旋转: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 6: 四元数逆
    try
        q = quaternion_ops('from_euler', 0.3, -0.2, 0.8);
        q_inv = quaternion_ops('inverse', q);
        q_identity = quaternion_ops('multiply', q, q_inv);
        assert(abs(q_identity(1) - 1) < 1e-8, 'q * q^-1 应为单位四元数');
        assert(norm(q_identity(2:4)) < 1e-8, 'q * q^-1 虚部应为 0');
        fprintf('[PASS] 四元数逆\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数逆: %s\n', e.message);
        failed = failed + 1;
    end

    %% ---- 四元数动力学测试 ----

    % 测试 7: 四元数模式悬停
    try
        dyn = QuadcopterDynamics(quad);
        dyn.use_quaternion = true;
        dyn.reset([0;0;5], [0;0;0], [0;0;0], [0;0;0]);
        hover_thrust = quad.mass * quad.g;
        state_dot = dyn.dynamics_quat(0, dyn.state, [0;0;hover_thrust], [0;0;0]);
        assert(abs(state_dot(6)) < 0.01, '四元数模式悬停加速度应为 0');
        fprintf('[PASS] 四元数模式悬停\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数模式悬停: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 8: 四元数模式重力下落
    try
        dyn = QuadcopterDynamics(quad);
        dyn.use_quaternion = true;
        dyn.reset([0;0;5], [0;0;0], [0;0;0], [0;0;0]);
        state_dot = dyn.dynamics_quat(0, dyn.state, [0;0;0], [0;0;0]);
        assert(abs(state_dot(6) - (-quad.g)) < 0.01, '应受重力 -g');
        fprintf('[PASS] 四元数模式重力下落\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数模式重力下落: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 9: 四元数模式大角度 (无万向锁)
    try
        dyn = QuadcopterDynamics(quad);
        dyn.use_quaternion = true;
        % 接近 90° 俯仰 (Euler 模式下 tan(theta) -> inf)
        dyn.reset([0;0;5], [0;0;0], [0;pi/2*0.99;0], [0;0;0]);
        hover_thrust = quad.mass * quad.g;
        state_dot = dyn.dynamics_quat(0, dyn.state, [0;0;hover_thrust], [0;0;0]);
        % 检查导数是否有限
        assert(all(isfinite(state_dot)), '大角度时导数应有限');
        fprintf('[PASS] 四元数模式大角度 (无万向锁)\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 四元数模式大角度: %s\n', e.message);
        failed = failed + 1;
    end

    %% ---- 地面效应测试 ----

    % 测试 10: 远离地面无地面效应
    try
        dyn = QuadcopterDynamics(quad);
        f_ge = dyn.ground_effect_factor(10);  % 10m 高度
        assert(abs(f_ge - 1.0) < 0.01, '高处应无地面效应');
        fprintf('[PASS] 远离地面无地面效应\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 远离地面无地面效应: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 11: 近地面有地面效应
    try
        dyn = QuadcopterDynamics(quad);
        f_ge = dyn.ground_effect_factor(0.15);  % 15cm 高度
        assert(f_ge > 1.0, '近地面应有推力增益');
        assert(f_ge <= 1.5, '增益应被钳位');
        fprintf('[PASS] 近地面有地面效应\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 近地面有地面效应: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 12: 地面效应随高度递减
    try
        dyn = QuadcopterDynamics(quad);
        f1 = dyn.ground_effect_factor(0.1);
        f2 = dyn.ground_effect_factor(0.3);
        f3 = dyn.ground_effect_factor(1.0);
        assert(f1 > f2, '越近地面效应越强');
        assert(f2 > f3, '地面效应随高度递减');
        fprintf('[PASS] 地面效应随高度递减\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 地面效应随高度递减: %s\n', e.message);
        failed = failed + 1;
    end

    %% ---- 电池电压测试 ----

    % 测试 13: 标称电压推力系数
    try
        dyn = QuadcopterDynamics(quad);
        ct = dyn.battery_thrust_coeff();
        assert(abs(ct - motor.k_t) < 1e-10, '标称电压时系数应不变');
        fprintf('[PASS] 标称电压推力系数\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 标称电压推力系数: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 14: 低电压推力系数降低
    try
        dyn = QuadcopterDynamics(quad);
        dyn.battery_voltage = 0.8 * dyn.V_nominal;  % 80% 电压
        ct = dyn.battery_thrust_coeff();
        ct_ratio = ct / motor.k_t;
        assert(ct_ratio < 1.0, '低电压时推力系数应降低');
        assert(abs(ct_ratio - 0.64) < 0.01, '应为 (0.8)^2 = 0.64');
        fprintf('[PASS] 低电压推力系数降低\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 低电压推力系数降低: %s\n', e.message);
        failed = failed + 1;
    end

    %% ---- VRS 检测测试 ----

    % 测试 15: 平飞无 VRS
    try
        dyn = QuadcopterDynamics(quad);
        [vrs, loss] = dyn.detect_vortex_ring(0, 500);  % 无下降
        assert(~vrs, '平飞不应触发 VRS');
        assert(loss == 1.0, '无损失');
        fprintf('[PASS] 平飞无 VRS\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 平飞无 VRS: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 16: 快速下降触发 VRS
    try
        dyn = QuadcopterDynamics(quad);
        [vrs, loss] = dyn.detect_vortex_ring(3.0, 500);  % 快速下降
        assert(vrs, '快速下降应触发 VRS');
        assert(loss < 1.0, '应有推力损失');
        assert(loss >= 0.5, '损失不应超过 50%');
        fprintf('[PASS] 快速下降触发 VRS\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 快速下降触发 VRS: %s\n', e.message);
        failed = failed + 1;
    end

    %% ---- 前飞升力修正测试 ----

    % 测试 17: 悬停无前飞升力修正
    try
        dyn = QuadcopterDynamics(quad);
        f_tl = dyn.translational_lift(0);
        assert(abs(f_tl - 1.0) < 0.01, '悬停时修正应为 1');
        fprintf('[PASS] 悬停无前飞升力修正\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 悬停无前飞升力修正: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 18: 前飞有升力修正
    try
        dyn = QuadcopterDynamics(quad);
        f_tl = dyn.translational_lift(20);  % 20 m/s 前飞
        assert(f_tl > 1.0, '前飞应有升力增益');
        assert(f_tl <= 1.3, '增益应被钳位');
        fprintf('[PASS] 前飞有升力修正\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 前飞有升力修正: %s\n', e.message);
        failed = failed + 1;
    end

    %% ---- C1 连续失速模型测试 ----

    % 测试 19: 失速前 CL 线性
    try
        aero_m = AeroModel(aero, fw);
        alpha = 5 * pi/180;  % 远低于失速
        CL = aero_m.CL_alpha(alpha);
        CL_expected = aero.CL0 + aero.CL_alpha * alpha;
        assert(abs(CL - CL_expected) < 0.01, '线性区应符合 CL0 + CL_alpha*alpha');
        fprintf('[PASS] 失速前 CL 线性\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 失速前 CL 线性: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 20: 失速点连续性
    try
        aero_m = AeroModel(aero, fw);
        % 在失速角两侧取值, 应该连续
        delta = 0.001;
        CL_below = aero_m.CL_alpha(aero.alpha_stall - delta);
        CL_above = aero_m.CL_alpha(aero.alpha_stall + delta);
        assert(abs(CL_below - CL_above) < 0.1, '失速点附近应连续');
        fprintf('[PASS] 失速点连续性\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 失速点连续性: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 21: 失速后 CL 下降
    try
        aero_m = AeroModel(aero, fw);
        CL_stall = aero_m.CL_alpha(aero.alpha_stall);
        CL_post = aero_m.CL_alpha(aero.alpha_stall + 10*pi/180);
        assert(CL_post < CL_stall, '失速后 CL 应下降');
        fprintf('[PASS] 失速后 CL 下降\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 失速后 CL 下降: %s\n', e.message);
        failed = failed + 1;
    end

    %% ---- Dryden 湍流测试 ----

    % 测试 22: 无湍流模式
    try
        wm = WindModel();
        wm.set_constant([5; 0; 0]);
        wm.set_turbulence_type('none');
        wind = wm.update(0.01);
        assert(norm(wind - [5;0;0]) < 1e-10, '无湍流时应等于常值风');
        fprintf('[PASS] 无湍流模式\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 无湍流模式: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 23: Dryden 湍流非零
    try
        wm = WindModel();
        wm.set_constant([0; 0; 0]);
        wm.set_turbulence_type('dryden');
        wm.V_airspeed = 15;
        wm.altitude = 100;
        % 多步运行使滤波器收敛
        for i = 1:100
            wind = wm.update(0.01);
        end
        % 湍流应产生非零扰动
        assert(norm(wind) > 0, 'Dryden 湍流应产生扰动');
        fprintf('[PASS] Dryden 湍流非零\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] Dryden 湍流非零: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 24: Dryden 湍流有界
    try
        wm = WindModel();
        wm.set_constant([0; 0; 0]);
        wm.set_turbulence_type('dryden');
        wm.V_airspeed = 15;
        wm.altitude = 100;
        max_wind = 0;
        for i = 1:1000
            wind = wm.update(0.01);
            max_wind = max(max_wind, norm(wind));
        end
        assert(max_wind < 20, 'Dryden 湍流应有界 (<20 m/s)');
        fprintf('[PASS] Dryden 湍流有界\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] Dryden 湍流有界: %s\n', e.message);
        failed = failed + 1;
    end

    %% ---- 综合测试 ----

    % 测试 25: get_state 兼容性 (Euler 模式)
    try
        dyn = QuadcopterDynamics(quad);
        dyn.reset([1;2;3], [0.1;0.2;0.3], [0.1;0.2;0.3], [0.01;0.02;0.03]);
        [pos, vel, att, rates, R] = dyn.get_state();
        assert(norm(pos - [1;2;3]) < 1e-10, '位置应一致');
        assert(norm(att - [0.1;0.2;0.3]) < 1e-10, '姿态应一致');
        fprintf('[PASS] get_state 兼容性 (Euler)\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] get_state 兼容性 (Euler): %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 26: get_state 兼容性 (四元数模式)
    try
        dyn = QuadcopterDynamics(quad);
        dyn.use_quaternion = true;
        dyn.reset([1;2;3], [0.1;0.2;0.3], [0.1;0.2;0.3], [0.01;0.02;0.03]);
        [pos, vel, att, rates, R] = dyn.get_state();
        assert(norm(pos - [1;2;3]) < 1e-10, '位置应一致');
        assert(abs(att(1) - 0.1) < 1e-6, 'phi 应一致');
        assert(abs(att(2) - 0.2) < 1e-6, 'theta 应一致');
        assert(abs(att(3) - 0.3) < 1e-6, 'psi 应一致');
        fprintf('[PASS] get_state 兼容性 (四元数)\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] get_state 兼容性 (四元数): %s\n', e.message);
        failed = failed + 1;
    end

    fprintf('\n=== 结果: %d 通过, %d 失败 ===\n', passed, failed);
end
