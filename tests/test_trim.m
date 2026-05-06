function test_trim()
%TEST_trim  固定翼配平和气动模型测试
%
%   运行: init_project; test_trim

    init_project;
    fprintf('=== 固定翼配平测试 ===\n\n');

    passed = 0;
    failed = 0;

    % 测试 1: 气动模型迎角计算
    try
        aero_model = AeroModel(aero, fw);
        V_body = [25; 0; 0];  % 纯前飞
        [alpha, beta, V] = aero_model.wind_angles(V_body);
        assert(abs(alpha) < 0.01, '水平飞行迎角应接近 0');
        assert(abs(beta) < 0.01, '无侧滑');
        assert(abs(V - 25) < 0.01, '空速应为 25');
        fprintf('[PASS] 气流角计算\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 气流角计算: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 2: 配平计算
    try
        aero_model = AeroModel(aero, fw);
        [alpha_trim, de_trim, T_trim] = aero_model.trim(fw.V_cruise, 0);

        % 配平迎角应在合理范围
        assert(alpha_trim > 0 && alpha_trim < 10*pi/180, ...
            '配平迎角应在 0~10 度');
        % 配平推力应为正
        assert(T_trim > 0, '配平推力应为正');
        % 配平升降舵应在合理范围
        assert(abs(de_trim) < fw.elevator_max, '配平升降舵应在限幅内');
        fprintf('[PASS] 配平计算\n');
        fprintf('  alpha_trim = %.2f deg\n', rad2deg(alpha_trim));
        fprintf('  de_trim = %.2f deg\n', rad2deg(de_trim));
        fprintf('  T_trim = %.1f N\n', T_trim);
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 配平计算: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 3: 升力系数线性区
    try
        aero_model = AeroModel(aero, fw);
        CL_0 = aero_model.CL_alpha(0);
        CL_5 = aero_model.CL_alpha(5 * pi/180);

        assert(abs(CL_0 - aero.CL0) < 0.01, 'alpha=0 时 CL 应为 CL0');
        assert(CL_5 > CL_0, '迎角增大应增加升力');
        fprintf('[PASS] 升力系数线性区\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 升力系数线性区: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 4: 失速特性
    try
        aero_model = AeroModel(aero, fw);
        CL_stall = aero_model.CL_alpha(aero.alpha_stall);
        CL_post = aero_model.CL_alpha(aero.alpha_stall + 5*pi/180);

        assert(CL_post < CL_stall, '失速后升力应下降');
        fprintf('[PASS] 失速特性\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 失速特性: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 5: 阻力系数
    try
        aero_model = AeroModel(aero, fw);
        CD_0 = aero_model.CD_alpha(0, 0);
        CD_5 = aero_model.CD_alpha(0.5, 0);

        assert(CD_0 >= aero.CD0, '零升阻力应 >= CD0');
        assert(CD_5 > CD_0, '升力增大应增加诱导阻力');
        fprintf('[PASS] 阻力系数\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 阻力系数: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 6: 舵面限幅
    try
        dyn = FixedWingDynamics(fw, aero);
        dyn.set_surfaces(100, 100, 100);  % 超限
        assert(abs(dyn.surfaces.elevator) <= fw.elevator_max, '升降舵应限幅');
        assert(abs(dyn.surfaces.aileron) <= fw.aileron_max, '副翼应限幅');
        assert(abs(dyn.surfaces.rudder) <= fw.rudder_max, '方向舵应限幅');
        fprintf('[PASS] 舵面限幅\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 舵面限幅: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 7: 固定翼动力学基本运行
    try
        dyn = FixedWingDynamics(fw, aero);
        aero_model = AeroModel(aero, fw);
        [alpha_trim, ~, T_trim] = aero_model.trim(fw.V_cruise, 0);

        % 设置配平状态
        state = zeros(12, 1);
        state(4) = fw.V_cruise * cos(alpha_trim);
        state(6) = fw.V_cruise * sin(alpha_trim);
        state(8) = alpha_trim;

        F_thrust = [T_trim; 0; 0];
        state_dot = dyn.dynamics(0, state, F_thrust, []);

        % 配平状态导数应接近零
        assert(norm(state_dot(4:6)) < 1.0, '配平平动加速度应接近零');
        assert(norm(state_dot(10:12)) < 0.5, '配平角加速度应接近零');
        fprintf('[PASS] 固定翼动力学基本运行\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 固定翼动力学基本运行: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 8: 气流坐标系到机体系变换
    try
        % 迎角=0, 侧滑=0: 升力沿 Z 轴, 阻力沿 X 轴
        aero_model = AeroModel(aero, fw);
        F_wind = [100; 0; 50];  % [升力; 阻力; 侧力]
        F_body = aero_model.wind_to_body(F_wind, 0, 0);

        % 此时: 升力沿 Z 轴, 阻力沿 X 轴 (简化)
        assert(abs(F_body(3) - 100) < 1, '迎角=0 时升力应沿 Z 轴');
        fprintf('[PASS] 气流坐标系变换\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 气流坐标系变换: %s\n', e.message);
        failed = failed + 1;
    end

    fprintf('\n=== 结果: %d 通过, %d 失败 ===\n', passed, failed);
end
