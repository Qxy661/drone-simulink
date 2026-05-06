function test_modular()
%TEST_modular  模块化/集群测试
%
%   运行: init_project; test_modular

    init_project;
    fprintf('=== 模块化/集群测试 ===\n\n');

    passed = 0;
    failed = 0;

    % 测试 1: 4 旋翼混控
    try
        config = struct('n_rotors', 4, 'k_t', 1.5e-5, 'k_d', 2e-7, ...
            'positions', [0.225 0.225 0; 0.225 -0.225 0; -0.225 -0.225 0; -0.225 0.225 0], ...
            'directions', [1 -1 -1 1], 'tau_m', 0.02, 'omega_max', 1200, ...
            'mass', 1.5, 'I', diag([0.0035 0.0035 0.0065]), 'g', 9.81);
        dyn = ModularDynamics(config);
        assert(size(dyn.mixer, 1) == 4 && size(dyn.mixer, 2) == 4, '混控矩阵应为 4x4');
        fprintf('[PASS] 4 旋翼混控矩阵\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 4 旋翼混控矩阵: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 2: 6 旋翼冗余
    try
        config.n_rotors = 6;
        angles = (0:5) * 60 * pi/180;
        config.positions = [0.3*cos(angles)', 0.3*sin(angles)', zeros(6,1)];
        config.directions = [1 -1 1 -1 1 -1];
        dyn = ModularDynamics(config);
        assert(size(dyn.mixer, 2) == 6, '应有 6 列');
        fprintf('[PASS] 6 旋翼冗余配置\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 6 旋翼冗余配置: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 3: 故障注入
    try
        config.n_rotors = 4;
        config.positions = [0.225 0.225 0; 0.225 -0.225 0; -0.225 -0.225 0; -0.225 0.225 0];
        config.directions = [1 -1 -1 1];
        dyn = ModularDynamics(config);
        dyn.inject_fault(1, 0);  % 电机 1 失效
        assert(dyn.fault.failed(1) == true, '电机 1 应标记为失效');
        assert(dyn.count_active() == 3, '应有 3 个活跃电机');
        fprintf('[PASS] 故障注入\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 故障注入: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 4: 自适应重分配
    try
        dyn = ModularDynamics(config);
        dyn.inject_fault(1, 0);
        commands = [15; 0; 0; 0];  % 悬停推力
        omega_cmd = dyn.adaptive_mix(commands);
        assert(length(omega_cmd) == 4, '输出应为 4 维');
        assert(omega_cmd(1) == 0, '失效电机转速应为 0');
        assert(all(omega_cmd(2:4) > 0), '活跃电机应有转速');
        fprintf('[PASS] 自适应重分配\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 自适应重分配: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 5: 编队位置计算
    try
        swarm = SwarmManager(5, config, 3.0);
        positions = swarm.compute_formation_positions([0;0;-10]);
        assert(size(positions, 1) == 5, '应有 5 个位置');
        fprintf('[PASS] 编队位置计算\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 编队位置计算: %s\n', e.message);
        failed = failed + 1;
    end

    % 测试 6: 避碰力
    try
        swarm = SwarmManager(3, config, 2.0);
        swarm.drones{1}.state(1:3) = [0;0;-5];
        swarm.drones{2}.state(1:3) = [0.5;0;-5];  % 很近
        swarm.drones{3}.state(1:3) = [10;0;-5];   % 很远
        forces = swarm.compute_collision_avoidance();
        assert(norm(forces(1,:)) > 0, '近距无人机应有排斥力');
        assert(norm(forces(3,:)) < 0.01, '远距无人机排斥力应接近 0');
        fprintf('[PASS] 避碰力计算\n');
        passed = passed + 1;
    catch e
        fprintf('[FAIL] 避碰力计算: %s\n', e.message);
        failed = failed + 1;
    end

    fprintf('\n=== 结果: %d 通过, %d 失败 ===\n', passed, failed);
end
