% EVTOL_MISSION  eVTOL 完整任务仿真
%
%   演示: 起飞 → 巡航 → 减速 → 降落
%   模拟典型城市空中交通 (UAM) 任务
%
%   运行:
%     init_project
%     evtol_mission

    init_project;

    % 任务: 完整 VTOL 任务
    mission = struct();
    mission.type = 'vtol_mission';
    mission.speed = 25;       % 巡航速度 [m/s]

    % 运行仿真
    duration = 120;
    [t, state, mode_log, energy] = run_evtol_sim(mission, duration);

    % 绘图
    plot_evtol_results(t, state, mode_log, energy);

    % 打印性能指标
    fprintf('\n=== eVTOL 完整任务性能指标 ===\n');

    N = length(t);

    % 起飞段
    takeoff_idx = 1:round(10 / (t(2)-t(1)));
    alt_takeoff = -state(takeoff_idx(end), 3);
    fprintf('  起飞高度: %.1f m\n', alt_takeoff);

    % 巡航段
    cruise_start = round(10 / (t(2)-t(1)));
    cruise_end = round(60 / (t(2)-t(1)));
    V_cruise = sqrt(sum(state(cruise_start:cruise_end, 4:6).^2, 2));
    fprintf('  巡航速度: %.1f m/s\n', mean(V_cruise));

    % 航程
    total_dist = sum(sqrt(sum(diff(state(:, 1:3)).^2, 2)));
    fprintf('  总航程: %.1f m (%.2f km)\n', total_dist, total_dist/1000);

    % 能耗效率
    fprintf('  总能耗: %.2f Wh\n', energy(end));
    if total_dist > 0
        fprintf('  能效: %.2f Wh/km\n', energy(end) / (total_dist/1000));
    end

    % 降落段
    final_alt = -state(end, 3);
    fprintf('  最终高度: %.1f m\n', final_alt);
end
