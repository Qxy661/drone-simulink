% EVTOL_TRANSITION  eVTOL 悬停↔巡航过渡仿真
%
%   演示: 悬停 → 加速过渡 → 巡航 → 减速过渡 → 悬停
%
%   运行:
%     init_project
%     evtol_transition

    init_project;

    % 任务: 过渡测试
    mission = struct();
    mission.type = 'transition';

    % 运行仿真
    duration = 60;
    [t, state, mode_log, energy] = run_evtol_sim(mission, duration);

    % 绘图
    plot_evtol_results(t, state, mode_log, energy);

    % 打印性能指标
    fprintf('\n=== eVTOL 过渡性能指标 ===\n');

    N = length(t);

    % 悬停段 (前 10s)
    hover_idx = 1:round(10 / (t(2)-t(1)));
    alt_hover = -state(hover_idx, 3);
    fprintf('  悬停高度误差: %.2f m\n', mean(abs(alt_hover - 50)));

    % 巡航段 (后 20s)
    cruise_idx = round(40 / (t(2)-t(1)):N);
    V_cruise = sqrt(sum(state(cruise_idx, 4:6).^2, 2));
    fprintf('  巡航速度: %.1f m/s\n', mean(V_cruise));

    % 总能耗
    fprintf('  总能耗: %.2f Wh\n', energy(end));

    % 模式统计
    hover_count = sum(strcmp(mode_log, 'hover'));
    trans_count = sum(strcmp(mode_log, 'transition'));
    cruise_count = sum(strcmp(mode_log, 'cruise'));
    fprintf('  模式分布: 悬停=%d, 过渡=%d, 巡航=%d\n', ...
            hover_count, trans_count, cruise_count);
end
