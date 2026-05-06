% QUAD_CIRCLE  四旋翼圆形轨迹仿真示例
%
%   演示: 起飞 -> 悬停 -> 圆形轨迹 -> 悬停
%   圆心: (0, 0, 5), 半径: 3m, 周期: 10s
%
%   运行:
%     init_project
%     quad_circle

    init_project;

    % 创建轨迹: 圆形
    traj = TrajectoryGenerator();
    traj.set_circle([0; 0; 5], 3, 10);

    % 运行仿真
    [t, state, cmd] = run_quad_sim(traj, 30);

    % 绘图
    plot_results(t, state);

    % 打印性能指标
    fprintf('\n=== 圆形轨迹性能指标 ===\n');

    % 取中间段分析 (跳过起飞过渡)
    N = length(t);
    start_idx = round(N * 0.2);
    end_idx = round(N * 0.9);
    idx = start_idx:end_idx;

    % 计算期望位置
    pos_des = zeros(length(idx), 3);
    for k = 1:length(idx)
        pos_des(k, :) = traj.generate(t(idx(k)))';
    end

    % 跟踪误差
    err = state(idx, 1:3) - pos_des;
    track_err = sqrt(sum(err.^2, 2));

    fprintf('  平均跟踪误差: %.4f m\n', mean(track_err));
    fprintf('  最大跟踪误差: %.4f m\n', max(track_err));

    if max(track_err) < 0.2
        fprintf('  [PASS] 轨迹跟踪精度达标\n');
    else
        fprintf('  [WARN] 轨迹跟踪精度未达标，需调参\n');
    end
end
