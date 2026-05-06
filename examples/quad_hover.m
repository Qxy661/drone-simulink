% QUAD_HOVER  四旋翼悬停仿真示例
%
%   演示: 起飞 -> 悬停在 (0, 0, 5)
%
%   运行:
%     init_project
%     quad_hover

    init_project;

    % 创建轨迹: 悬停在 (0, 0, 5)
    traj = TrajectoryGenerator();
    traj.set_hover([0; 0; 5]);

    % 运行仿真
    [t, state, cmd] = run_quad_sim(traj, 20);

    % 绘图
    plot_results(t, state);

    % 打印性能指标
    fprintf('\n=== 悬停性能指标 ===\n');

    % 取后 50% 数据分析稳态
    N = length(t);
    steady_idx = round(N/2):N;

    % 稳态位置误差
    pos_err_x = mean(abs(state(steady_idx, 1)));
    pos_err_y = mean(abs(state(steady_idx, 2)));
    pos_err_z = mean(abs(state(steady_idx, 3) - 5));

    fprintf('  X 稳态误差: %.4f m\n', pos_err_x);
    fprintf('  Y 稳态误差: %.4f m\n', pos_err_y);
    fprintf('  Z 稳态误差: %.4f m\n', pos_err_z);

    % Z 振荡 (标准差)
    z_osc = std(state(steady_idx, 3));
    fprintf('  Z 振荡 (std): %.4f m\n', z_osc);

    % 最终高度
    fprintf('  最终高度: %.2f m\n', state(end, 3));

    if pos_err_z < 0.05 && z_osc < 0.02
        fprintf('  [PASS] 悬停精度达标\n');
    else
        fprintf('  [WARN] 悬停精度未达标，需调参\n');
    end
end
