% COMPOUND_VTOL  复合翼垂直起降任务仿真
%
%   演示: 起飞 → 巡航 → 减速 → 降落
%   展示升力旋翼+推进器+气动的协同工作
%
%   运行:
%     init_project
%     compound_vtol

    init_project;

    mission = struct();
    mission.type = 'vtol';

    duration = 120;
    [t, state, mode_log, energy] = run_compound_sim(mission, duration);

    % 绘图
    figure('Name', 'Compound Wing VTOL', 'Position', [100 100 1200 800]);

    subplot(2,2,1);
    plot(t, -state(:,3), 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('高度 [m]'); title('高度'); grid on;

    subplot(2,2,2);
    V = sqrt(sum(state(:,4:6).^2, 2));
    plot(t, V, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('空速 [m/s]'); title('空速'); grid on;

    subplot(2,2,3);
    plot(t, energy, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('能耗 [Wh]'); title('累计能耗'); grid on;

    subplot(2,2,4);
    plot3(state(:,1), state(:,2), -state(:,3), 'b-', 'LineWidth', 2);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('高度 [m]');
    title('3D 轨迹'); grid on; axis equal; view(3);

    fprintf('\n=== 复合翼任务指标 ===\n');
    fprintf('  总航程: %.1f m\n', sum(sqrt(sum(diff(state(:,1:3)).^2, 2))));
    fprintf('  总能耗: %.2f Wh\n', energy(end));
    fprintf('  最终高度: %.1f m\n', -state(end,3));
end
