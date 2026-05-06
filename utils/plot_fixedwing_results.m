function plot_fixedwing_results(t, state, cmd)
%PLOT_FIXEDWING_RESULTS  绘制固定翼仿真结果
%
%   plot_fixedwing_results(t, state, cmd)
%
%   输入:
%     t     - 时间向量 [Nx1]
%     state - 状态矩阵 [Nx12]
%     cmd   - 命令矩阵 [Nx4] [elevator, aileron, rudder, throttle]

    figure('Name', 'Fixed-Wing Simulation Results', 'Position', [100 100 1400 900]);

    % 高度
    subplot(3, 3, 1);
    alt = -state(:, 3);  % z 向下为正, 高度向上为正
    plot(t, alt, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('高度 [m]');
    title('高度'); grid on;

    % 空速
    subplot(3, 3, 2);
    V = sqrt(sum(state(:, 4:6).^2, 2));
    plot(t, V, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('空速 [m/s]');
    title('空速'); grid on;

    % 航向
    subplot(3, 3, 3);
    heading_deg = rad2deg(state(:, 9));
    plot(t, heading_deg, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('航向 [deg]');
    title('航向'); grid on;

    % 姿态
    subplot(3, 3, 4);
    att_deg = rad2deg(state(:, 7:9));
    plot(t, att_deg(:,1), 'b-', t, att_deg(:,2), 'r-', t, att_deg(:,3), 'g-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('角度 [deg]');
    title('姿态'); legend('\phi', '\theta', '\psi'); grid on;

    % 角速度
    subplot(3, 3, 5);
    plot(t, state(:,10), 'b-', t, state(:,11), 'r-', t, state(:,12), 'g-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('角速度 [rad/s]');
    title('角速度'); legend('p', 'q', 'r'); grid on;

    % 舵面
    subplot(3, 3, 6);
    cmd_deg = rad2deg(cmd(:, 1:3));
    plot(t, cmd_deg(:,1), 'b-', t, cmd_deg(:,2), 'r-', t, cmd_deg(:,3), 'g-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('舵面偏转 [deg]');
    title('舵面命令'); legend('升降舵', '副翼', '方向舵'); grid on;

    % 油门
    subplot(3, 3, 7);
    plot(t, cmd(:,4), 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('油门 (0~1)');
    title('油门'); grid on; ylim([0 1]);

    % 3D 轨迹
    subplot(3, 3, [8, 9]);
    pos = state(:, 1:3);
    plot3(pos(:,1), pos(:,2), -pos(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(pos(1,1), pos(1,2), -pos(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot3(pos(end,1), pos(end,2), -pos(end,3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('高度 [m]');
    title('3D 轨迹'); legend('飞行轨迹', '起点', '终点'); grid on;
    axis equal; view(3);
end
