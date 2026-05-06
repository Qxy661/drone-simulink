function plot_results(t, state, pos_des, att_des)
%PLOT_RESULTS  绘制仿真结果
%
%   plot_results(t, state)
%   plot_results(t, state, pos_des)
%   plot_results(t, state, pos_des, att_des)
%
%   输入:
%     t       - 时间向量 [Nx1]
%     state   - 状态矩阵 [Nx12]
%     pos_des - 期望位置 [Nx3] (可选)
%     att_des - 期望姿态 [Nx3] (可选)

    figure('Name', 'Quadcopter Simulation Results', 'Position', [100 100 1200 800]);

    % 位置
    subplot(3, 2, 1);
    plot(t, state(:,1), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, state(:,2), 'r-', 'LineWidth', 1.5);
    plot(t, state(:,3), 'g-', 'LineWidth', 1.5);
    if nargin >= 3 && ~isempty(pos_des)
        plot(t, pos_des(:,1), 'b--', t, pos_des(:,2), 'r--', t, pos_des(:,3), 'g--');
    end
    xlabel('时间 [s]'); ylabel('位置 [m]');
    title('位置'); legend('x', 'y', 'z'); grid on;

    % 速度
    subplot(3, 2, 2);
    plot(t, state(:,4), 'b-', t, state(:,5), 'r-', t, state(:,6), 'g-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('速度 [m/s]');
    title('速度'); legend('vx', 'vy', 'vz'); grid on;

    % 姿态
    subplot(3, 2, 3);
    att_deg = rad2deg(state(:,7:9));
    plot(t, att_deg(:,1), 'b-', t, att_deg(:,2), 'r-', t, att_deg(:,3), 'g-', 'LineWidth', 1.5);
    if nargin >= 4 && ~isempty(att_des)
        att_des_deg = rad2deg(att_des);
        hold on;
        plot(t, att_des_deg(:,1), 'b--', t, att_des_deg(:,2), 'r--', t, att_des_deg(:,3), 'g--');
    end
    xlabel('时间 [s]'); ylabel('角度 [deg]');
    title('姿态'); legend('\phi', '\theta', '\psi'); grid on;

    % 角速度
    subplot(3, 2, 4);
    plot(t, state(:,10), 'b-', t, state(:,11), 'r-', t, state(:,12), 'g-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('角速度 [rad/s]');
    title('角速度'); legend('p', 'q', 'r'); grid on;

    % 3D 轨迹
    subplot(3, 2, [5, 6]);
    plot3(state(:,1), state(:,2), state(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(state(1,1), state(1,2), state(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot3(state(end,1), state(end,2), state(end,3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
    if nargin >= 3 && ~isempty(pos_des)
        plot3(pos_des(:,1), pos_des(:,2), pos_des(:,3), 'r--', 'LineWidth', 1);
    end
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title('3D 轨迹'); legend('实际', '起点', '终点'); grid on;
    axis equal; view(3);
end
