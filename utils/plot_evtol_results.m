function plot_evtol_results(t, state, mode_log, energy)
%PLOT_EVTOL_RESULTS  绘制 eVTOL 仿真结果
%
%   plot_evtol_results(t, state, mode_log, energy)

    figure('Name', 'eVTOL Simulation Results', 'Position', [100 100 1400 900]);

    % 高度
    subplot(3, 3, 1);
    alt = -state(:, 3);
    plot(t, alt, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('高度 [m]');
    title('高度'); grid on;

    % 空速
    subplot(3, 3, 2);
    V = sqrt(sum(state(:, 4:6).^2, 2));
    plot(t, V, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('空速 [m/s]');
    title('空速'); grid on;

    % 姿态
    subplot(3, 3, 3);
    att_deg = rad2deg(state(:, 7:9));
    plot(t, att_deg(:,1), 'b-', t, att_deg(:,2), 'r-', t, att_deg(:,3), 'g-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('角度 [deg]');
    title('姿态'); legend('\phi', '\theta', '\psi'); grid on;

    % 角速度
    subplot(3, 3, 4);
    plot(t, state(:,10), 'b-', t, state(:,11), 'r-', t, state(:,12), 'g-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('角速度 [rad/s]');
    title('角速度'); legend('p', 'q', 'r'); grid on;

    % 飞行模式
    subplot(3, 3, 5);
    mode_num = zeros(length(mode_log), 1);
    for i = 1:length(mode_log)
        switch mode_log{i}
            case 'hover', mode_num(i) = 0;
            case 'transition', mode_num(i) = 1;
            case 'cruise', mode_num(i) = 2;
        end
    end
    area(t, mode_num, 'FaceColor', [0.8 0.9 1]);
    xlabel('时间 [s]'); ylabel('模式');
    title('飞行模式');
    yticks([0 1 2]); yticklabels({'悬停', '过渡', '巡航'});
    ylim([-0.5 2.5]); grid on;

    % 能耗
    subplot(3, 3, 6);
    plot(t, energy, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('能耗 [Wh]');
    title('累计能耗'); grid on;

    % 功率 (瞬时)
    subplot(3, 3, 7);
    dt = t(2) - t(1);
    power = diff(energy) / dt * 3600;  % W
    plot(t(2:end), power, 'b-', 'LineWidth', 1);
    xlabel('时间 [s]'); ylabel('功率 [W]');
    title('瞬时功率'); grid on;

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
