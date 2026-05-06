function animate_3d(t, state, params, varargin)
%ANIMATE_3D  四旋翼 3D 动画
%
%   animate_3d(t, state, params)
%   animate_3d(t, state, params, 'speed', 0.5, 'trail', true)
%
%   输入:
%     t      - 时间向量
%     state  - 状态矩阵 [Nx12]
%     params - quad 参数结构体
%     可选参数:
%       'speed'  - 播放速度倍数 (默认 1)
%       'trail'  - 是否显示轨迹 (默认 true)
%       'save'   - 保存文件名 (如 'quad.avi')

    % 解析可选参数
    p = inputParser;
    addParameter(p, 'speed', 1);
    addParameter(p, 'trail', true);
    addParameter(p, 'save', '');
    parse(p, varargin{:});
    opts = p.Results;

    figure('Name', 'Quadcopter Animation', 'Position', [100 100 800 600]);

    arm = params.arm_length;

    % 坐标轴范围
    margin = 1;
    x_range = [min(state(:,1))-margin, max(state(:,1))+margin];
    y_range = [min(state(:,2))-margin, max(state(:,2))+margin];
    z_range = [min(state(:,3))-margin, max(state(:,3))+margin];

    % 确保最小范围
    if diff(x_range) < 1, x_range = [-1, 1]; end
    if diff(y_range) < 1, y_range = [-1, 1]; end
    if diff(z_range) < 1, z_range = [0, 2]; end

    % 保存视频
    if ~isempty(opts.save)
        v = VideoWriter(opts.save);
        v.FrameRate = 30;
        open(v);
    end

    % 绘制间隔
    skip = max(1, round(1 / (opts.speed * (t(2)-t(1)) * 30)));

    % 轨迹数据
    trail_x = []; trail_y = []; trail_z = [];

    for i = 1:skip:length(t)
        clf;

        pos = state(i, 1:3)';
        att = state(i, 7:9)';

        % 旋转矩阵
        R = euler_to_rotation(att(1), att(2), att(3));

        % 四个电机位置 (body frame)
        motor_body = [
             arm,  0, 0;   % 电机 1
             0,  arm, 0;   % 电机 2
            -arm,  0, 0;   % 电机 3
             0, -arm, 0;   % 电机 4
        ]';

        % 转换到世界坐标
        motor_world = R * motor_body + pos;

        % 绘制机臂
        hold on;
        plot3([motor_world(1,1), motor_world(1,3)], ...
              [motor_world(2,1), motor_world(2,3)], ...
              [motor_world(3,1), motor_world(3,3)], 'b-', 'LineWidth', 2);
        plot3([motor_world(1,2), motor_world(1,4)], ...
              [motor_world(2,2), motor_world(2,4)], ...
              [motor_world(3,2), motor_world(3,4)], 'b-', 'LineWidth', 2);

        % 绘制电机
        for m = 1:4
            plot3(motor_world(1,m), motor_world(2,m), motor_world(3,m), ...
                  'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        end

        % 绘制中心
        plot3(pos(1), pos(2), pos(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

        % 绘制轨迹
        if opts.trail
            trail_x = [trail_x, pos(1)];
            trail_y = [trail_y, pos(2)];
            trail_z = [trail_z, pos(3)];
            plot3(trail_x, trail_y, trail_z, 'g-', 'LineWidth', 1);
        end

        % 坐标轴设置
        xlim(x_range); ylim(y_range); zlim(z_range);
        xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
        title(sprintf('t = %.2f s', t(i)));
        grid on; axis equal; view(3);
        drawnow;

        % 保存帧
        if ~isempty(opts.save)
            frame = getframe(gcf);
            writeVideo(v, frame);
        end
    end

    if ~isempty(opts.save)
        close(v);
        fprintf('动画已保存: %s\n', opts.save);
    end
end
