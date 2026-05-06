classdef SwarmManager < handle
%SWARMmanager  集群管理器
%   多机通信、编队、避碰、任务分配
%
%   编队模式:
%     - V 字形 (V-shape)
%     - 线形 (Line)
%     - 圆形 (Circle)
%     - 自定义 (Custom)
%
%   避碰策略:
%     - 最小安全距离
%     - 速度排斥力

    properties
        drones          % 无人机列表 (cell array of ModularDynamics)
        n_drones        % 无人机数量
        formation       % 编队模式
        formation_sep   % 编队间距 [m]
        safe_dist       % 安全距离 [m]
        comm_range      % 通信范围 [m]
        targets         % 各机目标位置 [nx3]
    end

    methods
        function obj = SwarmManager(n_drones, config, formation_sep)
        %SWARMmanager  构造函数
            obj.n_drones = n_drones;
            obj.formation_sep = formation_sep;
            obj.safe_dist = formation_sep * 0.5;
            obj.comm_range = 100;  % 通信范围 [m]

            % 创建无人机
            obj.drones = cell(n_drones, 1);
            for i = 1:n_drones
                obj.drones{i} = ModularDynamics(config);
            end

            obj.formation = 'v_shape';
            obj.targets = zeros(n_drones, 3);
        end

        function set_formation(obj, mode, center)
        %SET_formation  设置编队
            obj.formation = mode;
            obj.targets = obj.compute_formation_positions(center);
        end

        function positions = compute_formation_positions(obj, center)
        %COMPUTE_formation_POSITIONS  计算编队位置
            n = obj.n_drones;
            sep = obj.formation_sep;
            positions = zeros(n, 3);

            switch obj.formation
                case 'line'
                    % 线形编队
                    for i = 1:n
                        positions(i, :) = center + [(i-1)*sep, 0, 0];
                    end

                case 'v_shape'
                    % V 字形编队
                    for i = 1:n
                        side = 2*mod(i-1,2) - 1;  % 交替左右
                        row = ceil(i/2);
                        positions(i, :) = center + [-row*sep*0.5, side*row*sep*0.5, 0];
                    end

                case 'circle'
                    % 圆形编队
                    radius = sep * n / (2*pi);
                    for i = 1:n
                        angle = 2*pi*(i-1)/n;
                        positions(i, :) = center + [radius*cos(angle), radius*sin(angle), 0];
                    end

                case 'grid'
                    % 网格编队
                    cols = ceil(sqrt(n));
                    for i = 1:n
                        row = floor((i-1)/cols);
                        col = mod(i-1, cols);
                        positions(i, :) = center + [row*sep, col*sep, 0];
                    end
            end
        end

        function [avoidance_forces] = compute_collision_avoidance(obj)
        %COMPUTE_collision_avoidance  计算避碰排斥力
            avoidance_forces = zeros(obj.n_drones, 3);

            for i = 1:obj.n_drones
                pos_i = obj.drones{i}.state(1:3)';
                for j = 1:obj.n_drones
                    if i == j, continue; end
                    pos_j = obj.drones{j}.state(1:3)';
                    dist = norm(pos_i - pos_j);

                    if dist < obj.safe_dist && dist > 0.01
                        % 排斥力 (距离越近越强)
                        direction = (pos_i - pos_j) / dist;
                        magnitude = 2.0 * (1/dist - 1/obj.safe_dist);
                        avoidance_forces(i, :) = avoidance_forces(i, :) + magnitude * direction';
                    end
                end
            end
        end

        function omega_cmds = update(obj, dt)
        %UPDATE  集群控制更新
            omega_cmds = cell(obj.n_drones, 1);

            % 计算避碰力
            avoidance = compute_collision_avoidance(obj);

            for i = 1:obj.n_drones
                drone = obj.drones{i};
                pos = drone.state(1:3);
                vel = drone.state(4:6);

                % 目标位置 (含避碰修正)
                target = obj.targets(i, :)' + avoidance(i, :)';

                % 简单 P 控制
                err = target - pos;
                Kp = 1.5;
                Kd = 0.8;
                cmd_vel = Kp * err - Kd * vel;

                % 转换为推力命令
                thrust = drone.config.mass * (cmd_vel + [0; 0; drone.config.g]);
                thrust = max(0, thrust(3));

                % 简化: 均分到各旋翼
                n = drone.config.n_rotors;
                omega_sq = thrust / (n * drone.config.k_t);
                omega_cmds{i} = ones(n, 1) * sqrt(max(0, omega_sq));
            end
        end

        function states = get_all_states(obj)
        %GET_all_states  获取所有无人机状态
            states = zeros(obj.n_drones, 12);
            for i = 1:obj.n_drones
                states(i, :) = obj.drones{i}.state';
            end
        end

        function step(obj, dt)
        %STEP  仿真一步
            omega_cmds = obj.update(dt);
            for i = 1:obj.n_drones
                k1 = obj.drones{i}.dynamics(0, obj.drones{i}.state, omega_cmds{i});
                k2 = obj.drones{i}.dynamics(0, obj.drones{i}.state + dt/2*k1, omega_cmds{i});
                k3 = obj.drones{i}.dynamics(0, obj.drones{i}.state + dt/2*k2, omega_cmds{i});
                k4 = obj.drones{i}.dynamics(0, obj.drones{i}.state + dt*k3, omega_cmds{i});
                obj.drones{i}.state = obj.drones{i}.state + dt/6*(k1 + 2*k2 + 2*k3 + k4);

                % 地面约束
                if obj.drones{i}.state(3) > 0
                    obj.drones{i}.state(3) = 0;
                    obj.drones{i}.state(6) = min(0, obj.drones{i}.state(6));
                end
            end
        end
    end
end
