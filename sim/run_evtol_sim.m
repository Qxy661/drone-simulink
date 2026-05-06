function [t_log, state_log, mode_log, energy_log] = run_evtol_sim(mission, duration)
%RUN_EVTOL_SIM  eVTOL 仿真主函数
%
%   [t, state, mode, energy] = run_evtol_sim(mission)
%   [t, state, mode, energy] = run_evtol_sim(mission, 120)
%
%   输入:
%     mission  - 任务结构体:
%       .type       - 'transition' / 'waypoint' / 'vtol_mission'
%       .waypoints  - Nx3 航点 (可选)
%       .altitude   - 巡航高度 [m]
%       .speed      - 巡航速度 [m/s]
%     duration - 仿真时长 [s] (默认 120)
%   输出:
%     t_log     - 时间向量
%     state_log - 状态矩阵 [Nx12]
%     mode_log  - 模式记录 {'hover','transition','cruise'}
%     energy_log- 能耗记录 [Wh]

    % 加载参数
    evtol_params;
    aero_params;
    controller_params;
    sim_params;

    if nargin < 2 || isempty(duration)
        duration = 120;
    end

    % 创建模块
    dynamics = EvtolDynamics(evtol, aero);
    trans_ctrl = TransitionController(ctrl, evtol, sim.dt);

    % 初始化状态
    dynamics.reset([0; 0; -50], [0; 0; 0], [0; 0; 0], [0; 0; 0]);

    % 预分配日志
    N = ceil(duration / sim.dt);
    t_log = zeros(N, 1);
    state_log = zeros(N, 12);
    mode_log = cell(N, 1);
    energy_log = zeros(N, 1);

    t_log(1) = 0;
    state_log(1, :) = dynamics.state';
    mode_log{1} = 'hover';
    energy_log(1) = 0;

    fprintf('开始 eVTOL 仿真: duration=%.1fs\n', duration);

    for i = 2:N
        t = (i-1) * sim.dt;
        state = dynamics.state;

        % 任务调度
        [pos_des, heading_des, V_des] = evtol_mission_schedule(mission, t, state);

        % 过渡控制
        [omega_cmd, tilt_cmd, mode] = trans_ctrl.update(...
            state, pos_des, heading_des, V_des, sim.dt);

        % 设置倾转角
        dynamics.set_tilt_cmd(tilt_cmd);

        % 动力学积分 (RK4)
        k1 = dynamics.dynamics(t, state, omega_cmd, []);
        k2 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k1, omega_cmd, []);
        k3 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k2, omega_cmd, []);
        k4 = dynamics.dynamics(t + sim.dt, state + sim.dt * k3, omega_cmd, []);
        dynamics.state = state + sim.dt/6 * (k1 + 2*k2 + 2*k3 + k4);

        % 高度约束
        if dynamics.state(3) > 0
            dynamics.state(3) = 0;
            dynamics.state(6) = min(0, dynamics.state(6));
        end

        % 记录
        t_log(i) = t;
        state_log(i, :) = dynamics.state';
        mode_log{i} = mode;
        energy_log(i) = dynamics.energy_used;
    end

    fprintf('仿真完成: %d 步, 能耗: %.2f Wh\n', N, dynamics.energy_used);
end

function [pos_des, heading_des, V_des] = evtol_mission_schedule(mission, t, state)
%EVTOL_MISSION_SCHEDULE  任务调度
    switch mission.type
        case 'transition'
            % 简单过渡测试: 悬停 → 加速 → 巡航
            if t < 10
                pos_des = [0; 0; -50];
                V_des = 0;
            elseif t < 30
                pos_des = [100; 0; -50];
                V_des = 25;
            else
                pos_des = [100; 0; -50];
                V_des = 25;
            end
            heading_des = 0;

        case 'waypoint'
            % 航点任务
            if isfield(mission, 'waypoints') && ~isempty(mission.waypoints)
                wp = mission.waypoints;
                % 简单: 始终朝向第一个航点
                pos_des = wp(1, :)';
                V_des = mission.speed;
                heading_des = atan2(pos_des(2) - state(2), pos_des(1) - state(1));
            else
                pos_des = [0; 0; -50];
                V_des = 0;
                heading_des = 0;
            end

        case 'vtol_mission'
            % 完整任务: 起飞 → 巡航 → 降落
            if t < 10
                % 起飞
                pos_des = [0; 0; -100];
                V_des = 0;
            elseif t < 60
                % 巡航
                pos_des = [500; 0; -100];
                V_des = mission.speed;
            elseif t < 80
                % 减速
                pos_des = [500; 0; -100];
                V_des = 5;
            else
                % 降落
                pos_des = [500; 0; -10];
                V_des = 0;
            end
            heading_des = 0;

        otherwise
            pos_des = [0; 0; -50];
            V_des = 0;
            heading_des = 0;
    end
end
