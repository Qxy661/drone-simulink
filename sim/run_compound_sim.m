function [t_log, state_log, mode_log, energy_log] = run_compound_sim(mission, duration)
%RUN_COMPOUND_SIM  复合翼仿真主函数
%
%   输入:
%     mission  - 任务结构体 (.type, .altitude, .speed)
%     duration - 仿真时长 [s]
%   输出:
%     t_log, state_log, mode_log, energy_log

    compound_params;
    aero_params;
    controller_params;
    sim_params;

    if nargin < 2 || isempty(duration)
        duration = 120;
    end

    dynamics = CompoundDynamics(cw, aero);

    % 初始化
    dynamics.reset([0; 0; -50], [0; 0; 0], [0; 0; 0], [0; 0; 0]);

    N = ceil(duration / sim.dt);
    t_log = zeros(N, 1);
    state_log = zeros(N, 12);
    mode_log = cell(N, 1);
    energy_log = zeros(N, 1);

    t_log(1) = 0;
    state_log(1, :) = dynamics.state';
    mode_log{1} = 'hover';
    energy_log(1) = 0;

    fprintf('开始复合翼仿真: duration=%.1fs\n', duration);

    for i = 2:N
        t = (i-1) * sim.dt;
        state = dynamics.state;

        % 任务调度
        [lift_cmd, push_cmd, elev, ail, rud, mode] = compound_control(mission, t, state, cw);

        commands = struct('lift_cmd', lift_cmd, 'push_cmd', push_cmd, ...
                         'elevator', elev, 'aileron', ail, 'rudder', rud);

        % 动力学积分 (RK4)
        k1 = dynamics.dynamics(t, state, commands, []);
        k2 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k1, commands, []);
        k3 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k2, commands, []);
        k4 = dynamics.dynamics(t + sim.dt, state + sim.dt * k3, commands, []);
        dynamics.state = state + sim.dt/6 * (k1 + 2*k2 + 2*k3 + k4);

        % 高度约束
        if dynamics.state(3) > 0
            dynamics.state(3) = 0;
            dynamics.state(6) = min(0, dynamics.state(6));
        end

        t_log(i) = t;
        state_log(i, :) = dynamics.state';
        mode_log{i} = mode;
        energy_log(i) = dynamics.energy_used;
    end

    fprintf('仿真完成: %d 步, 能耗: %.2f Wh\n', N, dynamics.energy_used);
end

function [lift_cmd, push_cmd, elev, ail, rud, mode] = compound_control(mission, t, state, cw)
%COMPOUND_control  复合翼控制逻辑

    alt = -state(3);
    vel = state(4:6);
    V = norm(vel);
    att = state(7:9);

    % 默认值
    elev = 0; ail = 0; rud = 0;

    switch mission.type
        case 'vtol'
            % 垂直起降任务
            if t < 15
                % 起飞
                mode = 'hover';
                alt_des = 100;
                lift_cmd = ones(4, 1) * sqrt(cw.mass * cw.g / (cw.n_lift * cw.lift_kt));
                push_cmd = 0;
            elseif t < 60
                % 巡航
                mode = 'cruise';
                alt_des = 100;
                push_cmd = cw.push_omega_max * 0.6;
                lift_cmd = ones(4, 1) * cw.omega_hover * 0.3;  % 辅助
            elseif t < 80
                % 减速
                mode = 'transition';
                alt_des = 100;
                push_cmd = cw.push_omega_max * 0.2;
                lift_cmd = ones(4, 1) * cw.omega_hover * 0.8;
            else
                % 降落
                mode = 'hover';
                alt_des = 10;
                lift_cmd = ones(4, 1) * sqrt(cw.mass * cw.g * 0.8 / (cw.n_lift * cw.lift_kt));
                push_cmd = 0;
            end

            % 高度修正
            alt_err = alt_des - alt;
            Kp_z = 2.0;
            lift_correction = Kp_z * alt_err;
            lift_cmd = lift_cmd + lift_correction;
            lift_cmd = max(0, min(cw.lift_omega_max, lift_cmd));

        case 'hover_test'
            mode = 'hover';
            lift_cmd = ones(4, 1) * cw.omega_hover;
            push_cmd = 0;

        otherwise
            mode = 'hover';
            lift_cmd = ones(4, 1) * cw.omega_hover;
            push_cmd = 0;
    end
end
