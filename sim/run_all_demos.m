function run_all_demos()
%RUN_ALL_DEMOS  运行全部构型演示, 生成对比结果
%   用法:
%     cd E:\drone-simulink
%     init_project
%     run_all_demos

    fprintf('╔══════════════════════════════════════════╗\n');
    fprintf('║    Drone Simulink — 全构型仿真演示       ║\n');
    fprintf('╚══════════════════════════════════════════╝\n\n');

    %% 1. 四旋翼悬停
    fprintf('━━━ [1/5] 四旋翼悬停 ━━━\n');
    quad_params; motor_params; controller_params; sim_params;
    traj = TrajectoryGenerator(); traj.set_hover([0;0;5]);
    [t1, s1, ~] = run_quad_sim(traj, 20);
    plot_results(t1, s1);
    print_perf_quad(t1, s1, '悬停');

    %% 2. 四旋翼圆形轨迹
    fprintf('\n━━━ [2/5] 四旋翼圆形轨迹 ━━━\n');
    traj2 = TrajectoryGenerator(); traj2.set_circle([0;0;5], 3, 10);
    [t2, s2, ~] = run_quad_sim(traj2, 30);
    plot_results(t2, s2);
    print_perf_quad(t2, s2, '圆形轨迹');

    %% 3. 固定翼平飞
    fprintf('\n━━━ [3/5] 固定翼定常平飞 ━━━\n');
    fixedwing_params; aero_params;
    fw_traj = struct('type','line','altitude',100,'heading',0,'speed',fw.V_cruise);
    [t3, s3, c3] = run_fixedwing_sim(fw_traj, 30);
    plot_fixedwing_results(t3, s3, c3);

    %% 4. eVTOL 过渡
    fprintf('\n━━━ [4/5] eVTOL 悬停→巡航过渡 ━━━\n');
    evtol_params;
    mission = struct('type','transition','speed',25);
    [t4, s4, m4, e4] = run_evtol_sim(mission, 60);

    %% 5. 复合翼任务
    fprintf('\n━━━ [5/5] 复合翼 VTOL 任务 ━━━\n');
    compound_params;
    mission5 = struct('type','vtol','speed',25);
    [t5, s5, m5, e5] = run_compound_sim(mission5, 100);

    %% 汇总
    fprintf('\n╔══════════════════════════════════════════╗\n');
    fprintf('║           全部仿真完成                    ║\n');
    fprintf('╚══════════════════════════════════════════╝\n');
    fprintf('  四旋翼悬停:   %d 步\n', length(t1));
    fprintf('  四旋翼圆形:   %d 步\n', length(t2));
    fprintf('  固定翼平飞:   %d 步\n', length(t3));
    fprintf('  eVTOL 过渡:   %d 步\n', length(t4));
    fprintf('  复合翼任务:   %d 步\n', length(t5));
end

function print_perf_quad(t, state, name)
    N = length(t);
    idx = round(N/2):N;
    pos_err = mean(abs(state(idx, 1:3) - mean(state(idx, 1:3))));
    z_std = std(state(idx, 3));
    fprintf('  [%s] X/Y/Z 稳态误差: %.3f/%.3f/%.3f m, Z振荡: %.4f m\n', ...
        name, pos_err(1), pos_err(2), pos_err(3), z_std);
end
