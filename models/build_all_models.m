function build_all_models()
%BUILD_ALL_MODELS  构建所有构型的 Simulink 模型
%   一次性生成全部 .slx 文件
%
%   用法:
%     cd E:\drone-simulink
%     init_project
%     build_all_models
%
%   生成:
%     quad_cascade_pid.slx   - 四旋翼级联PID控制
%     fixedwing_control.slx  - 固定翼高度+航向控制
%     evtol_transition.slx   - eVTOL悬停↔巡航过渡
%     compound_vtol.slx      - 复合翼全包线控制

    fprintf('========================================\n');
    fprintf('  构建全部 Simulink 模型\n');
    fprintf('========================================\n\n');

    t0 = tic;

    try
        fprintf('[1/4] 四旋翼级联PID...\n');
        build_quad_model;
        fprintf('  ✓ quad_cascade_pid.slx\n\n');
    catch e
        fprintf('  ✗ 四旋翼失败: %s\n\n', e.message);
    end

    try
        fprintf('[2/4] 固定翼控制...\n');
        build_fixedwing_model;
        fprintf('  ✓ fixedwing_control.slx\n\n');
    catch e
        fprintf('  ✗ 固定翼失败: %s\n\n', e.message);
    end

    try
        fprintf('[3/4] eVTOL过渡...\n');
        build_evtol_model;
        fprintf('  ✓ evtol_transition.slx\n\n');
    catch e
        fprintf('  ✗ eVTOL失败: %s\n\n', e.message);
    end

    try
        fprintf('[4/4] 复合翼控制...\n');
        build_compound_model;
        fprintf('  ✓ compound_vtol.slx\n\n');
    catch e
        fprintf('  ✗ 复合翼失败: %s\n\n', e.message);
    end

    fprintf('========================================\n');
    fprintf('  完成! 耗时 %.1f 秒\n', toc(t0));
    fprintf('========================================\n');
    fprintf('\n打开模型:\n');
    fprintf('  open_system(''quad_cascade_pid'')\n');
    fprintf('  open_system(''fixedwing_control'')\n');
    fprintf('  open_system(''evtol_transition'')\n');
    fprintf('  open_system(''compound_vtol'')\n');
end
