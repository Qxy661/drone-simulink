function build_evtol_model()
%BUILD_EVTOL_MODEL  构建 eVTOL 倾转旋翼 Simulink 模型
%   运行后生成 evtol_transition.slx
%   模型包含: 悬停/过渡/巡航三模式控制 + 倾转机构 + 混合动力学
%
%   用法:
%     cd E:\drone-simulink
%     init_project
%     build_evtol_model
%     open_system('evtol_transition')

    mdl = 'evtol_transition';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    open_system(mdl);

    set_param(mdl, 'Solver','ode4','FixedStep','0.002','StopTime','120');

    %% ===================== 顶层 =====================

    % --- 任务输入 ---
    add_block('simulink/Sources/Constant', [mdl '/Alt_Des'], ...
        'Position',[30,80,70,100], 'Value','50');
    add_block('simulink/Sources/Step', [mdl '/V_Des'], ...
        'Position',[30,150,70,170], 'Time','15','After','25','Before','0');

    % --- 模式调度子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Mode_Scheduler'], ...
        'Position',[160,60,340,180]);
    build_mode_scheduler_subsys(mdl);

    % --- 悬停控制子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Hover_Ctrl'], ...
        'Position',[400,40,580,120]);
    build_hover_ctrl_subsys(mdl);

    % --- 巡航控制子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Cruise_Ctrl'], ...
        'Position',[400,140,580,220]);
    build_cruise_ctrl_subsys(mdl);

    % --- 混合器 (悬停↔巡航) ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Blend'], ...
        'Position',[640,60,800,200]);
    build_blend_subsys(mdl);

    % --- eVTOL 动学子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/eVTOL_Dyn'], ...
        'Position',[860,60,1080,220]);
    build_evtol_dyn_subsys(mdl);

    % --- Demux ---
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
        'Position',[1130,80,1135,210], 'Outputs','12');

    % --- 反馈 Mux ---
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Fb'], ...
        'Position',[100,280,105,380], 'Inputs','6');

    % --- Scope ---
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Pos'], ...
        'Position',[1200,50,1240,90]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Mode'], ...
        'Position',[1200,120,1240,160]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Tilt'], ...
        'Position',[1200,190,1240,230]);
    add_block('simulink/Sinks/To Workspace', [mdl '/ToWS'], ...
        'VariableName','evtol_state','Position',[1200,260,1240,290]);

    %% ===================== 连线 =====================
    add_line(mdl, 'Alt_Des/1', 'Mode_Scheduler/1');
    add_line(mdl, 'V_Des/1',   'Mode_Scheduler/2');

    add_line(mdl, 'Mode_Scheduler/1', 'Hover_Ctrl/1');   % mode
    add_line(mdl, 'Mode_Scheduler/2', 'Cruise_Ctrl/1');  % mode
    add_line(mdl, 'Alt_Des/1',        'Hover_Ctrl/2');
    add_line(mdl, 'Alt_Des/1',        'Cruise_Ctrl/2');
    add_line(mdl, 'V_Des/1',          'Cruise_Ctrl/3');

    add_line(mdl, 'Hover_Ctrl/1',  'Blend/1');   % hover omega
    add_line(mdl, 'Cruise_Ctrl/1', 'Blend/2');   % cruise push
    add_line(mdl, 'Mode_Scheduler/1','Blend/3');  % mode/alpha

    add_line(mdl, 'Blend/1',  'eVTOL_Dyn/1');  % lift_cmd
    add_line(mdl, 'Blend/2',  'eVTOL_Dyn/2');  % push_cmd
    add_line(mdl, 'Blend/3',  'eVTOL_Dyn/3');  % tilt_cmd

    add_line(mdl, 'eVTOL_Dyn/1', 'Demux/1');

    % 反馈
    add_line(mdl, 'Demux/1', 'Mux_Fb/1');
    add_line(mdl, 'Demux/2', 'Mux_Fb/2');
    add_line(mdl, 'Demux/3', 'Mux_Fb/3');
    add_line(mdl, 'Demux/7', 'Mux_Fb/4');
    add_line(mdl, 'Demux/8', 'Mux_Fb/5');
    add_line(mdl, 'Demux/9', 'Mux_Fb/6');
    add_line(mdl, 'Mux_Fb/1','Mode_Scheduler/3');

    add_line(mdl, 'Demux/3', 'Scope_Pos/1');
    add_line(mdl, 'Blend/3', 'Scope_Tilt/1');
    add_line(mdl, 'Demux/1','ToWS/1');

    save_system(mdl, fullfile(pwd, [mdl '.slx']));
    fprintf('已生成: %s\n', fullfile(pwd, [mdl '.slx']));
end

%% ==================== 模式调度器 ====================
function build_mode_scheduler_subsys(mdl)
    sys = [mdl '/Mode_Scheduler'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/alt_des'], 'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/V_des'],   'Position',[20,100,40,115]);
    add_block('simulink/Sources/In1', [sys '/fb'],      'Position',[20,160,40,175]);
    add_block('simulink/Sinks/Out1',  [sys '/mode'],    'Position',[350,40,370,55]);
    add_block('simulink/Sinks/Out1',  [sys '/alpha'],   'Position',[350,120,370,135]);

    % Demux 反馈: [x,y,z,phi,theta,psi]
    add_block('simulink/Signal Routing/Demux', [sys '/Dfb'], 'Position',[70,150,75,210], 'Outputs','6');

    % 空速估计 (简化: norm(vx,vy))
    add_block('simulink/Math Operations/Sum of Elements', [sys '/Vsum'], ...
        'Position',[120,50,140,70]);
    add_block('simulink/Math Operations/Math Function', [sys '/Vsqrt'], ...
        'Position',[170,50,200,70], 'Operator','sqrt');

    % 倾转比例: alpha = clamp((V-5)/(20-5), 0, 1)
    add_block('simulink/Math Operations/Bias', [sys '/Vbias'], ...
        'Position',[230,50,260,70], 'Bias','-5');
    add_block('simulink/Math Operations/Gain', [sys '/Vscale'], ...
        'Position',[280,50,310,70], 'Gain','0.0667');
    add_block('simulink/Discontinuities/Saturation', [sys '/Vsat'], ...
        'Position',[320,50,350,70], 'UpperLimit','1','LowerLimit','0');

    % mode 输出 (0=hover, 1=cruise)
    add_line(sys, 'V_des/1','Vsum/1');
    add_line(sys, 'Vsum/1','Vsqrt/1');
    add_line(sys, 'Vsqrt/1','Vbias/1');
    add_line(sys, 'Vbias/1','Vscale/1');
    add_line(sys, 'Vscale/1','Vsat/1');
    add_line(sys, 'Vsat/1','alpha/1');
    add_line(sys, 'Vsat/1','mode/1');
end

%% ==================== 悬停控制子系统 ====================
function build_hover_ctrl_subsys(mdl)
    sys = [mdl '/Hover_Ctrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/mode'],   'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/alt_des'],'Position',[20,100,40,115]);
    add_block('simulink/Sinks/Out1',  [sys '/omega_cmd'],'Position',[400,60,420,75]);

    % 简化: hover omega = sqrt(m*g/(4*kt))
    add_block('simulink/Sources/Constant', [sys '/hover_w'], ...
        'Position',[100,60,150,80], 'Value','398');

    % 高度修正 PID
    add_block('simulink/Math Operations/Sum', [sys '/Se'], 'Position',[180,40,200,60], 'Inputs','+-');
    add_block('simulink/Continuous/PID Controller', [sys '/PID_Z'], ...
        'Position',[230,35,320,65], 'P','2.0','I','0.3','D','1.0', ...
        'UpperSaturationLimit','200','LowerSaturationLimit','-200');
    add_block('simulink/Math Operations/Sum', [sys '/Sw'], 'Position',[350,55,370,75], 'Inputs','++');

    add_line(sys, 'alt_des/1','Se/1');
    add_line(sys, 'Se/1','PID_Z/1');
    add_line(sys, 'hover_w/1','Sw/1');
    add_line(sys, 'PID_Z/1','Sw/2');
    add_line(sys, 'Sw/1','omega_cmd/1');
end

%% ==================== 巡航控制子系统 ====================
function build_cruise_ctrl_subsys(mdl)
    sys = [mdl '/Cruise_Ctrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/mode'],     'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/alt_des'],  'Position',[20,100,40,115]);
    add_block('simulink/Sources/In1', [sys '/V_des'],    'Position',[20,160,40,175]);
    add_block('simulink/Sinks/Out1',  [sys '/push_cmd'], 'Position',[350,60,370,75]);

    % 推进器转速 = f(V_des)
    add_block('simulink/Math Operations/Gain', [sys '/V2omega'], ...
        'Position',[150,150,200,170], 'Gain','40');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat'], ...
        'Position',[250,150,300,170], 'UpperLimit','2000','LowerLimit','0');

    add_line(sys, 'V_des/1','V2omega/1');
    add_line(sys, 'V2omega/1','Sat/1');
    add_line(sys, 'Sat/1','push_cmd/1');
end

%% ==================== 混合器 ====================
function build_blend_subsys(mdl)
    sys = [mdl '/Blend'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/hover_w'],   'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/cruise_p'],   'Position',[20,100,40,115]);
    add_block('simulink/Sources/In1', [sys '/alpha'],      'Position',[20,160,40,175]);
    add_block('simulink/Sinks/Out1',  [sys '/lift_cmd'],   'Position',[400,40,420,55]);
    add_block('simulink/Sinks/Out1',  [sys '/push_cmd'],   'Position',[400,100,420,115]);
    add_block('simulink/Sinks/Out1',  [sys '/tilt_cmd'],   'Position',[400,160,420,175]);

    % lift = (1-alpha) * hover_w
    add_block('simulink/Math Operations/Sum',  [sys '/Sa'], 'Position',[100,155,120,175], 'Inputs','+-');
    add_block('simulink/Sources/Constant',     [sys '/one'],'Position',[50,170,70,190], 'Value','1');
    add_block('simulink/Math Operations/Product', [sys '/Pl'], 'Position',[200,40,220,60]);

    % push = alpha * cruise_p
    add_block('simulink/Math Operations/Product', [sys '/Pp'], 'Position',[200,100,220,120]);

    % tilt = alpha * pi/2
    add_block('simulink/Math Operations/Gain', [sys '/toRad'], ...
        'Position',[250,155,290,175], 'Gain','1.5708');

    add_line(sys, 'one/1','Sa/1'); add_line(sys, 'alpha/1','Sa/2');
    add_line(sys, 'Sa/1','Pl/1'); add_line(sys, 'hover_w/1','Pl/2');
    add_line(sys, 'Pl/1','lift_cmd/1');
    add_line(sys, 'alpha/1','Pp/1'); add_line(sys, 'cruise_p/1','Pp/2');
    add_line(sys, 'Pp/1','push_cmd/1');
    add_line(sys, 'alpha/1','toRad/1');
    add_line(sys, 'toRad/1','tilt_cmd/1');
end

%% ==================== eVTOL 动学子系统 ====================
function build_evtol_dyn_subsys(mdl)
    sys = [mdl '/eVTOL_Dyn'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/lift_cmd'], 'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/push_cmd'], 'Position',[20,90,40,105]);
    add_block('simulink/Sources/In1', [sys '/tilt_cmd'], 'Position',[20,140,40,155]);
    add_block('simulink/Sinks/Out1',  [sys '/state'],    'Position',[420,80,440,95]);

    add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/ev6dof'], ...
        'Position',[150,50,320,130]);

    add_block('simulink/Continuous/Integrator', [sys '/Int'], ...
        'Position',[340,65,380,115], 'InitialCondition','[0;0;-50;0;0;0;0;0;0;0;0;0]');

    add_line(sys, 'lift_cmd/1','ev6dof/1');
    add_line(sys, 'push_cmd/1','ev6dof/2');
    add_line(sys, 'tilt_cmd/1','ev6dof/3');
    add_line(sys, 'Int/1','ev6dof/4');
    add_line(sys, 'ev6dof/1','Int/1');
    add_line(sys, 'Int/1','state/1');
end
