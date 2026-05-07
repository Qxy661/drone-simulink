function build_compound_model()
%BUILD_COMPOUND_MODEL  构建复合翼 Simulink 模型
%   运行后生成 compound_vtol.slx
%   模型包含: 升力旋翼+推进器+气动面 全包线控制
%
%   用法:
%     cd E:\drone-simulink
%     init_project
%     build_compound_model
%     open_system('compound_vtol')

    mdl = 'compound_vtol';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    open_system(mdl);

    set_param(mdl, 'Solver','ode4','FixedStep','0.002','StopTime','100');

    %% ===================== 顶层 =====================

    % --- 任务输入 ---
    add_block('simulink/Sources/Step', [mdl '/Alt_Cmd'], ...
        'Position',[30,80,70,100], 'Time','0','After','100','Before','0');
    add_block('simulink/Sources/Step', [mdl '/V_Cmd'], ...
        'Position',[30,150,70,170], 'Time','20','After','25','Before','0');

    % --- 模式管理器 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Mode_Mgr'], ...
        'Position',[160,60,340,180]);
    build_mode_mgr_subsys(mdl);

    % --- 升力旋翼控制 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Lift_Ctrl'], ...
        'Position',[400,40,580,120]);
    build_lift_ctrl_subsys(mdl);

    % --- 推进器控制 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Push_Ctrl'], ...
        'Position',[400,140,580,220]);
    build_push_ctrl_subsys(mdl);

    % --- 舵面控制 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Surface_Ctrl'], ...
        'Position',[400,240,580,320]);
    build_surface_ctrl_subsys(mdl);

    % --- 复合翼动力学 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Compound_Dyn'], ...
        'Position',[660,60,880,280]);
    build_compound_dyn_subsys(mdl);

    % --- Demux ---
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
        'Position',[930,80,935,260], 'Outputs','12');

    % --- 反馈 Mux ---
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Fb'], ...
        'Position',[100,350,105,450], 'Inputs','6');

    % --- Scope ---
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Pos'], ...
        'Position',[1000,50,1040,90]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Alt'], ...
        'Position',[1000,120,1040,160]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Motor'], ...
        'Position',[1000,190,1040,230]);
    add_block('simulink/Sinks/To Workspace', [mdl '/ToWS'], ...
        'VariableName','cw_state','Position',[1000,260,1040,290]);

    %% ===================== 连线 =====================
    add_line(mdl, 'Alt_Cmd/1', 'Mode_Mgr/1');
    add_line(mdl, 'V_Cmd/1',   'Mode_Mgr/2');

    add_line(mdl, 'Mode_Mgr/1', 'Lift_Ctrl/1');
    add_line(mdl, 'Mode_Mgr/2', 'Push_Ctrl/1');
    add_line(mdl, 'Mode_Mgr/1', 'Surface_Ctrl/1');
    add_line(mdl, 'Alt_Cmd/1',  'Lift_Ctrl/2');

    add_line(mdl, 'Lift_Ctrl/1',    'Compound_Dyn/1');
    add_line(mdl, 'Push_Ctrl/1',    'Compound_Dyn/2');
    add_line(mdl, 'Surface_Ctrl/1', 'Compound_Dyn/3');
    add_line(mdl, 'Surface_Ctrl/2', 'Compound_Dyn/4');
    add_line(mdl, 'Surface_Ctrl/3', 'Compound_Dyn/5');

    add_line(mdl, 'Compound_Dyn/1', 'Demux/1');

    % 反馈
    add_line(mdl, 'Demux/1', 'Mux_Fb/1');
    add_line(mdl, 'Demux/2', 'Mux_Fb/2');
    add_line(mdl, 'Demux/3', 'Mux_Fb/3');
    add_line(mdl, 'Demux/7', 'Mux_Fb/4');
    add_line(mdl, 'Demux/8', 'Mux_Fb/5');
    add_line(mdl, 'Demux/9', 'Mux_Fb/6');
    add_line(mdl, 'Mux_Fb/1','Mode_Mgr/3');

    add_line(mdl, 'Demux/3', 'Scope_Alt/1');
    add_line(mdl, 'Demux/1', 'ToWS/1');

    save_system(mdl, fullfile(pwd, [mdl '.slx']));
    fprintf('已生成: %s\n', fullfile(pwd, [mdl '.slx']));
end

%% ==================== 模式管理器 ====================
function build_mode_mgr_subsys(mdl)
    sys = [mdl '/Mode_Mgr'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/alt_cmd'], 'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/V_cmd'],   'Position',[20,100,40,115]);
    add_block('simulink/Sources/In1', [sys '/fb'],      'Position',[20,160,40,175]);
    add_block('simulink/Sinks/Out1',  [sys '/mode'],    'Position',[300,40,320,55]);
    add_block('simulink/Sinks/Out1',  [sys '/V_des'],   'Position',[300,100,320,115]);

    % mode: 0=hover, 1=transition, 2=cruise
    add_block('simulink/Signal Routing/Demux', [sys '/Dfb'], 'Position',[70,150,75,210], 'Outputs','6');
    add_block('simulink/Math Operations/Sum of Elements', [sys '/Vsum'], 'Position',[120,40,140,60]);
    add_block('simulink/Math Operations/Math Function', [sys '/Vsqrt'], 'Position',[170,40,200,60], 'Operator','sqrt');

    % mode = V < 5 ? 0 : (V > 20 ? 2 : 1)
    add_block('simulink/Discontinuities/Dead Zone', [sys '/DZ'], ...
        'Position',[230,40,270,60], 'LowerValue','-5','UpperValue','-5');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat'], ...
        'Position',[280,40,310,60], 'UpperLimit','2','LowerLimit','0');

    add_line(sys, 'V_cmd/1','Vsum/1');
    add_line(sys, 'Vsum/1','Vsqrt/1');
    add_line(sys, 'Vsqrt/1','DZ/1');
    add_line(sys, 'DZ/1','Sat/1');
    add_line(sys, 'Sat/1','mode/1');
    add_line(sys, 'V_cmd/1','V_des/1');
end

%% ==================== 升力旋翼控制 ====================
function build_lift_ctrl_subsys(mdl)
    sys = [mdl '/Lift_Ctrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/mode'],    'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/alt_des'], 'Position',[20,100,40,115]);
    add_block('simulink/Sinks/Out1',  [sys '/lift_cmd'],'Position',[350,60,370,75]);

    % hover omega = sqrt(m*g/(4*kt))
    add_block('simulink/Sources/Constant', [sys '/w0'], 'Position',[80,60,120,80], 'Value','530');
    add_block('simulink/Math Operations/Sum', [sys '/Se'], 'Position',[160,40,180,60], 'Inputs','+-');
    add_block('simulink/Continuous/PID Controller', [sys '/PID_Z'], ...
        'Position',[210,35,300,65], 'P','2.0','I','0.3','D','1.0', ...
        'UpperSaturationLimit','300','LowerSaturationLimit','-300');
    add_block('simulink/Math Operations/Sum', [sys '/Sw'], 'Position',[310,55,330,75], 'Inputs','++');

    add_line(sys, 'alt_des/1','Se/1'); add_line(sys, 'Se/1','PID_Z/1');
    add_line(sys, 'w0/1','Sw/1'); add_line(sys, 'PID_Z/1','Sw/2');
    add_line(sys, 'Sw/1','lift_cmd/1');
end

%% ==================== 推进器控制 ====================
function build_push_ctrl_subsys(mdl)
    sys = [mdl '/Push_Ctrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/V_des'],    'Position',[20,50,40,65]);
    add_block('simulink/Sinks/Out1',  [sys '/push_cmd'], 'Position',[250,50,270,65]);

    add_block('simulink/Math Operations/Gain', [sys '/G'], 'Position',[100,45,150,65], 'Gain','60');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat'], ...
        'Position',[180,45,220,65], 'UpperLimit','2000','LowerLimit','0');

    add_line(sys, 'V_des/1','G/1'); add_line(sys, 'G/1','Sat/1'); add_line(sys, 'Sat/1','push_cmd/1');
end

%% ==================== 舵面控制 ====================
function build_surface_ctrl_subsys(mdl)
    sys = [mdl '/Surface_Ctrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/mode'], 'Position',[20,50,40,65]);
    add_block('simulink/Sinks/Out1',  [sys '/elev'], 'Position',[250,30,270,45]);
    add_block('simulink/Sinks/Out1',  [sys '/ail'],  'Position',[250,70,270,85]);
    add_block('simulink/Sinks/Out1',  [sys '/rud'],  'Position',[250,110,270,125]);

    % 简化: mode>0 时舵面有效
    add_block('simulink/Sources/Constant', [sys '/zero1'], 'Position',[100,30,130,45], 'Value','0');
    add_block('simulink/Sources/Constant', [sys '/zero2'], 'Position',[100,70,130,85], 'Value','0');
    add_block('simulink/Sources/Constant', [sys '/zero3'], 'Position',[100,110,130,125],'Value','0');

    add_line(sys, 'zero1/1','elev/1');
    add_line(sys, 'zero2/1','ail/1');
    add_line(sys, 'zero3/1','rud/1');
end

%% ==================== 复合翼动力学子系统 ====================
function build_compound_dyn_subsys(mdl)
    sys = [mdl '/Compound_Dyn'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/lift'],  'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/push'],  'Position',[20,90,40,105]);
    add_block('simulink/Sources/In1', [sys '/elev'],  'Position',[20,140,40,155]);
    add_block('simulink/Sources/In1', [sys '/ail'],   'Position',[20,190,40,205]);
    add_block('simulink/Sources/In1', [sys '/rud'],   'Position',[20,240,40,255]);
    add_block('simulink/Sinks/Out1',  [sys '/state'], 'Position',[420,120,440,135]);

    add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/cw6dof'], ...
        'Position',[160,60,320,200]);

    add_block('simulink/Continuous/Integrator', [sys '/Int'], ...
        'Position',[340,90,380,170], 'InitialCondition','[0;0;-50;0;0;0;0;0;0;0;0;0]');

    add_line(sys, 'lift/1','cw6dof/1');
    add_line(sys, 'push/1','cw6dof/2');
    add_line(sys, 'elev/1','cw6dof/3');
    add_line(sys, 'ail/1','cw6dof/4');
    add_line(sys, 'rud/1','cw6dof/5');
    add_line(sys, 'Int/1','cw6dof/6');
    add_line(sys, 'cw6dof/1','Int/1');
    add_line(sys, 'Int/1','state/1');
end
