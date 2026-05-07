function build_fixedwing_model()
%BUILD_FIXEDWING_MODEL  构建固定翼 Simulink 模型
%   运行后生成 fixedwing_control.slx
%   模型包含: 高度控制(升降舵+油门) + 航向控制(副翼+方向舵) + 气动动力学
%
%   用法:
%     cd E:\drone-simulink
%     init_project
%     build_fixedwing_model
%     open_system('fixedwing_control')

    mdl = 'fixedwing_control';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    open_system(mdl);

    set_param(mdl, 'Solver','ode4','FixedStep','0.002','StopTime','60');
    set_param(mdl, 'SaveFormat','StructureWithTime');

    %% ===================== 顶层 =====================

    % --- 期望输入 ---
    add_block('simulink/Sources/Constant', [mdl '/Alt_Des'], ...
        'Position',[30,80,70,100], 'Value','100');
    add_block('simulink/Sources/Constant', [mdl '/Heading_Des'], ...
        'Position',[30,150,70,170], 'Value','0');
    add_block('simulink/Sources/Constant', [mdl '/V_Des'], ...
        'Position',[30,220,70,240], 'Value','25');

    % --- 高度控制子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Altitude_Ctrl'], ...
        'Position',[180,60,380,120]);
    build_alt_ctrl_subsys(mdl);

    % --- 航向控制子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Heading_Ctrl'], ...
        'Position',[180,140,380,210]);
    build_heading_ctrl_subsys(mdl);

    % --- 固定翼动力学子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/FW_Dynamics'], ...
        'Position',[500,60,720,260]);
    build_fw_dynamics_subsys(mdl);

    % --- 气动模型子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Aero_Model'], ...
        'Position',[800,100,980,220]);
    build_aero_subsys(mdl);

    % --- Demux 状态 ---
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
        'Position',[770,80,775,250], 'Outputs','12');

    % --- 反馈 Mux ---
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Fb_Alt'], ...
        'Position',[130,300,135,370], 'Inputs','5');
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Fb_Head'], ...
        'Position',[130,400,135,480], 'Inputs','5');

    % --- Scope ---
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Alt'], ...
        'Position',[1050,60,1090,100], 'NumInputPorts','2');
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Heading'], ...
        'Position',[1050,130,1090,170]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Surfaces'], ...
        'Position',[1050,200,1090,240]);
    add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_State'], ...
        'VariableName','fw_state','Position',[1050,270,1090,300]);

    %% ===================== 连线 =====================
    % 期望 → 控制器
    add_line(mdl, 'Alt_Des/1',     'Altitude_Ctrl/1');
    add_line(mdl, 'Heading_Des/1', 'Heading_Ctrl/1');
    add_line(mdl, 'V_Des/1',       'Altitude_Ctrl/2');

    % 控制器 → 动力学
    add_line(mdl, 'Altitude_Ctrl/1',  'FW_Dynamics/1');  % elevator
    add_line(mdl, 'Altitude_Ctrl/2',  'FW_Dynamics/2');  % throttle
    add_line(mdl, 'Heading_Ctrl/1',   'FW_Dynamics/3');  % aileron
    add_line(mdl, 'Heading_Ctrl/2',   'FW_Dynamics/4');  % rudder

    % 动力学 → Demux
    add_line(mdl, 'FW_Dynamics/1', 'Demux/1');

    % 反馈: 高度
    add_line(mdl, 'Demux/3',  'Mux_Fb_Alt/1');  % z
    add_line(mdl, 'Demux/8',  'Mux_Fb_Alt/2');  % theta
    add_line(mdl, 'Demux/4',  'Mux_Fb_Alt/3');  % vx
    add_line(mdl, 'Demux/5',  'Mux_Fb_Alt/4');  % vy
    add_line(mdl, 'Demux/6',  'Mux_Fb_Alt/5');  % vz
    add_line(mdl, 'Mux_Fb_Alt/1', 'Altitude_Ctrl/3');

    % 反馈: 航向
    add_line(mdl, 'Demux/9',  'Mux_Fb_Head/1');  % psi
    add_line(mdl, 'Demux/7',  'Mux_Fb_Head/2');  % phi
    add_line(mdl, 'Demux/10', 'Mux_Fb_Head/3');  % p
    add_line(mdl, 'Demux/11', 'Mux_Fb_Head/4');  % q
    add_line(mdl, 'Demux/12', 'Mux_Fb_Head/5');  % r
    add_line(mdl, 'Mux_Fb_Head/1', 'Heading_Ctrl/3');

    % Scope
    add_line(mdl, 'Alt_Des/1',   'Scope_Alt/1');
    add_line(mdl, 'Demux/3',     'Scope_Alt/2');
    add_line(mdl, 'Demux/9',     'Scope_Heading/1');
    add_line(mdl, 'FW_Dynamics/1','Scope_Surfaces/1');
    add_line(mdl, 'Demux/1',     'ToWS_State/1');

    %% 保存
    save_system(mdl, fullfile(pwd, [mdl '.slx']));
    fprintf('已生成: %s\n', fullfile(pwd, [mdl '.slx']));
end

%% ==================== 高度控制子系统 ====================
function build_alt_ctrl_subsys(mdl)
    sys = [mdl '/Altitude_Ctrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/alt_des'],  'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/V_des'],    'Position',[20,100,40,115]);
    add_block('simulink/Sources/In1', [sys '/fb'],       'Position',[20,180,40,195]);
    add_block('simulink/Sinks/Out1',  [sys '/elevator'], 'Position',[680,40,700,55]);
    add_block('simulink/Sinks/Out1',  [sys '/throttle'], 'Position',[680,120,700,135]);

    % Demux 反馈: [z, theta, vx, vy, vz]
    add_block('simulink/Signal Routing/Demux', [sys '/Dfb'], 'Position',[70,170,75,230], 'Outputs','5');

    % 高度误差
    add_block('simulink/Math Operations/Gain', [sys '/NegZ'], 'Position',[110,170,140,190], 'Gain','-1');
    add_block('simulink/Math Operations/Sum',  [sys '/Se'],   'Position',[180,40,200,60], 'Inputs','+-');

    % 高度PID → 俯仰角期望
    add_block('simulink/Continuous/PID Controller', [sys '/PID_Alt'], ...
        'Position',[240,35,340,65], 'P','0.5','I','0','D','0.3', ...
        'UpperSaturationLimit','0.52','LowerSaturationLimit','-0.52');

    % 俯仰角误差
    add_block('simulink/Math Operations/Sum', [sys '/Se2'], 'Position',[390,40,410,60], 'Inputs','+-');

    % 俯仰PID → 升降舵
    add_block('simulink/Continuous/PID Controller', [sys '/PID_Pitch'], ...
        'Position',[450,35,550,65], 'P','2.0','I','0.1','D','0.5', ...
        'UpperSaturationLimit','0.52','LowerSaturationLimit','-0.52');

    % 空速计算
    add_block('simulink/Math Operations/Sum of Elements', [sys '/Vsum'], ...
        'Position',[130,100,150,120]);
    add_block('simulink/Math Operations/Math Function', [sys '/Vsqrt'], ...
        'Position',[180,100,210,120], 'Operator','sqrt');
    add_block('simulink/Math Operations/Sum', [sys '/SV'], 'Position',[260,100,280,120], 'Inputs','+-');

    % 空速PID → 油门
    add_block('simulink/Continuous/PID Controller', [sys '/PID_V'], ...
        'Position',[320,95,420,125], 'P','0.3','I','0.05','D','0.01', ...
        'UpperSaturationLimit','1','LowerSaturationLimit','0');

    % 连线
    add_line(sys, 'alt_des/1','Se/1');
    add_line(sys, 'fb/1','Dfb/1');
    add_line(sys, 'Dfb/1','NegZ/1');
    add_line(sys, 'NegZ/1','Se/2');
    add_line(sys, 'Se/1','PID_Alt/1');
    add_line(sys, 'PID_Alt/1','Se2/1');
    add_line(sys, 'Dfb/2','Se2/2');
    add_line(sys, 'Se2/1','PID_Pitch/1');
    add_line(sys, 'PID_Pitch/1','elevator/1');
    add_line(sys, 'Dfb/3','Vsum/1'); add_line(sys, 'Dfb/4','Vsum/1'); add_line(sys, 'Dfb/5','Vsum/1');
    add_line(sys, 'Vsum/1','Vsqrt/1');
    add_line(sys, 'V_des/1','SV/1'); add_line(sys, 'Vsqrt/1','SV/2');
    add_line(sys, 'SV/1','PID_V/1');
    add_line(sys, 'PID_V/1','throttle/1');
end

%% ==================== 航向控制子系统 ====================
function build_heading_ctrl_subsys(mdl)
    sys = [mdl '/Heading_Ctrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/hdg_des'],  'Position',[20,50,40,65]);
    add_block('simulink/Sources/In1', [sys '/fb'],       'Position',[20,150,40,165]);
    add_block('simulink/Sinks/Out1',  [sys '/aileron'],  'Position',[650,40,670,55]);
    add_block('simulink/Sinks/Out1',  [sys '/rudder'],   'Position',[650,130,670,145]);

    % Demux 反馈: [psi, phi, p, q, r]
    add_block('simulink/Signal Routing/Demux', [sys '/Dfb'], 'Position',[70,140,75,200], 'Outputs','5');

    % 航向误差
    add_block('simulink/Math Operations/Sum', [sys '/Se'], 'Position',[130,50,150,70], 'Inputs','+-');

    % 航向PID → 滚转角期望
    add_block('simulink/Continuous/PID Controller', [sys '/PID_Hdg'], ...
        'Position',[190,45,290,75], 'P','1.0','I','0.05','D','0.2', ...
        'UpperSaturationLimit','0.78','LowerSaturationLimit','-0.78');

    % 滚转误差
    add_block('simulink/Math Operations/Sum', [sys '/Se2'], 'Position',[340,45,360,65], 'Inputs','+-');

    % 滚转PID → 副翼
    add_block('simulink/Continuous/PID Controller', [sys '/PID_Roll'], ...
        'Position',[400,40,500,70], 'P','3.0','I','0.1','D','0.5', ...
        'UpperSaturationLimit','0.44','LowerSaturationLimit','-0.44');

    % 侧滑→方向舵 (协调转弯)
    add_block('simulink/Math Operations/Gain', [sys '/Kbeta'], ...
        'Position',[200,150,250,170], 'Gain','-0.5');

    % 连线
    add_line(sys, 'hdg_des/1','Se/1'); add_line(sys, 'Dfb/1','Se/2');
    add_line(sys, 'Se/1','PID_Hdg/1');
    add_line(sys, 'PID_Hdg/1','Se2/1'); add_line(sys, 'Dfb/2','Se2/2');
    add_line(sys, 'Se2/1','PID_Roll/1');
    add_line(sys, 'PID_Roll/1','aileron/1');
    add_line(sys, 'Dfb/3','Kbeta/1');
    add_line(sys, 'Kbeta/1','rudder/1');
end

%% ==================== 固定翼动力学子系统 ====================
function build_fw_dynamics_subsys(mdl)
    sys = [mdl '/FW_Dynamics'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/elev'],    'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/thr'],     'Position',[20,90,40,105]);
    add_block('simulink/Sources/In1', [sys '/ail'],     'Position',[20,140,40,155]);
    add_block('simulink/Sources/In1', [sys '/rud'],     'Position',[20,190,40,205]);
    add_block('simulink/Sinks/Out1',  [sys '/state'],   'Position',[520,100,540,115]);

    % MATLAB Function: 6DOF + 气动
    add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/fw6dof'], ...
        'Position',[180,60,400,160]);

    % 积分器
    add_block('simulink/Continuous/Integrator', [sys '/Int'], ...
        'Position',[430,80,480,140], 'InitialCondition','[0;0;-100;25;0;0;0;0.05;0;0;0;0]');

    add_line(sys, 'elev/1','fw6dof/1');
    add_line(sys, 'thr/1','fw6dof/2');
    add_line(sys, 'ail/1','fw6dof/3');
    add_line(sys, 'rud/1','fw6dof/4');
    add_line(sys, 'Int/1','fw6dof/5');
    add_line(sys, 'fw6dof/1','Int/1');
    add_line(sys, 'Int/1','state/1');
end

%% ==================== 气动模型子系统 ====================
function build_aero_subsys(mdl)
    sys = [mdl '/Aero_Model'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/V_body'], 'Position',[20,50,40,65]);
    add_block('simulink/Sinks/Out1',  [sys '/F_aero'], 'Position',[250,50,270,65]);

    add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/aero_calc'], ...
        'Position',[80,30,200,80]);

    add_line(sys, 'V_body/1','aero_calc/1');
    add_line(sys, 'aero_calc/1','F_aero/1');
end
