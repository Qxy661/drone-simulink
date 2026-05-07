function build_quad_model()
%BUILD_QUAD_MODEL  构建四旋翼级联PID Simulink模型
%   运行后生成 quad_cascade_pid.slx
%   模型包含完整控制框图: 轨迹→位置环→姿态环→混控→电机→动力学→传感器
%
%   用法:
%     cd E:\drone-simulink
%     init_project
%     build_quad_model
%     open_system('quad_cascade_pid')

    mdl = 'quad_cascade_pid';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    open_system(mdl);

    %% 模型配置
    set_param(mdl, 'Solver','ode4','FixedStep','0.001','StopTime','20');
    set_param(mdl, 'SaveFormat','StructureWithTime');

    %% ===================== 顶层模块 =====================

    % --- 轨迹生成 ---
    add_block('simulink/Sources/Step', [mdl '/Step_X'], ...
        'Position',[30,100,60,120], 'Time','2','After','5','Before','0');
    add_block('simulink/Sources/Step', [mdl '/Step_Y'], ...
        'Position',[30,160,60,180], 'Time','5','After','3','Before','0');
    add_block('simulink/Sources/Step', [mdl '/Step_Z'], ...
        'Position',[30,220,60,240], 'Time','0','After','5','Before','0');
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Pos_Des'], ...
        'Position',[110,110,115,235], 'Inputs','3');

    % --- 位置环子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Position_Ctrl'], ...
        'Position',[220,100,400,250]);
    build_pos_ctrl_subsys(mdl);

    % --- 姿态环子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Attitude_Ctrl'], ...
        'Position',[470,80,650,270]);
    build_att_ctrl_subsys(mdl);

    % --- 混控器子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Mixer'], ...
        'Position',[720,100,880,250]);
    build_mixer_subsys(mdl);

    % --- 电机子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Motors'], ...
        'Position',[950,100,1100,250]);
    build_motor_subsys(mdl);

    % --- 动学子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Dynamics_6DOF'], ...
        'Position',[1170,80,1380,270]);
    build_dynamics_subsys(mdl);

    % --- 传感器子系统 ---
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Sensors'], ...
        'Position',[1050,320,1200,430]);
    build_sensor_subsys(mdl);

    % --- Scope ---
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Pos'], ...
        'Position',[1480,60,1520,100], 'NumInputPorts','2');
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Att'], ...
        'Position',[1480,130,1520,170]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Motor'], ...
        'Position',[1480,200,1520,240]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope_3D'], ...
        'Position',[1480,270,1520,310]);

    % --- To Workspace ---
    add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Pos'], ...
        'VariableName','pos_log','Position',[1480,340,1520,370]);
    add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Att'], ...
        'VariableName','att_log','Position',[1480,390,1520,420]);

    % --- 反馈 Mux ---
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Fb_Pos'], ...
        'Position',[160,350,165,430], 'Inputs','3');
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Fb_Att'], ...
        'Position',[420,350,425,480], 'Inputs','6');
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux_State'], ...
        'Position',[1430,100,1435,260], 'Outputs','12');

    %% ===================== 顶层连线 =====================
    % 轨迹 → Mux
    add_line(mdl, 'Step_X/1', 'Mux_Pos_Des/1');
    add_line(mdl, 'Step_Y/1', 'Mux_Pos_Des/2');
    add_line(mdl, 'Step_Z/1', 'Mux_Pos_Des/3');
    % Mux → 位置环
    add_line(mdl, 'Mux_Pos_Des/1', 'Position_Ctrl/1');
    % 位置环 → 姿态环
    add_line(mdl, 'Position_Ctrl/1', 'Attitude_Ctrl/1');  % att_des
    add_line(mdl, 'Position_Ctrl/2', 'Attitude_Ctrl/2');  % thrust
    % 姿态环 → 混控器 (力矩)
    add_line(mdl, 'Attitude_Ctrl/1', 'Mixer/2');
    % 位置环 thrust → 混控器
    add_line(mdl, 'Position_Ctrl/2', 'Mixer/1');
    % 混控器 → 电机
    add_line(mdl, 'Mixer/1', 'Motors/1');
    % 电机 → 动力学
    add_line(mdl, 'Motors/1', 'Dynamics_6DOF/1');
    % 动力学 → Demux
    add_line(mdl, 'Dynamics_6DOF/1', 'Demux_State/1');
    % 反馈: pos
    add_line(mdl, 'Demux_State/1', 'Mux_Fb_Pos/1');
    add_line(mdl, 'Demux_State/2', 'Mux_Fb_Pos/2');
    add_line(mdl, 'Demux_State/3', 'Mux_Fb_Pos/3');
    add_line(mdl, 'Mux_Fb_Pos/1', 'Position_Ctrl/2');
    % 反馈: att+rates
    add_line(mdl, 'Demux_State/7',  'Mux_Fb_Att/1');
    add_line(mdl, 'Demux_State/8',  'Mux_Fb_Att/2');
    add_line(mdl, 'Demux_State/9',  'Mux_Fb_Att/3');
    add_line(mdl, 'Demux_State/10', 'Mux_Fb_Att/4');
    add_line(mdl, 'Demux_State/11', 'Mux_Fb_Att/5');
    add_line(mdl, 'Demux_State/12', 'Mux_Fb_Att/6');
    add_line(mdl, 'Mux_Fb_Att/1', 'Attitude_Ctrl/3');
    % Scope
    add_line(mdl, 'Mux_Pos_Des/1', 'Scope_Pos/1');
    add_line(mdl, 'Mux_Fb_Pos/1',  'Scope_Pos/2');
    add_line(mdl, 'Mux_Fb_Att/1',  'Scope_Att/1');
    add_line(mdl, 'Mixer/1',        'Scope_Motor/1');
    add_line(mdl, 'Demux_State/1',  'ToWS_Pos/1');
    add_line(mdl, 'Demux_State/7',  'ToWS_Att/1');

    %% 保存
    save_system(mdl, fullfile(pwd, [mdl '.slx']));
    fprintf('已生成: %s\n', fullfile(pwd, [mdl '.slx']));
end

%% ==================== 位置环子系统 ====================
function build_pos_ctrl_subsys(mdl)
    sys = [mdl '/Position_Ctrl'];
    % 删除默认连线
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    % 端口
    add_block('simulink/Sources/In1',     [sys '/pos_des'], 'Position',[20,60,40,75]);
    add_block('simulink/Sources/In1',     [sys '/pos_cur'], 'Position',[20,160,40,175]);
    add_block('simulink/Sinks/Out1',      [sys '/att_des'], 'Position',[680,60,700,75]);
    add_block('simulink/Sinks/Out1',      [sys '/thrust'],  'Position',[680,160,700,175]);

    % Demux
    add_block('simulink/Signal Routing/Demux', [sys '/D1'], 'Position',[80,45,85,100], 'Outputs','3');
    add_block('simulink/Signal Routing/Demux', [sys '/D2'], 'Position',[80,145,85,200], 'Outputs','3');

    % Sum (误差)
    add_block('simulink/Math Operations/Sum', [sys '/Sx'], 'Position',[140,50,160,70], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Sy'], 'Position',[140,100,160,120], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Sz'], 'Position',[140,150,160,170], 'Inputs','+-');

    % PID
    add_block('simulink/Continuous/PID Controller', [sys '/PIDx'], ...
        'Position',[200,45,280,75], 'P','1.5','I','0.2','D','0.8', ...
        'UpperSaturationLimit','10','LowerSaturationLimit','-10');
    add_block('simulink/Continuous/PID Controller', [sys '/PIDy'], ...
        'Position',[200,95,280,125], 'P','1.5','I','0.2','D','0.8', ...
        'UpperSaturationLimit','10','LowerSaturationLimit','-10');
    add_block('simulink/Continuous/PID Controller', [sys '/PIDz'], ...
        'Position',[200,145,280,175], 'P','2.0','I','0.3','D','1.0', ...
        'UpperSaturationLimit','5','LowerSaturationLimit','-5');

    % 加速度→姿态转换: phi=atan2(ay,g), theta=-atan2(ax,g)
    add_block('simulink/Math Operations/Trigonometric Function', [sys '/atan2_R'], ...
        'Position',[360,50,420,70], 'Operator','atan2');
    add_block('simulink/Math Operations/Trigonometric Function', [sys '/atan2_P'], ...
        'Position',[360,110,420,130], 'Operator','atan2');
    add_block('simulink/Math Operations/Gain', [sys '/Neg'], ...
        'Position',[460,110,500,130], 'Gain','-1');
    add_block('simulink/Sources/Constant', [sys '/g'], ...
        'Position',[300,180,330,200], 'Value','9.81');

    % 偏航=0
    add_block('simulink/Sources/Constant', [sys '/psi0'], ...
        'Position',[520,150,550,170], 'Value','0');

    % Mux att_des
    add_block('simulink/Signal Routing/Mux', [sys '/MuxA'], ...
        'Position',[600,50,605,175], 'Inputs','3');

    % 推力 = m*g (悬停)
    add_block('simulink/Sources/Constant', [sys '/T0'], ...
        'Position',[520,200,560,220], 'Value','14.715');

    % 连线
    add_line(sys, 'pos_des/1','D1/1');
    add_line(sys, 'pos_cur/1','D2/1');
    add_line(sys, 'D1/1','Sx/1'); add_line(sys, 'D2/1','Sx/2');
    add_line(sys, 'D1/2','Sy/1'); add_line(sys, 'D2/2','Sy/2');
    add_line(sys, 'D1/3','Sz/1'); add_line(sys, 'D2/3','Sz/2');
    add_line(sys, 'Sx/1','PIDx/1');
    add_line(sys, 'Sy/1','PIDy/1');
    add_line(sys, 'Sz/1','PIDz/1');
    add_line(sys, 'PIDy/1','atan2_R/1');  add_line(sys, 'g/1','atan2_R/2');
    add_line(sys, 'PIDx/1','atan2_P/1');  add_line(sys, 'g/1','atan2_P/2');
    add_line(sys, 'atan2_P/1','Neg/1');
    add_line(sys, 'atan2_R/1','MuxA/1');
    add_line(sys, 'Neg/1',     'MuxA/2');
    add_line(sys, 'psi0/1',    'MuxA/3');
    add_line(sys, 'MuxA/1','att_des/1');
    add_line(sys, 'T0/1','thrust/1');
end

%% ==================== 姿态环子系统 ====================
function build_att_ctrl_subsys(mdl)
    sys = [mdl '/Attitude_Ctrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/att_des'],   'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/thrust_in'], 'Position',[20,100,40,115]);
    add_block('simulink/Sources/In1', [sys '/att_cur'],   'Position',[20,200,40,215]);
    add_block('simulink/Sinks/Out1',  [sys '/torques'],   'Position',[780,80,800,95]);

    % Demux
    add_block('simulink/Signal Routing/Demux', [sys '/DD'], 'Position',[70,30,75,70], 'Outputs','3');
    add_block('simulink/Signal Routing/Demux', [sys '/DC'], 'Position',[70,180,75,250], 'Outputs','6');

    % 角度误差 Sum
    add_block('simulink/Math Operations/Sum', [sys '/Se1'], 'Position',[130,30,150,50], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Se2'], 'Position',[130,70,150,90], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Se3'], 'Position',[130,110,150,130], 'Inputs','+-');

    % P 增益 (角度)
    add_block('simulink/Math Operations/Gain', [sys '/Kp_R'], 'Position',[190,25,240,55], 'Gain','4.0');
    add_block('simulink/Math Operations/Gain', [sys '/Kp_P'], 'Position',[190,65,240,95], 'Gain','4.0');
    add_block('simulink/Math Operations/Gain', [sys '/Kp_Y'], 'Position',[190,105,240,135], 'Gain','3.0');

    % D 增益 (角速度阻尼)
    add_block('simulink/Math Operations/Gain', [sys '/Kd_R'], 'Position',[190,170,240,190], 'Gain','0.5');
    add_block('simulink/Math Operations/Gain', [sys '/Kd_P'], 'Position',[190,200,240,220], 'Gain','0.5');
    add_block('simulink/Math Operations/Gain', [sys '/Kd_Y'], 'Position',[190,230,240,250], 'Gain','0.8');

    % P - D 合成
    add_block('simulink/Math Operations/Sum', [sys '/Sd1'], 'Position',[300,30,320,50], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Sd2'], 'Position',[300,70,320,90], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Sd3'], 'Position',[300,110,320,130], 'Inputs','+-');

    % 饱和限幅
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat1'], ...
        'Position',[380,25,440,55], 'UpperLimit','5','LowerLimit','-5');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat2'], ...
        'Position',[380,65,440,95], 'UpperLimit','5','LowerLimit','-5');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat3'], ...
        'Position',[380,105,440,135], 'UpperLimit','2','LowerLimit','-2');

    % I 增益 (积分, 可选)
    add_block('simulink/Continuous/Integrator', [sys '/Int_R'], ...
        'Position',[480,25,520,45], 'InitialCondition','0');
    add_block('simulink/Continuous/Integrator', [sys '/Int_P'], ...
        'Position',[480,65,520,85], 'InitialCondition','0');
    add_block('simulink/Continuous/Integrator', [sys '/Int_Y'], ...
        'Position',[480,105,520,125], 'InitialCondition','0');
    add_block('simulink/Math Operations/Gain', [sys '/Ki_R'], 'Position',[550,25,590,45], 'Gain','0.3');
    add_block('simulink/Math Operations/Gain', [sys '/Ki_P'], 'Position',[550,65,590,85], 'Gain','0.3');
    add_block('simulink/Math Operations/Gain', [sys '/Ki_Y'], 'Position',[550,105,590,125], 'Gain','0.1');

    % P+D+I 合成
    add_block('simulink/Math Operations/Sum', [sys '/St1'], 'Position',[630,25,650,50], 'Inputs','+++');
    add_block('simulink/Math Operations/Sum', [sys '/St2'], 'Position',[630,65,650,90], 'Inputs','+++');
    add_block('simulink/Math Operations/Sum', [sys '/St3'], 'Position',[630,105,650,130], 'Inputs','+++');

    % Mux 力矩
    add_block('simulink/Signal Routing/Mux', [sys '/MuxT'], ...
        'Position',[720,30,725,130], 'Inputs','3');

    % 连线: 误差
    add_line(sys, 'att_des/1','DD/1');
    add_line(sys, 'att_cur/1','DC/1');
    add_line(sys, 'DD/1','Se1/1'); add_line(sys, 'DC/1','Se1/2');
    add_line(sys, 'DD/2','Se2/1'); add_line(sys, 'DC/2','Se2/2');
    add_line(sys, 'DD/3','Se3/1'); add_line(sys, 'DC/3','Se3/2');
    % P
    add_line(sys, 'Se1/1','Kp_R/1');
    add_line(sys, 'Se2/1','Kp_P/1');
    add_line(sys, 'Se3/1','Kp_Y/1');
    % D
    add_line(sys, 'DC/4','Kd_R/1');
    add_line(sys, 'DC/5','Kd_P/1');
    add_line(sys, 'DC/6','Kd_Y/1');
    % P-D
    add_line(sys, 'Kp_R/1','Sd1/1'); add_line(sys, 'Kd_R/1','Sd1/2');
    add_line(sys, 'Kp_P/1','Sd2/1'); add_line(sys, 'Kd_P/1','Sd2/2');
    add_line(sys, 'Kp_Y/1','Sd3/1'); add_line(sys, 'Kd_Y/1','Sd3/2');
    % 饱和
    add_line(sys, 'Sd1/1','Sat1/1');
    add_line(sys, 'Sd2/1','Sat2/1');
    add_line(sys, 'Sd3/1','Sat3/1');
    % I
    add_line(sys, 'Se1/1','Int_R/1'); add_line(sys, 'Int_R/1','Ki_R/1');
    add_line(sys, 'Se2/1','Int_P/1'); add_line(sys, 'Int_P/1','Ki_P/1');
    add_line(sys, 'Se3/1','Int_Y/1'); add_line(sys, 'Int_Y/1','Ki_Y/1');
    % P+D+I
    add_line(sys, 'Sat1/1','St1/1'); add_line(sys, 'Kd_R/1','St1/2'); add_line(sys, 'Ki_R/1','St1/3');
    add_line(sys, 'Sat2/1','St2/1'); add_line(sys, 'Kd_P/1','St2/2'); add_line(sys, 'Ki_P/1','St2/3');
    add_line(sys, 'Sat3/1','St3/1'); add_line(sys, 'Kd_Y/1','St3/2'); add_line(sys, 'Ki_Y/1','St3/3');
    % 输出
    add_line(sys, 'St1/1','MuxT/1');
    add_line(sys, 'St2/1','MuxT/2');
    add_line(sys, 'St3/1','MuxT/3');
    add_line(sys, 'MuxT/1','torques/1');
end

%% ==================== 混控器子系统 ====================
function build_mixer_subsys(mdl)
    sys = [mdl '/Mixer'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/thrust'],  'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/torques'], 'Position',[20,120,40,135]);
    add_block('simulink/Sinks/Out1',  [sys '/omega_sq'],'Position',[520,80,540,95]);

    % Demux torques
    add_block('simulink/Signal Routing/Demux', [sys '/DT'], 'Position',[70,110,75,155], 'Outputs','3');

    % 混控: 均分推力 + 力矩修正
    add_block('simulink/Math Operations/Gain', [sys '/T/4'], 'Position',[120,30,160,50], 'Gain','0.25');

    % 电机1: T/4 + tau_psi - tau_phi
    add_block('simulink/Math Operations/Sum', [sys '/S1'], 'Position',[220,20,240,45], 'Inputs','++-');
    % 电机2: T/4 - tau_psi + tau_phi
    add_block('simulink/Math Operations/Sum', [sys '/S2'], 'Position',[220,60,240,85], 'Inputs','+--');
    % 电机3: T/4 + tau_psi - tau_theta
    add_block('simulink/Math Operations/Sum', [sys '/S3'], 'Position',[220,100,240,125], 'Inputs','++-');
    % 电机4: T/4 - tau_psi + tau_theta
    add_block('simulink/Math Operations/Sum', [sys '/S4'], 'Position',[220,140,240,165], 'Inputs','+--');

    % 限幅 (转速平方>=0)
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat1'], ...
        'Position',[300,15,360,45], 'UpperLimit','1e6','LowerLimit','0');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat2'], ...
        'Position',[300,55,360,85], 'UpperLimit','1e6','LowerLimit','0');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat3'], ...
        'Position',[300,95,360,125], 'UpperLimit','1e6','LowerLimit','0');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat4'], ...
        'Position',[300,135,360,165], 'UpperLimit','1e6','LowerLimit','0');

    % Mux
    add_block('simulink/Signal Routing/Mux', [sys '/Mux'], ...
        'Position',[430,20,435,165], 'Inputs','4');

    % 连线
    add_line(sys, 'thrust/1','T/4/1');
    add_line(sys, 'torques/1','DT/1');
    add_line(sys, 'T/4/1','S1/1'); add_line(sys, 'DT/3','S1/2'); add_line(sys, 'DT/1','S1/3');
    add_line(sys, 'T/4/1','S2/1'); add_line(sys, 'DT/3','S2/2'); add_line(sys, 'DT/1','S2/3');
    add_line(sys, 'T/4/1','S3/1'); add_line(sys, 'DT/3','S3/2'); add_line(sys, 'DT/2','S3/3');
    add_line(sys, 'T/4/1','S4/1'); add_line(sys, 'DT/3','S4/2'); add_line(sys, 'DT/2','S4/3');
    add_line(sys, 'S1/1','Sat1/1'); add_line(sys, 'S2/1','Sat2/1');
    add_line(sys, 'S3/1','Sat3/1'); add_line(sys, 'S4/1','Sat4/1');
    add_line(sys, 'Sat1/1','Mux/1'); add_line(sys, 'Sat2/1','Mux/2');
    add_line(sys, 'Sat3/1','Mux/3'); add_line(sys, 'Sat4/1','Mux/4');
    add_line(sys, 'Mux/1','omega_sq/1');
end

%% ==================== 电机子系统 ====================
function build_motor_subsys(mdl)
    sys = [mdl '/Motors'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/omega_sq'], 'Position',[20,70,40,85]);
    add_block('simulink/Sinks/Out1',  [sys '/omega'],    'Position',[480,70,500,85]);

    add_block('simulink/Signal Routing/Demux', [sys '/D'], 'Position',[80,50,85,110], 'Outputs','4');

    % 一阶惯性: tau=0.02s
    add_block('simulink/Continuous/Transfer Fcn', [sys '/M1'], 'Position',[160,25,260,50], 'Numerator','[1]','Denominator','[0.02 1]');
    add_block('simulink/Continuous/Transfer Fcn', [sys '/M2'], 'Position',[160,60,260,85], 'Numerator','[1]','Denominator','[0.02 1]');
    add_block('simulink/Continuous/Transfer Fcn', [sys '/M3'], 'Position',[160,95,260,120],'Numerator','[1]','Denominator','[0.02 1]');
    add_block('simulink/Continuous/Transfer Fcn', [sys '/M4'], 'Position',[160,130,260,155],'Numerator','[1]','Denominator','[0.02 1]');

    % sqrt
    add_block('simulink/Math Operations/Math Function', [sys '/Sq1'], 'Position',[310,25,350,50], 'Operator','sqrt');
    add_block('simulink/Math Operations/Math Function', [sys '/Sq2'], 'Position',[310,60,350,85], 'Operator','sqrt');
    add_block('simulink/Math Operations/Math Function', [sys '/Sq3'], 'Position',[310,95,350,120],'Operator','sqrt');
    add_block('simulink/Math Operations/Math Function', [sys '/Sq4'], 'Position',[310,130,350,155],'Operator','sqrt');

    add_block('simulink/Signal Routing/Mux', [sys '/M'], 'Position',[410,30,415,150], 'Inputs','4');

    add_line(sys, 'omega_sq/1','D/1');
    add_line(sys, 'D/1','M1/1'); add_line(sys, 'D/2','M2/1'); add_line(sys, 'D/3','M3/1'); add_line(sys, 'D/4','M4/1');
    add_line(sys, 'M1/1','Sq1/1'); add_line(sys, 'M2/1','Sq2/1'); add_line(sys, 'M3/1','Sq3/1'); add_line(sys, 'M4/1','Sq4/1');
    add_line(sys, 'Sq1/1','M/1'); add_line(sys, 'Sq2/1','M/2'); add_line(sys, 'Sq3/1','M/3'); add_line(sys, 'Sq4/1','M/4');
    add_line(sys, 'M/1','omega/1');
end

%% ==================== 动学子系统 ====================
function build_dynamics_subsys(mdl)
    sys = [mdl '/Dynamics_6DOF'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/omega'], 'Position',[20,80,40,95]);
    add_block('simulink/Sinks/Out1',  [sys '/state'], 'Position',[580,80,600,95]);

    % MATLAB Function block 实现 6DOF
    add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/dyn6dof'], ...
        'Position',[200,50,420,130]);

    % 积分器
    add_block('simulink/Continuous/Integrator', [sys '/Int'], ...
        'Position',[470,65,520,115], 'InitialCondition','[0;0;0;0;0;0;0;0;0;0;0;0]');

    % 连线
    add_line(sys, 'omega/1','dyn6dof/1');
    add_line(sys, 'dyn6dof/1','Int/1');
    add_line(sys, 'Int/1','dyn6dof/2');
    add_line(sys, 'Int/1','state/1');
end

%% ==================== 传感器子系统 ====================
function build_sensor_subsys(mdl)
    sys = [mdl '/Sensors'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/state_true'], 'Position',[20,50,40,65]);
    add_block('simulink/Sinks/Out1',  [sys '/state_meas'], 'Position',[250,50,270,65]);

    % 噪声
    add_block('simulink/Sources/Uniform Random Number', [sys '/noise'], ...
        'Position',[80,90,140,110], 'Minimum','-0.05','Maximum','0.05');
    add_block('simulink/Math Operations/Sum', [sys '/S'], 'Position',[170,50,190,70], 'Inputs','++');

    add_line(sys, 'state_true/1','S/1');
    add_line(sys, 'noise/1','S/2');
    add_line(sys, 'S/1','state_meas/1');
end
