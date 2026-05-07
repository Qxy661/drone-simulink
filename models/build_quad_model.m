function build_quad_model()
%BUILD_QUAD_MODEL  构建四旋翼级联PID Simulink模型 (可直接运行)
%   生成 quad_cascade_pid.slx
%   控制框图: 轨迹→位置PID→姿态PD→混控→电机动力学→6DOF→反馈
%
%   所有动力学用 MATLAB Function block 实现, 控制器用标准 Simulink 块
%
%   用法:
%     cd E:\drone-simulink; init_project; build_quad_model
%     open_system('quad_cascade_pid')
%     sim('quad_cascade_pid')

    mdl = 'quad_cascade_pid';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    set_param(mdl, 'Solver','ode4','FixedStep','0.001','StopTime','20');
    set_param(mdl, 'SaveFormat','StructureWithTime','StopTime','20');

    %% ========== 轨迹输入 ==========
    add_const(mdl, 'X_des', 5,   [30,100,65,120]);
    add_const(mdl, 'Y_des', 3,   [30,160,65,180]);
    add_const(mdl, 'Z_des', 10,  [30,220,65,240]);
    add_mux(mdl, 'Mux_Des', 3, [100,110,105,235]);

    %% ========== 位置环子系统 ==========
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/PosCtrl'], ...
        'Position',[200,100,370,240], 'BackgroundColor','cyan');
    build_pos_ctrl(mdl);

    %% ========== 姿态环子系统 ==========
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/AttCtrl'], ...
        'Position',[430,80,600,260], 'BackgroundColor','green');
    build_att_ctrl(mdl);

    %% ========== 混控器子系统 ==========
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Mixer'], ...
        'Position',[660,100,820,240], 'BackgroundColor','yellow');
    build_mixer(mdl);

    %% ========== 电机动力学子系统 ==========
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Motors'], ...
        'Position',[880,100,1040,240], 'BackgroundColor','magenta');
    build_motors(mdl);

    %% ========== 6DOF 动力学子系统 ==========
    add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Dynamics6DOF'], ...
        'Position',[1100,80,1300,260], 'BackgroundColor','red');
    build_dynamics_6dof(mdl);

    %% ========== 反馈 Mux ==========
    add_mux(mdl, 'Mux_FbPos', 3, [140,350,145,430]);
    add_mux(mdl, 'Mux_FbAtt', 6, [370,350,375,480]);

    %% ========== Scope ==========
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Pos'], ...
        'Position',[1400,50,1440,90], 'NumInputPorts','2');
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Att'], ...
        'Position',[1400,120,1440,160], 'NumInputPorts','2');
    add_block('simulink/Sinks/Scope', [mdl '/Scope_Motor'], ...
        'Position',[1400,190,1440,220]);
    add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Pos'], ...
        'VariableName','pos_log','Position',[1400,250,1440,280]);
    add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Att'], ...
        'VariableName','att_log','Position',[1400,300,1440,330]);

    %% ========== 顶层连线 ==========
    add_line(mdl, 'X_des/1', 'Mux_Des/1');
    add_line(mdl, 'Y_des/1', 'Mux_Des/2');
    add_line(mdl, 'Z_des/1', 'Mux_Des/3');
    add_line(mdl, 'Mux_Des/1',  'PosCtrl/1');
    add_line(mdl, 'PosCtrl/1',  'AttCtrl/1');   % att_des
    add_line(mdl, 'PosCtrl/2',  'AttCtrl/2');   % thrust
    add_line(mdl, 'PosCtrl/2',  'Mixer/1');     % thrust
    add_line(mdl, 'AttCtrl/1',  'Mixer/2');     % torques
    add_line(mdl, 'Mixer/1',    'Motors/1');    % omega_sq
    add_line(mdl, 'Motors/1',   'Dynamics6DOF/1'); % omega

    % 状态反馈: Dynamics6DOF → Demux → Mux → 控制器
    % Dynamics6DOF 输出 12 维状态
    add_line(mdl, 'Dynamics6DOF/1', 'Mux_FbPos/1');  % x
    add_line(mdl, 'Dynamics6DOF/1', 'Mux_FbPos/2', 'autoroute','on');  % y
    add_line(mdl, 'Dynamics6DOF/1', 'Mux_FbPos/3', 'autoroute','on');  % z
    % 需要 Demux 来分离状态分量
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux_State'], ...
        'Position',[1340,100,1345,250], 'Outputs','12');
    add_line(mdl, 'Dynamics6DOF/1', 'Demux_State/1');

    % 位置反馈
    add_line(mdl, 'Demux_State/1', 'Mux_FbPos/1');
    add_line(mdl, 'Demux_State/2', 'Mux_FbPos/2');
    add_line(mdl, 'Demux_State/3', 'Mux_FbPos/3');
    add_line(mdl, 'Mux_FbPos/1',  'PosCtrl/2');

    % 姿态+角速度反馈
    add_line(mdl, 'Demux_State/7',  'Mux_FbAtt/1');
    add_line(mdl, 'Demux_State/8',  'Mux_FbAtt/2');
    add_line(mdl, 'Demux_State/9',  'Mux_FbAtt/3');
    add_line(mdl, 'Demux_State/10', 'Mux_FbAtt/4');
    add_line(mdl, 'Demux_State/11', 'Mux_FbAtt/5');
    add_line(mdl, 'Demux_State/12', 'Mux_FbAtt/6');
    add_line(mdl, 'Mux_FbAtt/1',   'AttCtrl/3');

    % Scope
    add_line(mdl, 'Mux_Des/1',     'Scope_Pos/1');
    add_line(mdl, 'Mux_FbPos/1',   'Scope_Pos/2');
    add_line(mdl, 'Mux_FbAtt/1',   'Scope_Att/1');
    add_line(mdl, 'Mux_FbAtt/2',   'Scope_Att/2');
    add_line(mdl, 'Mixer/1',       'Scope_Motor/1');
    add_line(mdl, 'Demux_State/1', 'ToWS_Pos/1');
    add_line(mdl, 'Demux_State/7', 'ToWS_Att/1');

    %% 保存
    save_system(mdl, fullfile(pwd, [mdl '.slx']));
    fprintf('已生成: %s.slx — 打开后可直接运行\n', mdl);
end

%% ==================== 位置环子系统 ====================
function build_pos_ctrl(mdl)
    sys = [mdl '/PosCtrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1',     [sys '/pos_des'], 'Position',[20,60,40,75]);
    add_block('simulink/Sources/In1',     [sys '/pos_cur'], 'Position',[20,160,40,175]);
    add_block('simulink/Sinks/Out1',      [sys '/att_des'], 'Position',[620,60,640,75]);
    add_block('simulink/Sinks/Out1',      [sys '/thrust'],  'Position',[620,160,640,175]);

    % Demux
    add_block('simulink/Signal Routing/Demux', [sys '/D1'], 'Position',[70,45,75,100], 'Outputs','3');
    add_block('simulink/Signal Routing/Demux', [sys '/D2'], 'Position',[70,145,75,200], 'Outputs','3');

    % 误差 Sum
    add_block('simulink/Math Operations/Sum', [sys '/Sx'], 'Position',[130,50,150,70], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Sy'], 'Position',[130,100,150,120], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Sz'], 'Position',[130,150,150,170], 'Inputs','+-');

    % PID 控制器 (标准 Simulink PID 块)
    add_block('simulink/Continuous/PID Controller', [sys '/PIDx'], ...
        'Position',[190,45,270,75], 'P','1.5','I','0.2','D','0.8', ...
        'UpperSaturationLimit','10','LowerSaturationLimit','-10');
    add_block('simulink/Continuous/PID Controller', [sys '/PIDy'], ...
        'Position',[190,95,270,125], 'P','1.5','I','0.2','D','0.8', ...
        'UpperSaturationLimit','10','LowerSaturationLimit','-10');
    add_block('simulink/Continuous/PID Controller', [sys '/PIDz'], ...
        'Position',[190,145,270,175], 'P','2.0','I','0.3','D','1.0', ...
        'UpperSaturationLimit','5','LowerSaturationLimit','-5');

    % 加速度→期望姿态: phi_d=atan2(ay,g), theta_d=-atan2(ax,g)
    add_block('simulink/Math Operations/Trigonometric Function', [sys '/atan2_R'], ...
        'Position',[330,50,390,70], 'Operator','atan2');
    add_block('simulink/Math Operations/Trigonometric Function', [sys '/atan2_P'], ...
        'Position',[330,110,390,130], 'Operator','atan2');
    add_block('simulink/Math Operations/Gain', [sys '/Neg'], ...
        'Position',[430,110,470,130], 'Gain','-1');
    add_block('simulink/Sources/Constant', [sys '/g'], ...
        'Position',[280,180,310,200], 'Value','9.81');
    add_block('simulink/Sources/Constant', [sys '/psi0'], ...
        'Position',[480,150,510,170], 'Value','0');
    add_block('simulink/Signal Routing/Mux', [sys '/MuxA'], ...
        'Position',[540,50,545,175], 'Inputs','3');
    add_block('simulink/Sources/Constant', [sys '/T0'], ...
        'Position',[480,200,520,220], 'Value','14.715');

    % 连线
    add_line(sys, 'pos_des/1','D1/1'); add_line(sys, 'pos_cur/1','D2/1');
    add_line(sys, 'D1/1','Sx/1'); add_line(sys, 'D2/1','Sx/2');
    add_line(sys, 'D1/2','Sy/1'); add_line(sys, 'D2/2','Sy/2');
    add_line(sys, 'D1/3','Sz/1'); add_line(sys, 'D2/3','Sz/2');
    add_line(sys, 'Sx/1','PIDx/1'); add_line(sys, 'Sy/1','PIDy/1'); add_line(sys, 'Sz/1','PIDz/1');
    add_line(sys, 'PIDy/1','atan2_R/1'); add_line(sys, 'g/1','atan2_R/2');
    add_line(sys, 'PIDx/1','atan2_P/1'); add_line(sys, 'g/1','atan2_P/2');
    add_line(sys, 'atan2_P/1','Neg/1');
    add_line(sys, 'atan2_R/1','MuxA/1'); add_line(sys, 'Neg/1','MuxA/2'); add_line(sys, 'psi0/1','MuxA/3');
    add_line(sys, 'MuxA/1','att_des/1');
    add_line(sys, 'T0/1','thrust/1');
end

%% ==================== 姿态环子系统 ====================
function build_att_ctrl(mdl)
    sys = [mdl '/AttCtrl'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/att_des'],  'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/thr_in'],   'Position',[20,100,40,115]);
    add_block('simulink/Sources/In1', [sys '/att_cur'],   'Position',[20,200,40,215]);
    add_block('simulink/Sinks/Out1',  [sys '/torques'],  'Position',[750,80,770,95]);

    % Demux
    add_block('simulink/Signal Routing/Demux', [sys '/DD'], 'Position',[70,30,75,70], 'Outputs','3');
    add_block('simulink/Signal Routing/Demux', [sys '/DC'], 'Position',[70,180,75,260], 'Outputs','6');

    % 角度误差
    add_block('simulink/Math Operations/Sum', [sys '/Se1'], 'Position',[120,30,140,50], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Se2'], 'Position',[120,70,140,90], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Se3'], 'Position',[120,110,140,130], 'Inputs','+-');

    % P 增益
    add_block('simulink/Math Operations/Gain', [sys '/Kp1'], 'Position',[170,25,220,55], 'Gain','4.0');
    add_block('simulink/Math Operations/Gain', [sys '/Kp2'], 'Position',[170,65,220,95], 'Gain','4.0');
    add_block('simulink/Math Operations/Gain', [sys '/Kp3'], 'Position',[170,105,220,135], 'Gain','3.0');

    % D 增益 (角速度阻尼)
    add_block('simulink/Math Operations/Gain', [sys '/Kd1'], 'Position',[170,170,220,190], 'Gain','0.5');
    add_block('simulink/Math Operations/Gain', [sys '/Kd2'], 'Position',[170,200,220,220], 'Gain','0.5');
    add_block('simulink/Math Operations/Gain', [sys '/Kd3'], 'Position',[170,230,220,250], 'Gain','0.8');

    % P - D
    add_block('simulink/Math Operations/Sum', [sys '/Sd1'], 'Position',[270,30,290,50], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Sd2'], 'Position',[270,70,290,90], 'Inputs','+-');
    add_block('simulink/Math Operations/Sum', [sys '/Sd3'], 'Position',[270,110,290,130], 'Inputs','+-');

    % 饱和
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat1'], ...
        'Position',[340,25,400,55], 'UpperLimit','5','LowerLimit','-5');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat2'], ...
        'Position',[340,65,400,95], 'UpperLimit','5','LowerLimit','-5');
    add_block('simulink/Discontinuities/Saturation', [sys '/Sat3'], ...
        'Position',[340,105,400,135], 'UpperLimit','2','LowerLimit','-2');

    % I 增益
    add_block('simulink/Continuous/Integrator', [sys '/Int1'], 'Position',[440,25,480,45]);
    add_block('simulink/Continuous/Integrator', [sys '/Int2'], 'Position',[440,65,480,85]);
    add_block('simulink/Continuous/Integrator', [sys '/Int3'], 'Position',[440,105,480,125]);
    add_block('simulink/Math Operations/Gain', [sys '/Ki1'], 'Position',[510,25,550,45], 'Gain','0.3');
    add_block('simulink/Math Operations/Gain', [sys '/Ki2'], 'Position',[510,65,550,85], 'Gain','0.3');
    add_block('simulink/Math Operations/Gain', [sys '/Ki3'], 'Position',[510,105,550,125], 'Gain','0.1');

    % P+D+I
    add_block('simulink/Math Operations/Sum', [sys '/St1'], 'Position',[590,25,610,50], 'Inputs','+++');
    add_block('simulink/Math Operations/Sum', [sys '/St2'], 'Position',[590,65,610,90], 'Inputs','+++');
    add_block('simulink/Math Operations/Sum', [sys '/St3'], 'Position',[590,105,610,130], 'Inputs','+++');

    add_block('simulink/Signal Routing/Mux', [sys '/MuxT'], 'Position',[680,30,685,130], 'Inputs','3');

    % 连线
    add_line(sys, 'att_des/1','DD/1'); add_line(sys, 'att_cur/1','DC/1');
    add_line(sys, 'DD/1','Se1/1'); add_line(sys, 'DC/1','Se1/2');
    add_line(sys, 'DD/2','Se2/1'); add_line(sys, 'DC/2','Se2/2');
    add_line(sys, 'DD/3','Se3/1'); add_line(sys, 'DC/3','Se3/2');
    add_line(sys, 'Se1/1','Kp1/1'); add_line(sys, 'Se2/1','Kp2/1'); add_line(sys, 'Se3/1','Kp3/1');
    add_line(sys, 'DC/4','Kd1/1'); add_line(sys, 'DC/5','Kd2/1'); add_line(sys, 'DC/6','Kd3/1');
    add_line(sys, 'Kp1/1','Sd1/1'); add_line(sys, 'Kd1/1','Sd1/2');
    add_line(sys, 'Kp2/1','Sd2/1'); add_line(sys, 'Kd2/1','Sd2/2');
    add_line(sys, 'Kp3/1','Sd3/1'); add_line(sys, 'Kd3/1','Sd3/2');
    add_line(sys, 'Sd1/1','Sat1/1'); add_line(sys, 'Sd2/1','Sat2/1'); add_line(sys, 'Sd3/1','Sat3/1');
    add_line(sys, 'Se1/1','Int1/1'); add_line(sys, 'Int1/1','Ki1/1');
    add_line(sys, 'Se2/1','Int2/1'); add_line(sys, 'Int2/1','Ki2/1');
    add_line(sys, 'Se3/1','Int3/1'); add_line(sys, 'Int3/1','Ki3/1');
    add_line(sys, 'Sat1/1','St1/1'); add_line(sys, 'Kd1/1','St1/2'); add_line(sys, 'Ki1/1','St1/3');
    add_line(sys, 'Sat2/1','St2/1'); add_line(sys, 'Kd2/1','St2/2'); add_line(sys, 'Ki2/1','St2/3');
    add_line(sys, 'Sat3/1','St3/1'); add_line(sys, 'Kd3/1','St3/2'); add_line(sys, 'Ki3/1','St3/3');
    add_line(sys, 'St1/1','MuxT/1'); add_line(sys, 'St2/1','MuxT/2'); add_line(sys, 'St3/1','MuxT/3');
    add_line(sys, 'MuxT/1','torques/1');
end

%% ==================== 混控器子系统 ====================
function build_mixer(mdl)
    sys = [mdl '/Mixer'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/thrust'],  'Position',[20,40,40,55]);
    add_block('simulink/Sources/In1', [sys '/torques'], 'Position',[20,120,40,135]);
    add_block('simulink/Sinks/Out1',  [sys '/omega_sq'],'Position',[500,80,520,95]);

    add_block('simulink/Signal Routing/Demux', [sys '/DT'], 'Position',[70,110,75,155], 'Outputs','3');

    % T/4
    add_block('simulink/Math Operations/Gain', [sys '/T4'], 'Position',[110,30,150,50], 'Gain','0.25');

    % 混控: X型布局
    % M1: T/4 + tau_psi - tau_phi
    add_block('simulink/Math Operations/Sum', [sys '/S1'], 'Position',[200,15,220,45], 'Inputs','++-');
    % M2: T/4 - tau_psi + tau_phi
    add_block('simulink/Math Operations/Sum', [sys '/S2'], 'Position',[200,55,220,85], 'Inputs','+--');
    % M3: T/4 + tau_psi - tau_theta
    add_block('simulink/Math Operations/Sum', [sys '/S3'], 'Position',[200,95,220,125], 'Inputs','++-');
    % M4: T/4 - tau_psi + tau_theta
    add_block('simulink/Math Operations/Sum', [sys '/S4'], 'Position',[200,135,220,165], 'Inputs','+--');

    % 限幅
    add_block('simulink/Discontinuities/Saturation', [sys '/L1'], 'Position',[270,10,330,45], 'UpperLimit','1.5e6','LowerLimit','0');
    add_block('simulink/Discontinuities/Saturation', [sys '/L2'], 'Position',[270,50,330,85], 'UpperLimit','1.5e6','LowerLimit','0');
    add_block('simulink/Discontinuities/Saturation', [sys '/L3'], 'Position',[270,90,330,125],'UpperLimit','1.5e6','LowerLimit','0');
    add_block('simulink/Discontinuities/Saturation', [sys '/L4'], 'Position',[270,130,330,165],'UpperLimit','1.5e6','LowerLimit','0');

    add_block('simulink/Signal Routing/Mux', [sys '/Mux'], 'Position',[400,15,405,165], 'Inputs','4');

    add_line(sys, 'thrust/1','T4/1');
    add_line(sys, 'torques/1','DT/1');
    add_line(sys, 'T4/1','S1/1'); add_line(sys, 'DT/3','S1/2'); add_line(sys, 'DT/1','S1/3');
    add_line(sys, 'T4/1','S2/1'); add_line(sys, 'DT/3','S2/2'); add_line(sys, 'DT/1','S2/3');
    add_line(sys, 'T4/1','S3/1'); add_line(sys, 'DT/3','S3/2'); add_line(sys, 'DT/2','S3/3');
    add_line(sys, 'T4/1','S4/1'); add_line(sys, 'DT/3','S4/2'); add_line(sys, 'DT/2','S4/3');
    add_line(sys, 'S1/1','L1/1'); add_line(sys, 'S2/1','L2/1'); add_line(sys, 'S3/1','L3/1'); add_line(sys, 'S4/1','L4/1');
    add_line(sys, 'L1/1','Mux/1'); add_line(sys, 'L2/1','Mux/2'); add_line(sys, 'L3/1','Mux/3'); add_line(sys, 'L4/1','Mux/4');
    add_line(sys, 'Mux/1','omega_sq/1');
end

%% ==================== 电机动力学子系统 ====================
function build_motors(mdl)
    sys = [mdl '/Motors'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/omega_sq'], 'Position',[20,70,40,85]);
    add_block('simulink/Sinks/Out1',  [sys '/omega'],    'Position',[460,70,480,85]);

    add_block('simulink/Signal Routing/Demux', [sys '/D'], 'Position',[70,50,75,110], 'Outputs','4');

    % 一阶惯性传递函数: 1/(tau*s+1), tau=0.02s
    for i = 1:4
        name = sprintf('/M%d', i);
        y = 15 + (i-1)*35;
        add_block('simulink/Continuous/Transfer Fcn', [sys name], ...
            'Position',[140,y,250,y+25], 'Numerator','[1]','Denominator','[0.02 1]');
    end

    % sqrt: omega = sqrt(omega_sq)
    for i = 1:4
        name = sprintf('/Sq%d', i);
        y = 15 + (i-1)*35;
        add_block('simulink/Math Operations/Math Function', [sys name], ...
            'Position',[300,y,340,y+25], 'Operator','sqrt');
    end

    add_block('simulink/Signal Routing/Mux', [sys '/Mux'], 'Position',[390,20,395,150], 'Inputs','4');

    add_line(sys, 'omega_sq/1','D/1');
    for i = 1:4
        add_line(sys, sprintf('D/%d',i), sprintf('M%d/1',i));
        add_line(sys, sprintf('M%d/1',i), sprintf('Sq%d/1',i));
        add_line(sys, sprintf('Sq%d/1',i), sprintf('Mux/%d',i));
    end
    add_line(sys, 'Mux/1','omega/1');
end

%% ==================== 6DOF 动力学子系统 ====================
function build_dynamics_6dof(mdl)
    sys = [mdl '/Dynamics6DOF'];
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']); delete_block([sys '/Out1']);

    add_block('simulink/Sources/In1', [sys '/omega'], 'Position',[20,80,40,95]);
    add_block('simulink/Sinks/Out1',  [sys '/state'], 'Position',[480,80,500,95]);

    % MATLAB Function block — 内含完整 6DOF 代码
    add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/dyn6dof'], ...
        'Position',[120,50,350,130]);

    % 设置 MATLAB Function 代码
    set_mafcn_code(sys, 'dyn6dof');

    % 积分器: state_dot → state
    add_block('simulink/Continuous/Integrator', [sys '/Int'], ...
        'Position',[380,65,430,115], 'InitialCondition','[0;0;0;0;0;0;0;0;0;0;0;0]');

    add_line(sys, 'omega/1','dyn6dof/1');
    add_line(sys, 'Int/1','dyn6dof/2');
    add_line(sys, 'dyn6dof/1','Int/1');
    add_line(sys, 'Int/1','state/1');
end

%% ==================== 设置 MATLAB Function 代码 ====================
function set_mafcn_code(sys, blk_name)
%SET_MAFCN_CODE  给 MATLAB Function block 写入动力学代码
    blk_path = [sys '/' blk_name];

    % 获取 Stateflow root
    rt = sfroot;

    % 找到 MATLAB Function block 的 chart
    % 方法: 通过 Block 类型查找
    all_blocks = rt.find('-isa','Stateflow.EMChart');
    target = [];
    for i = 1:numel(all_blocks)
        if contains(all_blocks(i).Path, blk_path)
            target = all_blocks(i);
            break;
        end
    end

    if isempty(target)
        % 备用: 直接通过 Path 查找
        try
            target = get_param(blk_path, 'SFChartObject');
        catch
            warning('无法设置 MATLAB Function 代码, 请手动编辑 %s', blk_path);
            return;
        end
    end

    % 写入动力学函数代码
    code = [
        'function state_dot = fcn(omega_cmd, state)' newline ...
        '%#codegen' newline ...
        '%% 四旋翼 6DOF 动力学' newline ...
        'm = 1.5; g = 9.81;' newline ...
        'kt = 1.5e-5; kd = 2.0e-7; l = 0.225;' newline ...
        'Ixx = 0.0035; Iyy = 0.0035; Izz = 0.0065;' newline ...
        '' newline ...
        'x=state(1); y=state(2); z=state(3);' newline ...
        'vx=state(4); vy=state(5); vz=state(6);' newline ...
        'phi=state(7); theta=state(8); psi=state(9);' newline ...
        'p=state(10); q=state(11); r=state(12);' newline ...
        '' newline ...
        'w1=omega_cmd(1); w2=omega_cmd(2);' newline ...
        'w3=omega_cmd(3); w4=omega_cmd(4);' newline ...
        '' newline ...
        'T = kt*(w1^2+w2^2+w3^2+w4^2);' newline ...
        'tau_phi = l*kt*(w2^2-w4^2);' newline ...
        'tau_theta = l*kt*(w3^2-w1^2);' newline ...
        'tau_psi = kd*(w1^2-w2^2+w3^2-w4^2);' newline ...
        '' newline ...
        'cp=cos(phi); sp=sin(phi);' newline ...
        'ct=cos(theta); st=sin(theta);' newline ...
        'cb=cos(psi); sb=sin(psi);' newline ...
        '' newline ...
        'ax = (T/m)*(cb*st*cp + sb*sp);' newline ...
        'ay = (T/m)*(sb*st*cp - cb*sp);' newline ...
        'az = (T/m)*(ct*cp) - g;' newline ...
        '' newline ...
        'p_dot = (tau_phi + (Iyy-Izz)*q*r) / Ixx;' newline ...
        'q_dot = (tau_theta + (Izz-Ixx)*p*r) / Iyy;' newline ...
        'r_dot = (tau_psi + (Ixx-Iyy)*p*q) / Izz;' newline ...
        '' newline ...
        'phi_dot = p + q*sp*(st/ct) + r*cp*(st/ct);' newline ...
        'theta_dot = q*cp - r*sp;' newline ...
        'psi_dot = q*sp/ct + r*cp/ct;' newline ...
        '' newline ...
        'state_dot = [vx;vy;vz; ax;ay;az;' newline ...
        '             phi_dot;theta_dot;psi_dot;' newline ...
        '             p_dot;q_dot;r_dot];' newline ...
        'end' newline
    ];

    try
        target.Script = code;
    catch
        % 如果 Script 属性不可用, 尝试通过 children
        try
            eml = target.find('-isa','Stateflow.EMFunction');
            if ~isempty(eml)
                eml(1).Script = code;
            end
        catch
            warning('自动设置代码失败, 请手动编辑 %s 中的 MATLAB Function', blk_path);
        end
    end
end

%% ==================== 工具函数 ====================
function add_const(mdl, name, val, pos)
    add_block('simulink/Sources/Constant', [mdl '/' name], ...
        'Position', pos, 'Value', num2str(val));
end

function add_mux(mdl, name, n, pos)
    add_block('simulink/Signal Routing/Mux', [mdl '/' name], ...
        'Position', pos, 'Inputs', num2str(n));
end
