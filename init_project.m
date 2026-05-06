function init_project()
% INIT_PROJECT  初始化 drone-simulink 项目
%   添加所有子目录到 MATLAB 路径，加载基础参数
%
%   用法:
%     cd E:\drone-simulink
%     init_project

    root = fileparts(mfilename('fullpath'));

    % 添加子目录
    addpath(root);
    addpath(fullfile(root, 'params'));
    addpath(fullfile(root, 'core'));
    addpath(fullfile(root, 'control'));
    addpath(fullfile(root, 'utils'));
    addpath(fullfile(root, 'sim'));
    addpath(fullfile(root, 'examples'));
    addpath(fullfile(root, 'tests'));

    % 加载基础参数
    quad_params;
    motor_params;
    controller_params;
    sensor_params;
    sim_params;

    fprintf('drone-simulink 项目已初始化\n');
    fprintf('  项目根目录: %s\n', root);
    fprintf('  运行 quad_hover 开始悬停仿真\n');
end
