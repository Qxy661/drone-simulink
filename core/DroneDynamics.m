classdef DroneDynamics < handle
%DRONEDYNAMICS  动力学基类
%   所有飞行器动力学的公共接口
%   子类: QuadcopterDynamics, FixedWingDynamics

    properties
        state       % 状态向量 [12x1]
        params      % 物理参数结构体
    end

    methods
        function obj = DroneDynamics(params)
            obj.params = params;
            obj.state = zeros(12, 1);
        end

        function reset(obj, pos, vel, att, rates)
        %RESET  重置状态
            obj.state(1:3) = pos(:);
            obj.state(4:6) = vel(:);
            obj.state(7:9) = att(:);
            obj.state(10:12) = rates(:);
        end

        function state_dot = dynamics(obj, t, state, forces, moments)
        %DYNAMICS  状态微分方程 (需子类实现)
        %   输入:
        %     t       - 时间
        %     state   - 12 维状态
        %     forces  - 外力 [Fx; Fy; Fz] (机体系)
        %     moments - 外力矩 [Mx; My; Mz] (机体系)
        %   输出:
        %     state_dot - 状态导数
            error('子类必须实现 dynamics 方法');
        end
    end

    methods (Static)
        function R = euler_to_rotation(phi, theta, psi)
        %EULER_TO_ROTATION  ZYX 欧拉角 -> 旋转矩阵 (body -> world)
            R = euler_to_rotation(phi, theta, psi);
        end
    end
end
