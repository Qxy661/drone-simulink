classdef TrajectoryGenerator < handle
%TRAJECTORYGENERATOR  轨迹生成器
%   支持: 悬停、航点、圆形轨迹

    properties
        mode            % 'hover', 'waypoint', 'circle'
        target_pos      % 目标位置 [3x1]
        waypoints       % 航点列表 [Nx3]
        current_wp      % 当前航点索引
        wp_radius       % 航点到达半径 [m]
        circle_center   % 圆心 [3x1]
        circle_radius   % 圆半径 [m]
        circle_omega    % 角速度 [rad/s]
        t_start         % 起始时间
    end

    methods
        function obj = TrajectoryGenerator()
            obj.mode = 'hover';
            obj.target_pos = zeros(3, 1);
            obj.waypoints = [];
            obj.current_wp = 1;
            obj.wp_radius = 0.3;
            obj.t_start = 0;
        end

        function set_hover(obj, pos)
        %SET_HOVER  设置悬停目标
            obj.mode = 'hover';
            obj.target_pos = pos(:);
        end

        function set_waypoints(obj, wps, radius)
        %SET_WAYPOINTS  设置航点列表
        %   wps    - Nx3 航点矩阵
        %   radius - 到达判定半径 [m]
            obj.mode = 'waypoint';
            obj.waypoints = wps;
            obj.current_wp = 1;
            obj.target_pos = wps(1, :)';
            if nargin >= 3
                obj.wp_radius = radius;
            end
        end

        function set_circle(obj, center, radius, period)
        %SET_CIRCLE  设置圆形轨迹
        %   center - 圆心 [3x1]
        %   radius - 半径 [m]
        %   period - 周期 [s]
            obj.mode = 'circle';
            obj.circle_center = center(:);
            obj.circle_radius = radius;
            obj.circle_omega = 2 * pi / period;
            obj.t_start = [];
        end

        function [pos_des, vel_des] = generate(obj, t)
        %GENERATE  生成期望位置和速度
        %   输入:
        %     t - 当前时间 [s]
        %   输出:
        %     pos_des - 3x1 期望位置
        %     vel_des - 3x1 期望速度

            switch obj.mode
                case 'hover'
                    pos_des = obj.target_pos;
                    vel_des = zeros(3, 1);

                case 'waypoint'
                    pos_des = obj.target_pos;
                    vel_des = zeros(3, 1);

                case 'circle'
                    if isempty(obj.t_start)
                        obj.t_start = t;
                    end
                    dt = t - obj.t_start;
                    angle = obj.circle_omega * dt;

                    pos_des = obj.circle_center + obj.circle_radius * ...
                        [cos(angle); sin(angle); 0];
                    vel_des = obj.circle_radius * obj.circle_omega * ...
                        [-sin(angle); cos(angle); 0];
            end
        end

        function reached = check_arrival(obj, current_pos)
        %CHECK_ARRIVAL  检查是否到达当前航点
            if ~strcmp(obj.mode, 'waypoint') || isempty(obj.waypoints)
                reached = false;
                return;
            end

            dist = norm(current_pos - obj.target_pos);
            if dist < obj.wp_radius
                if obj.current_wp < size(obj.waypoints, 1)
                    obj.current_wp = obj.current_wp + 1;
                    obj.target_pos = obj.waypoints(obj.current_wp, :)';
                    reached = false;
                else
                    reached = true;  % 最后一个航点
                end
            else
                reached = false;
            end
        end
    end
end
