function [alpha, beta, V] = wind_frame_transform(V_body)
%WIND_FRAME_TRANSFORM  机体速度 -> 气流角
%
%   [alpha, beta, V] = wind_frame_transform(V_body)
%
%   输入:
%     V_body - 3x1 机体速度 [u; v; w] (m/s)
%
%   输出:
%     alpha - 迎角 (Angle of Attack) [rad]
%     beta  - 侧滑角 (Sideslip) [rad]
%     V     - 空速 (Airspeed) [m/s]
%
%   定义:
%     alpha = atan2(w, u)     (迎角: 速度矢量与机头水平面的夹角)
%     beta  = asin(v / V)     (侧滑角: 速度矢量偏离对称面的角度)
%     V     = norm(V_body)    (空速大小)

    u = V_body(1);
    v = V_body(2);
    w = V_body(3);

    V = norm(V_body);

    if V < 0.1
        alpha = 0;
        beta = 0;
        return;
    end

    alpha = atan2(w, u);
    beta = asin(max(-1, min(1, v / V)));
end
