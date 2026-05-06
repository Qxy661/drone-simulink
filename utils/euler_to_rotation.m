function R = euler_to_rotation(phi, theta, psi)
%EULER_TO_ROTATION  ZYX 欧拉角 -> 旋转矩阵 (body -> world)
%
%   R = euler_to_rotation(phi, theta, psi)
%
%   输入:
%     phi   - 滚转角 [rad]
%     theta - 俯仰角 [rad]
%     psi   - 偏航角 [rad]
%
%   输出:
%     R - 3x3 旋转矩阵 (body -> world)
%
%   旋转顺序: R = Rz(psi) * Ry(theta) * Rx(phi)

    cp = cos(phi);   sp = sin(phi);
    ct = cos(theta); st = sin(theta);
    cy = cos(psi);   sy = sin(psi);

    R = [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp;
         sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp;
        -st,     ct*sp,             ct*cp            ];
end
