function varargout = quaternion_ops(op, varargin)
%QUATERNION_OPS  四元数运算工具集
%
%   用法:
%     R = quaternion_ops('to_rotation', q)
%     q = quaternion_ops('from_euler', phi, theta, psi)
%     q = quaternion_ops('multiply', q1, q2)
%     q = quaternion_ops('normalize', q)
%     q_dot = quaternion_ops('derivative', q, omega)
%     [phi,theta,psi] = quaternion_ops('to_euler', q)
%     q_inv = quaternion_ops('inverse', q)
%
%   四元数约定: q = [qw, qx, qy, qz] (实部在前)

    switch lower(op)
        case 'to_rotation'
            q = varargin{1};
            varargout{1} = quat_to_rotation(q);

        case 'from_euler'
            phi = varargin{1}; theta = varargin{2}; psi = varargin{3};
            varargout{1} = euler_to_quat(phi, theta, psi);

        case 'multiply'
            q1 = varargin{1}; q2 = varargin{2};
            varargout{1} = quat_multiply(q1, q2);

        case 'normalize'
            q = varargin{1};
            varargout{1} = quat_normalize(q);

        case 'derivative'
            q = varargin{1}; omega = varargin{2};
            varargout{1} = quat_derivative(q, omega);

        case 'to_euler'
            q = varargin{1};
            [phi, theta, psi] = quat_to_euler(q);
            varargout{1} = phi;
            varargout{2} = theta;
            varargout{3} = psi;

        case 'inverse'
            q = varargin{1};
            varargout{1} = quat_inverse(q);

        otherwise
            error('未知运算: %s', op);
    end
end

function R = quat_to_rotation(q)
%QUAT_TO_ROTATION  四元数 -> 旋转矩阵 (body -> world)
    q = quat_normalize(q);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);

    R = [1-2*(qy^2+qz^2),  2*(qx*qy-qw*qz),  2*(qx*qz+qw*qy);
         2*(qx*qy+qw*qz),  1-2*(qx^2+qz^2),  2*(qy*qz-qw*qx);
         2*(qx*qz-qw*qy),  2*(qy*qz+qw*qx),  1-2*(qx^2+qy^2)];
end

function q = euler_to_quat(phi, theta, psi)
%EULER_TO_QUAT  ZYX 欧拉角 -> 四元数
    cp = cos(phi/2);   sp = sin(phi/2);
    ct = cos(theta/2); st = sin(theta/2);
    cy = cos(psi/2);   sy = sin(psi/2);

    q = [cp*ct*cy + sp*st*sy;
         sp*ct*cy - cp*st*sy;
         cp*st*cy + sp*ct*sy;
         cp*ct*sy - sp*st*cy];
    q = quat_normalize(q);
end

function [phi, theta, psi] = quat_to_euler(q)
%QUAT_TO_EULER  四元数 -> ZYX 欧拉角
    q = quat_normalize(q);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);

    % Roll (phi)
    sinr_cosp = 2*(qw*qx + qy*qz);
    cosr_cosp = 1 - 2*(qx^2 + qy^2);
    phi = atan2(sinr_cosp, cosr_cosp);

    % Pitch (theta) - 用 asin 钳位防止数值问题
    sinp = 2*(qw*qy - qz*qx);
    sinp = max(-1, min(1, sinp));
    theta = asin(sinp);

    % Yaw (psi)
    siny_cosp = 2*(qw*qz + qx*qy);
    cosy_cosp = 1 - 2*(qy^2 + qz^2);
    psi = atan2(siny_cosp, cosy_cosp);
end

function q3 = quat_multiply(q1, q2)
%QUAT_MULTIPLY  四元数乘法 (Hamilton 积)
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);

    q3 = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
          w1*x2 + x1*w2 + y1*z2 - z1*y2;
          w1*y2 - x1*z2 + y1*w2 + z1*x2;
          w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

function qn = quat_normalize(q)
%QUAT_NORMALIZE  四元数归一化
    n = norm(q);
    if n < 1e-10
        qn = [1; 0; 0; 0];
    else
        qn = q / n;
    end
end

function q_dot = quat_derivative(q, omega)
%QUAT_DERIVATIVE  四元数时间导数
    omega_quat = [0; omega(:)];
    q_dot = 0.5 * quat_multiply(q, omega_quat);
end

function q_inv = quat_inverse(q)
%QUAT_INVERSE  四元数逆 (共轭/归一化)
    q = quat_normalize(q);
    q_inv = [q(1); -q(2); -q(3); -q(4)];
end
