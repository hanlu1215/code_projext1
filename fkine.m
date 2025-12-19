function T06 = fkine(q)
% FKINE
% 正运动学（经典 D-H 参数）
%
% 输入：
%   q  - 6x1 或 1x6 关节角向量 [rad]
%
% 输出：
%   T06 - 4x4 齐次变换矩阵（末端相对于基座）

    % -----------------------------
    % 关节变量
    % -----------------------------
    q = q(:);   % 强制列向量
    th1 = q(1);
    th2 = q(2);
    th3 = q(3);
    th4 = q(4);
    th5 = q(5);
    th6 = q(6);

    % -----------------------------
    % D-H 参数 [a alpha d theta]
    % -----------------------------
    DH = [
        0    pi/2   0.44   th1;
        3    0      0      th2;
        0   -pi/2   0      th3;
        0    pi/2   2.8    th4;
        0   -pi/2   0.24   th5;
        0    0      0.40   th6
    ];

    % -----------------------------
    % 正运动学递推
    % -----------------------------
    T06 = eye(4);

    for i = 1:6
        a = DH(i,1);
        alpha = DH(i,2);
        d = DH(i,3);
        theta = DH(i,4);

        Ti = [
            cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)   a*cos(theta);
            sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)   a*sin(theta);
            0            sin(alpha)               cos(alpha)              d;
            0            0                        0                       1
        ];

        T06 = T06 * Ti;
    end
end
