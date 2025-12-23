function [q, is_converged] = ikine(T_target, q0)
    % IKINE: 基于 Newton-Raphson 迭代法的六轴机械臂逆运动学数值解
    % 输入: T_target (4x4 目标矩阵), q0 (1x6 初始角度 rad)
    % 输出: q (1x6 解), is_converged (收敛标志)

    % 标准 D-H 参数定义 [a, alpha, d]
    DH = [
        0    pi/2   0.44;
        3    0      0;
        0   -pi/2   0;
        0    pi/2   2.8;
        0   -pi/2   0.24;
        0    0      0.40
    ];
    % 迭代参数设置
    max_iter = 500;     % 最大迭代次数
    tol = 1e-8;         % 收敛容许误差
    step_size = 0.5;    % 迭代步长因子
    damping = 0.01;     % 阻尼因子 (防止奇异点)
    
    q = q0(:);          % 统一为列向量
    is_converged = false;

    for k = 1:max_iter
        % 1. 正运动学: 计算当前末端位姿
        T_curr = eye(4);
        A = cell(1,6);
        for i = 1:6
            th = q(i); a = DH(i,1); al = DH(i,2); d = DH(i,3);
            A{i} = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
                    sin(th)  cos(th)*cos(al) -cos(th)*sin(al) sin(th)*a;
                    0        sin(al)          cos(al)         d;
                    0        0                0               1];
            T_curr = T_curr * A{i};
        end

        % 2. 计算误差向量 e
        % 位置误差 dp
        dp = T_target(1:3,4) - T_curr(1:3,4);
        % 姿态误差 dphi (n, o, a 向量法)
        nc = T_curr(1:3,1); oc = T_curr(1:3,2); ac = T_curr(1:3,3);
        nt = T_target(1:3,1); ot = T_target(1:3,2); at = T_target(1:3,3);
        dphi = 0.5 * (cross(nc,nt) + cross(oc,ot) + cross(ac,at));
        e = [dp; dphi];

        % 3. 收敛判断
        if norm(e) < tol
            is_converged = true;
            break;
        end
        % 4. 计算几何雅可比矩阵 J
        J = zeros(6,6);
        positions = zeros(3,7); 
        z_vectors = zeros(3,6);
        positions(:,1) = [0;0;0];
        % 预计算各坐标系位置和Z轴指向
        temp_T = eye(4);
        for i = 1:6
            z_vectors(:,i) = temp_T(1:3,3);
            positions(:,i) = temp_T(1:3,4);
            temp_T = temp_T * A{i};
        end
        pe = temp_T(1:3,4); % 末端位置
        for i = 1:6
            J(1:3,i) = cross(z_vectors(:,i), (pe - positions(:,i)));
            J(4:6,i) = z_vectors(:,i);
        end

        % 5. 更新关节角 (阻尼最小二乘更新)
        dq = (J' * J + damping^2 * eye(6)) \ (J' * e);
        q = q + step_size * dq;
    end
    
    % 最终结果归一化到 [-π, π]
    q = wrapToPi(q);

end