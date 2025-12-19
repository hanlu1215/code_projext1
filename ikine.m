function [q, is_converged] = ikine(T_target, q0)
    % IKINE 数值法求解六轴机械臂逆运动学
    % T_target: 4x4 目标位姿矩阵
    % q0: 1x6 初始关节角向量 (rad)
    
    % D-H 参数表 [a alpha d]
    DH_params = [
        0    pi/2   0.44;
        3    0      0;
        0   -pi/2   0;
        0    pi/2   2.8;
        0   -pi/2   0.24;
        0    0      0.40
    ];
    
    max_iter = 1000;      % 最大迭代次数
    tol = 1e-6;          % 容许误差
    lambda = 0.5;        % 迭代步长
    q = q0(:);           % 确保为列向量
    is_converged = false;

    for i = 1:max_iter
        % 1. 计算当前位姿 (正运动学)
        T_curr = fkine(q, DH_params);
        
        % 2. 计算误差向量 e (6x1)
        % 位置误差
        dp = T_target(1:3, 4) - T_curr(1:3, 4);
        % 姿态误差 (旋转矩阵法)
        Rc = T_curr(1:3, 1:3);
        Rt = T_target(1:3, 1:3);
        dphi = 0.5 * (cross(Rc(:,1), Rt(:,1)) + ...
                      cross(Rc(:,2), Rt(:,2)) + ...
                      cross(Rc(:,3), Rt(:,3)));
        e = [dp; dphi];
        
        % 检查是否收敛
        if norm(e) < tol
            is_converged = true;
            break;
        end
        
        % 3. 计算几何雅可比矩阵 J
        J = calc_jacobian(q, DH_params);
        
        % 4. 更新关节角 (使用阻尼最小二乘法处理奇异性)
        % dq = J \ e; 
        dq = pinv(J) * e; % 使用伪逆增强稳定性
        q = q + lambda * dq;
        
        % 5. 角度归一化到 [-pi, pi] 范围
        q = wrapToPi(q);
    end
end

function T = fkine(q, DH)
    T = eye(4);
    for i = 1:6
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = q(i);
        Ai = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
              sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
              0           sin(alpha)             cos(alpha)            d;
              0           0                      0                     1];
        T = T * Ai;
    end
end

function J = calc_jacobian(q, DH)
    % 数值扰动法或解析法计算雅可比
    J = zeros(6,6);
    T_list = cell(1,7);
    T_list{1} = eye(4);
    curr = eye(4);
    for i = 1:6
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = q(i);
        Ai = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
              sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
              0           sin(alpha)             cos(alpha)            d;
              0           0                      0                     1];
        curr = curr * Ai;
        T_list{i+1} = curr;
    end
    
    Pe = T_list{7}(1:3, 4);
    for i = 1:6
        z_prev = T_list{i}(1:3, 3);
        p_prev = T_list{i}(1:3, 4);
        J(1:3, i) = cross(z_prev, (Pe - p_prev)); % 平移部分
        J(4:6, i) = z_prev;                       % 旋转部分
    end
end