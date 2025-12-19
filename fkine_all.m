function T_all = fkine_all(q)
% =========================================================
% 计算从 {0} 到 {i} (i=1~6) 的齐次变换
% 输出 T_all(:,:,i) = ^0T_i
% =========================================================

    % ----------- D-H 参数 -----------
    DH = [ ...
        0     pi/2   0.44   q(1);
        3     0      0      q(2);
        0    -pi/2   0      q(3);
        0     pi/2   2.8    q(4);
        0    -pi/2   0.24   q(5);
        0     0      0.40   q(6) ];

    T = eye(4);
    T_all = zeros(4,4,6);

    for i = 1:6
        a = DH(i,1);
        alpha = DH(i,2);
        d = DH(i,3);
        theta = DH(i,4);

        A = [ ...
            cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)   a*cos(theta);
            sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)   a*sin(theta);
            0            sin(alpha)               cos(alpha)              d;
            0            0                        0                       1 ];

        T = T * A;
        T_all(:,:,i) = T;
    end
end
