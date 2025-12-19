hll;
%% Monte Carlo 采样点数
N = 100000;   % 可根据性能调整（2e4 ~ 1e5）

%% 关节角范围（弧度）
q_min = [
    -pi;
    -pi/2;
    -170*pi/180;
    -pi;
    -120*pi/180;
    -pi
];

q_max = [
     pi;
     pi/2;
     170*pi/180;
     pi;
     120*pi/180;
     pi
];

%% 预分配末端位置
P = zeros(N,3);

%% Monte Carlo 随机采样
for k = 1:N
    % 在关节空间内均匀随机采样
    q = q_min + (q_max - q_min) .* rand(6,1);

    % 正运动学
    T = fkine(q);

    % 提取末端位置
    P(k,:) = T(1:3,4).';
end

%% 统计末端xyz范围
x_min = min(P(:,1));
x_max = max(P(:,1));
y_min = min(P(:,2));
y_max = max(P(:,2));
z_min = min(P(:,3));
z_max = max(P(:,3));

fprintf('========== 末端工作空间统计 ==========\n');
fprintf('X 范围: [%.4f, %.4f] m，跨度: %.4f m\n', x_min, x_max, x_max - x_min);
fprintf('Y 范围: [%.4f, %.4f] m，跨度: %.4f m\n', y_min, y_max, y_max - y_min);
fprintf('Z 范围: [%.4f, %.4f] m，跨度: %.4f m\n', z_min, z_max, z_max - z_min);
fprintf('采样点数: %d\n', N);
fprintf('=====================================\n\n');

%% 可视化工作空间
figure('Position', [100, 100, 1200, 800], 'Color', 'w');
ax = gca;
% 绘制工作空间点云
scatter3(P(:,1), P(:,2), P(:,3), ...
         2, P(:,3), 'filled', ...
         'MarkerFaceAlpha', 0.3, ...
         'MarkerEdgeAlpha', 0.3);  % 添加透明度
hold on;

%% 绘制机械臂初始构型

% 绘制基座长方体
base_params = struct('length', 3.6, 'width', 2.36, 'height', 2.1, ...
                     'center', [-3.6/2, 0, -2.1/2]);
drawBox(ax, base_params);
hold(ax, 'on');
q_init = zeros(6,1);  % 初始构型（所有关节角度为0）

% 计算各连杆末端位置
joint_positions = zeros(7, 3);  % 包括基座和6个关节
joint_positions(1, :) = [0, 0, 0];  % 基座

% D-H 参数（与fkine.m中一致）
th1 = q_init(1);
th2 = q_init(2);
th3 = q_init(3);
th4 = q_init(4);
th5 = q_init(5);
th6 = q_init(6);

DH = [
    0    pi/2   0.44   th1;
    3    0      0      th2;
    0   -pi/2   0      th3;
    0    pi/2   2.8    th4;
    0   -pi/2   0.24   th5;
    0    0      0.40   th6
];

% 逐个关节计算变换矩阵
T = eye(4);
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
    
    T = T * Ti;
    joint_positions(i+1, :) = T(1:3, 4).';
end

% 绘制机械臂连杆
plot3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), ...
      'r-', 'LineWidth', 3, 'DisplayName', '机械臂初始构型');

% 绘制关节点
plot3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), ...
      'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% 绘制基座坐标系
quiver3(0, 0, 0, 0.2, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver3(0, 0, 0, 0, 0.2, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver3(0, 0, 0, 0, 0, 0.2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(0.22, 0, 0, 'X_0', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
text(0, 0.22, 0, 'Y_0', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');
text(0, 0, 0.22, 'Z_0', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'b');

% 绘制末端坐标系
T_end = fkine(q_init);
R_end = T_end(1:3, 1:3);
p_end = T_end(1:3, 4);
quiver3(p_end(1), p_end(2), p_end(3), R_end(1,1)*0.15, R_end(2,1)*0.15, R_end(3,1)*0.15, 'r', 'LineWidth', 2);
quiver3(p_end(1), p_end(2), p_end(3), R_end(1,2)*0.15, R_end(2,2)*0.15, R_end(3,2)*0.15, 'g', 'LineWidth', 2);
quiver3(p_end(1), p_end(2), p_end(3), R_end(1,3)*0.15, R_end(2,3)*0.15, R_end(3,3)*0.15, 'b', 'LineWidth', 2);

hold off;

% 美化设置
axis equal;
grid on;
box on;
set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.2, 'FontSize', 11);

xlabel('X (m)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Y (m)', 'FontSize', 13, 'FontWeight', 'bold');
zlabel('Z (m)', 'FontSize', 13, 'FontWeight', 'bold');
title(sprintf('6-DOF 机械臂末端工作空间（Monte Carlo 方法，N=%d）', N), ...
      'FontSize', 14, 'FontWeight', 'bold');

view(45, 30);
colormap(jet);


% 设置合适的视角范围
axis tight;
lighting gouraud;
camlight('headlight');
