clear; clc; close all;

%% ================= 1. 初始化 =================
robot = build_robot();

figure('Name','机械臂模型','NumberTitle','off');
ax = gca;
drawBox(ax); hold on;

q_start = zeros(6,1);
show(robot, q_start);
axis equal; grid on; box on;
view(-30,40);
show(robot, q_start);
axis(ax, 'equal');
view_Az = -30;
view_El = 40;
grid(ax, 'on');
box(ax, 'on');
view(ax, view_Az, view_El);
% 计算末端位姿并在图中显示位置与欧拉角
tform = fkine(q_start);
pos = tform(1:3,4);
R = tform(1:3,1:3);
% 使用 rotm2eul 获取 ZYX 顺序欧拉角（弧度），再转为角度
eul = rotm2eul(R, 'ZYX');
eul_deg = rad2deg(eul);
text_str = sprintf('末端位置: [%.2f, %.2f, %.2f] m\n末端欧拉角 (ZYX): [%.2f°, %.2f°, %.2f°]', ...
    pos(1), pos(2), pos(3), eul_deg(1), eul_deg(2), eul_deg(3));

% 在图形正下方添加文本注释
annotation('textbox', [0.15, 0.02, 0.7, 0.08], 'String', text_str, ...
    'FontSize', 8, 'Color', 'b', 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'LineWidth', 0.5, 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', 'FitBoxToText', 'off');

hold(ax, 'off');
%% ================= 2. 第一阶段：到达目标位姿 =================
target_eul = deg2rad([0 90 0]);
R_target = eul2rotm(target_eul,'ZYX');
p_target = [4.4; -0.9; -1.0];
T_target = eye(4);
T_target(1:3,1:3) = R_target;
T_target(1:3,4)   = p_target;

ik_solver = inverseKinematics('RigidBodyTree',robot);
weights = [1 1 1 1 1 1];
[q_end,~] = ik_solver('link6',T_target,weights,q_start);

%% 五次多项式
t_total = 10;
n_steps = 200;
time = linspace(0,t_total,n_steps);

q_traj1  = zeros(6,n_steps);
qd_traj1 = zeros(6,n_steps);
qdd_traj1 = zeros(6,n_steps);

for i = 1:6
    for k = 1:n_steps
        [q_traj1(i,k), qd_traj1(i,k), qdd_traj1(i,k)] = ...
            quintic_poly_full(q_start(i), q_end(i), 0, t_total, time(k));
    end
end

q_mid = q_traj1(:,end);

%% ================= 关节运动参数汇总图（第一阶段） =================

joint_names = {'关节1','关节2','关节3','关节4','关节5','关节6'};
colors = {'b','g','r','c','m','k'};

figure('Name','六自由度机械臂各关节运动参数汇总','Position',[200 200 1200 650]);

% (a) 关节角度
subplot(3,1,1); hold on;
for i = 1:6
    plot(time, rad2deg(q_traj1(i,:)), 'Color', colors{i}, 'LineWidth', 0.5);
end
ylabel('角度 (°)');
title('(a) 关节角度');
grid on; box on;
legend(joint_names,'Location','best');

% (b) 关节速度
subplot(3,1,2); hold on;
for i = 1:6
    plot(time, rad2deg(qd_traj1(i,:)), 'Color', colors{i}, 'LineWidth', 0.5);
end
ylabel('速度 (°/s)');
title('(b) 关节速度');
grid on; box on;
legend(joint_names,'Location','best');


% (c) 关节加速度
subplot(3,1,3); hold on;
for i = 1:6
    plot(time, rad2deg(qdd_traj1(i,:)), 'Color', colors{i}, 'LineWidth', 0.5);
end
xlabel('时间 (s)');
ylabel('加速度 (°/s^2)');
title('(c) 关节加速度');
grid on; box on;
legend(joint_names,'Location','best');


sgtitle('六自由度机械臂各关节运动参数汇总');

%% ================= 3. 第二阶段：沿第六关节 Z 轴平移（关键） =================
move_dist = 0.5;     % 50 cm
n_cart = 101;

[q_traj2, p_traj, err_traj] = cartesian_link6_Z_ikine(robot, q_mid, move_dist, n_cart);


%% ================= 4. 拼接轨迹 =================
q_traj = [q_traj1, q_traj2(:,2:end)];
n_all = size(q_traj,2);
% q_anim = q_traj2;   % 只动画展示笛卡尔空间轨迹
% n_all  = size(q_anim, 2);
% traj = zeros(n_all,3);

%% ================= 5. 动画 =================
figure('Name','机械臂动画','Position',[100 100 800 800]);
ax = gca;
% 固定坐标轴范围
xlim_range = [-4,6.2];
ylim_range = [-6.2,6.2];
zlim_range = [-5,4];

video_filename = 'robot_animation.gif';  % GIF 文件名
delayTime = 0.05;  % 每帧间隔时间（秒）
traj = zeros(n_all,3);
q_history = zeros(n_all, 6);

% for k = 1:n_all
%     q = q_anim(:,k);   % 只取第二段

for k = 1:n_all
    q = q_traj(:,k);
    q_history(k, :) = rad2deg(q)';  % 保存到矩阵
    T = fkine(q);
    traj(k,:) = T(1:3,4)';

    cla(ax);
    drawBox(ax); 
    hold(ax,'on');

    show(robot,q,'PreservePlot',false,'Parent',ax);

    % 绘制末端轨迹（灰色）
    if k > 1
        plot3(ax, traj(1:k,1), traj(1:k,2), traj(1:k,3), 'Color',[0.9 0.5 0.5], 'LineWidth', 1.5);
        % 初始点（绿色）
        plot3(ax, traj(1,1), traj(1,2), traj(1,3), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
        % 当前末端点（红色）
        plot3(ax, traj(k,1), traj(k,2), traj(k,3), 'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
    end
    error_norms = [];
    if k > size(q_traj1,2)
        idx = k - size(q_traj1,2) + 1;
        err_norm = norm(err_traj(:,idx));
        error_norms = [error_norms; err_norm];
    end
    assignin('base', 'cartesian_errors', error_norms);
    axis equal;
    xlim([-4 6]); ylim([-6 6]); zlim([-5 4]);
    view(-30,40); grid on; box on;

    title(ax, sprintf([
        '\\theta = [%.2f°, %.2f°, %.2f°, %.2f°, %.2f°, %.2f°]'], ...
        rad2deg(q(1)), rad2deg(q(2)), rad2deg(q(3)), ...
        rad2deg(q(4)), rad2deg(q(5)), rad2deg(q(6))), 'FontSize', 16);

    drawnow;

    % 捕获当前帧
    frame = getframe(gcf);
    im = frame2im(frame);
    [A, map] = rgb2ind(im, 256);

    % 写入 GIF 文件
    if k == 1
        imwrite(A, map, video_filename, 'gif', 'LoopCount', Inf, 'DelayTime', delayTime);
    else
        imwrite(A, map, video_filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
end
save('q_trajectory.mat', 'q_history');
fprintf('GIF 已保存为: %s\n', video_filename);


function [q_traj, p_traj, err_traj] = cartesian_link6_Z_ikine(robot, q0, dz, N)
    % 沿第六关节自身 Z 轴正方向平移（使用自定义 ikine）
    err_traj = zeros(3, N);   % 三维位置误差
    q_traj = zeros(6,N);
    p_traj = zeros(3,N);
    
    T0 = fkine(q0);
    R6 = T0(1:3,1:3);
    p6 = T0(1:3,4);
    
    q_prev = q0;
    
    for i = 1:N
        s = (i-1)/(N-1);
        p_i = p6 + s * dz * R6(:,3);
        
        T_i = eye(4);
        T_i(1:3,1:3) = R6;
        T_i(1:3,4)   = p_i;
        
        [q_i, ok] = ikine(T_i, q_prev);
        if ~ok
            error('IK 未收敛 @ step %d',i);
        end
        
        % ==== 误差计算（核心）====
        T_actual = fkine(q_i);
        p_actual = T_actual(1:3,4);
        err_traj(:,i) = p_actual - p_i;
        
        q_traj(:,i) = q_i;
        p_traj(:,i) = p_i;
        q_prev = q_i;
        
        
    end
    
    end

function [q, qd, qdd] = quintic_poly_full(q0, qf, t0, tf, t)
    tau = (t - t0) / (tf - t0);

    q = q0 + (qf - q0) * (10*tau^3 - 15*tau^4 + 6*tau^5);
    qd = (qf - q0) * (30*tau^2 - 60*tau^3 + 30*tau^4) / (tf - t0);
    qdd = (qf - q0) * (60*tau - 180*tau^2 + 120*tau^3) / (tf - t0)^2;
end
    
function q = quintic_polynomial(q0,qf,t0,tf,t)
    tau = (t-t0)/(tf-t0);
    q = q0 + (qf-q0)*(10*tau^3 - 15*tau^4 + 6*tau^5);
    end
        