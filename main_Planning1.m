clear; clc; close all;
% 初始化机器人模型
robot = build_robot();
figure('Name', '机械臂模型', 'NumberTitle', 'off');
ax = gca;
% 绘制基座长方体
drawBox(ax);
hold(ax, 'on');

config = [0,0,0,0,0,0]';  % 机械臂初始构型
% config = [0,pi,-3*pi/2,0,0,0]';  % 机械臂初始构型
% config = zeros(6,1);  % 机械臂初始构型（所有关节角度为0）
show(robot, config);
axis(ax, 'equal');
view_Az = -30;
view_El = 40;
grid(ax, 'on');
box(ax, 'on');
view(ax, view_Az, view_El);
% 计算末端位姿并在图中显示位置与欧拉角
tform = fkine(config);
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

%%
% 动画参数设置
n_joints = 6;  % 关节数量
t_total = 5.0;  % 总运动时间（秒）
config_start = zeros(6,1);  % 机械臂初始构型（所有关节角度为0）
% config_start = [0, pi, -3*pi/2, 0, 0, 0]';  % 起始构型
target_position = [4.4,-0.9,-1.0];  % 填入期望的末端位置坐标
target_eul = deg2rad([0, 90, 0]);  % 转换为弧度

% 2. 将欧拉角转换为旋转矩阵
R_target = eul2rotm(target_eul, 'ZYX');

% 3. 构建齐次变换矩阵
T_target = [R_target, target_position(:); 0 0 0 1];

% 4. 用自定义 ikine 计算目标关节解（只算一次）
q0 = [0; 0; pi/4; 0; 0; 0];   % 人工选定一个“解分支引导”
[config_end, is_converged] = ikine(T_target, q0);

if ~is_converged
    error('自定义逆运动学未收敛');
end

ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];
n_steps = 50;  % 动画步数
dt = t_total / (n_steps - 1);  % 时间间隔

% 为每个关节创建插值轨迹
% 生成五次多项式轨迹
config_trajectory = zeros(n_joints, n_steps);
velocity_trajectory = zeros(n_joints, n_steps);
acceleration_trajectory = zeros(n_joints, n_steps);
time_vector = linspace(0, t_total, n_steps);  % 时间向量
for i = 1:n_joints
    for j = 1:n_steps
        t = time_vector(j);
        [config_trajectory(i, j), velocity_trajectory(i, j), acceleration_trajectory(i, j)] = ...
            quintic_polynomial(config_start(i), config_end(i), 0, t_total, t);
    end
end

% 第一阶段：关节空间五次多项式（已有）
q_mid = config_trajectory(:, end);

% 第二阶段：沿第六关节 Z 轴正方向平移
move_dist = 0.50;   % 沿 link6 +Z 平移 30 cm
n_cart = 50;

[config_cart, p_cart] = cartesianZApproach_link6Z( ...
    robot, ik, q_mid, move_dist, n_cart, weights);

% 拼接轨迹
config_trajectory_full = [config_trajectory, config_cart(:,2:end)];

n_steps_full = size(config_trajectory_full, 2);
    


colors = {'b', 'g', 'r', 'c', 'm', 'k'};  % 不同关节的颜色
% line_styles = {'-', '--', ':', '-.', '-', '--'};  % 线型
joint_names = {'关节1', '关节2', '关节3', '关节4', '关节5', '关节6'};

figure('Name', '所有关节运动参数汇总图', 'NumberTitle', 'off', 'Position', [250, 250, 1200, 600]);
% 创建3个子图：角度、速度、加速度
for plot_type = 1:3
    subplot(3, 1, plot_type);
    hold on;
    
    for joint_idx = 1:n_joints
        switch plot_type
            case 1  % 角度
                data = rad2deg(config_trajectory(joint_idx, :));
                ylabel_str = '关节角度 (°)';
                plot_title = '(a) 关节角度';
            case 2  % 速度
                data = rad2deg(velocity_trajectory(joint_idx, :));
                ylabel_str = '关节速度 (°/s)';
                plot_title = '(b) 关节速度';
            case 3  % 加速度
                data = rad2deg(acceleration_trajectory(joint_idx, :));
                ylabel_str = '关节加速度 (°/s²)';
                plot_title = '(c) 关节加速度';
        end
        plot(time_vector, data, ...
            'Color', colors{joint_idx}, ...
            'LineWidth', 1.5, ...
            'DisplayName', joint_names{joint_idx});
    end
    
    xlabel('时间 (s)');
    ylabel(ylabel_str);
    title(plot_title);
    grid on;
    box on;
    legend('Location', 'best');
    xlim([0, t_total]);
end

% 添加总标题
sgtitle('六自由度机械臂各关节运动参数汇总', 'FontSize', 14, 'FontWeight', 'bold');

% 创建图形窗口
figure('Name', '机械臂运动动画', 'NumberTitle', 'off', 'Position', [100, 100, 1920, 1080]);
ax = gca;

% 初始显示以确定坐标轴范围
show(robot, config_start, 'Parent', ax);
axis(ax, 'equal');
grid(ax, 'on');
view_Az = -30;
view_El = 40;
view(ax, view_Az, view_El);

% 固定坐标轴范围
xlim_range = [-4,6.2];
ylim_range = [-6.2,6.2];
zlim_range = [-5,6];

% 创建视频写入对象
video_filename = 'robot_animation.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 30;  % 设置帧率（提高到30fps）
v.Quality = 100;   % 设置视频质量（最高质量）
open(v);

% 初始化末端轨迹存储
end_effector_trajectory = zeros(n_steps, 3);

% 动画循环
for step = 1:n_steps
    % 从轨迹中获取当前步的关节角度配置
    config = config_trajectory_full(:, step);

    % 计算末端执行器位置
    end_effector_transform = getTransform(robot, config, 'link6');
    end_effector_pos = end_effector_transform(1:3, 4);
    end_effector_trajectory(step, :) = end_effector_pos';

    % 清除当前axes并显示机器人
    cla(ax);

    % 绘制基座长方体
    drawBox(ax);

    hold(ax, 'on');
    show(robot, config, 'Parent', ax, 'PreservePlot', false);

    % 绘制末端轨迹
    if step > 1
        % 绘制灰色轨迹线
        plot3(ax, end_effector_trajectory(1:step, 1), ...
            end_effector_trajectory(1:step, 2), ...
            end_effector_trajectory(1:step, 3), ...
            'Color', [0.5 0.5 0.5], 'LineWidth', 2.5);

        % 绘制初始点（绿色）
        plot3(ax, end_effector_trajectory(1, 1), ...
            end_effector_trajectory(1, 2), ...
            end_effector_trajectory(1, 3), ...
            'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

        % 绘制当前终止点（红色）
        plot3(ax, end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), ...
            'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end

    hold(ax, 'off');

    % 固定坐标轴范围和视角
    axis(ax, 'equal');
    xlim(ax, xlim_range);
    ylim(ax, ylim_range);
    zlim(ax, zlim_range);
    view(ax, view_Az, view_El);

    grid(ax, 'on');
    box(ax, 'on');

    % 更新标题，显示各关节的当前角度
    title(ax, sprintf('步骤: %d/%d | θ=[%.2f°, %.2f°, %.2f°, %.2f°, %.2f°, %.2f°]', ...
        step, n_steps, rad2deg(config(1)), rad2deg(config(2)), rad2deg(config(3)), ...
        rad2deg(config(4)), rad2deg(config(5)), rad2deg(config(6))));

    % 暂停以产生动画效果
    drawnow;

    % 捕获当前帧并写入视频
    frame = getframe(gcf);
    writeVideo(v, frame);

    pause(0.05);
end

% 关闭视频文件
close(v);
fprintf('视频已保存为: %s\n', video_filename);

% 保持最终状态显示
title(ax, '动画完成 - 最终位置');

function [q, qd, qdd] = quintic_polynomial(q0, qf, t0, tf, t)
    % 计算归一化时间
    tau = (t - t0) / (tf - t0);
    
    % 五次多项式系数
    q = q0 + (qf - q0) * (10*tau^3 - 15*tau^4 + 6*tau^5);
    qd = (qf - q0) * (30*tau^2 - 60*tau^3 + 30*tau^4) / (tf - t0);
    qdd = (qf - q0) * (60*tau - 180*tau^2 + 120*tau^3) / (tf - t0)^2;
end

function [config_cart, p_traj] = cartesianZApproach_link6Z( ...
    robot, ik, q_start, move_dist, n_points, weights)
%--------------------------------------------------------------------------
% 功能：
%   保持第六关节姿态不变，使末端沿第六关节自身 Z 轴正方向平移
%
% 输入：
%   robot     - RigidBodyTree
%   ik        - inverseKinematics 对象
%   q_start   - 起始关节角（nx1）
%   move_dist - 沿 link6 +Z 方向的平移距离（m，正值）
%   n_points  - 插补点数
%   weights   - IK 权重
%
% 输出：
%   config_cart - 关节轨迹（nxn_points）
%   p_traj      - 末端位置轨迹（3xn_points）
%--------------------------------------------------------------------------

n_joints = numel(q_start);
config_cart = zeros(n_joints, n_points);
p_traj = zeros(3, n_points);

% 当前第六关节位姿
T6_start = getTransform(robot, q_start, 'link6');
R6 = T6_start(1:3,1:3);
p6 = T6_start(1:3,4);

% 第六关节 Z 轴（世界坐标系）
z6 = R6(:,3);

% 初始 IK 猜测
q_guess = q_start;

for i = 1:n_points
    s = (i-1)/(n_points-1);

    % 沿 link6 +Z 方向平移
    p_i = p6 + s * move_dist * z6;
    p_traj(:,i) = p_i;

    % 姿态保持不变
    T_i = [R6, p_i; 0 0 0 1];

    % 连续 IK
    [q_sol, ~] = ik('link6', T_i, weights, q_guess);

    config_cart(:,i) = q_sol;
    q_guess = q_sol;

    % 最终验证
    fprintf('\n最终验证:\n');
    for i = [1, round(n_points/2), n_points]
        T_final = getTransform(robot, config_cart(:,i), 'link6');
        pos_final = T_final(1:3,4);
        z_final = T_final(1:3,3);
        pos_error = norm(pos_final - p_traj(:,i));
        z_error = acosd(min(max(dot(z_final, z6), -1), 1));
        
        fprintf('  点 %d: 位置误差=%.2f mm, Z轴误差=%.2f°\n', ...
            i, pos_error*1000, z_error);
end
end
end