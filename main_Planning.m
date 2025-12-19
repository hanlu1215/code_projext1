clear; clc; close all;
% 初始化机器人模型
robot = build_robot();
figure('Name', '机械臂模型', 'NumberTitle', 'off');
ax = gca;
% 绘制基座长方体
drawBox(ax);
hold(ax, 'on');

config = [0,pi,-3*pi/2,0,0,0]';  % 机械臂初始构型
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
config_start = [0, pi, -3*pi/2, 0, 0, 0]';  % 起始构型
config_end = zeros(6, 1);  % 结束构型（所有关节角度为0）
n_steps = 50;  % 动画步数

% 为每个关节创建插值轨迹
config_trajectory = zeros(n_joints, n_steps);
for i = 1:n_joints
    config_trajectory(i, :) = linspace(config_start(i), config_end(i), n_steps);
end

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
    config = config_trajectory(:, step);

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


