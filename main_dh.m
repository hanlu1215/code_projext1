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
view_Az = -20;
view_El = 30;
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
