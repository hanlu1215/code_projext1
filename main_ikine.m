clear; clc; close all;
% 初始化机器人模型
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',6);
% D-H 参数表
% [a alpha d theta]
DH = [
    0    pi/2   0.44   0;
    3    0       0      0;
    0    -pi/2    0      0;
    0    pi/2    2.8    0;
    0    -pi/2    0.24   0;
    0    0       0.40   0
    ];
parent = 'base';
for i = 1:size(DH,1)
    body = rigidBody(['link',num2str(i)]);
    joint = rigidBodyJoint(['joint',num2str(i)], 'revolute');
    setFixedTransform(joint, DH(i,:), 'dh');
    body.Joint = joint;
    addBody(robot, body, parent);
    parent = body.Name;
end
% 逆运动学求解器
ik = inverseKinematics('RigidBodyTree', robot);
% 目标位姿
% R = [1 0 0; 0 -1 0; 0 0 -1];
R = axang2rotm([0, 1, 0, pi/2]); % 绕 y 轴旋转 90°
p = [-2; 0; 0]; % 末端位置
weights = [1 1 1  1 1 1];  % 权重向量，这里所有关节的权重相同

T_des = eye(4);
T_des(1:3,1:3) = R;
T_des(1:3,4) = p;
% 尝试多个初始猜测以获取不同解
initial_guesses = [
    zeros(6,1), ...              % 零位
    [0; 0; pi/4; 0; 0; 0], ...   % 上臂型引导
    [0; 0; -pi/4; 0; 0; 0], ...  % 下臂型引导
    [0; pi/6; pi/3; 0; 0; 0], ... % 其他配置
];

solutions = [];
for i = 1:size(initial_guesses, 2)
    [q_temp, solInfo_temp] = ik('link6', T_des, weights, initial_guesses(:,i));
    if strcmp(solInfo_temp.Status, 'success')
        solutions = [solutions, q_temp];
    end
end

% 显示所有找到的解
if ~isempty(solutions)
    disp('找到的所有解:');
    for i = 1:size(solutions, 2)
        fprintf('解 %d:\n', i);
        disp(rad2deg(solutions(:,i))');  % 显示为角度
    end
    
    % 选择上臂型解（通常第2关节为负值代表上臂型）
    shoulder_angles = solutions(2,:);  % 第2关节角度
    [~, idx] = max(shoulder_angles);   % 选择肩关节角度最小的（上臂型）
    q_sol = solutions(:, idx);
    fprintf('\n选择的上臂型解 (解%d):\n', idx);
    disp(rad2deg(q_sol)');
else
    warning('未找到有效解');
    q_sol = zeros(6,1);
end

figure('Name', '机械臂模型', 'NumberTitle', 'off');
ax = gca;

% 绘制基座长方体
base_params = struct('length', 3.6, 'width', 2.36, 'height', 2.1, ...
                     'center', [-3.6/2, 0, -2.1/2]);
drawBox(ax, base_params);
hold(ax, 'on');


% config = [0,pi,-3*pi/2,0,0,0]';  % 机械臂初始构型
% config = zeros(6,1);  % 机械臂初始构型（所有关节角度为0）
config = q_sol;  % 使用逆运动学求解得到的构型
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
