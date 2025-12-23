clear; clc; close all;
% 初始化机器人模型
robot = build_robot();
%% 设定
% 目标位姿
% R = [1 0 0; 0 -1 0; 0 0 -1];
R = axang2rotm([0, 1, 0, pi/2]); % 绕 y 轴旋转 90°
p = [-2; 0; 0]; % 末端位置
weights = [1 1 1  1 1 1];  % 权重向量，这里所有关节的权重相同

T_des = eye(4);
T_des(1:3,1:3) = R;
T_des(1:3,4) = p;

% 视角设置
view_Az = -60;
view_El = 35;
%% 自定义函数求解
q0 = [0; pi/4; 0; 0; 0; 0];  % 初始猜测
[q_my_sol, is_converged] = ikine(T_des, q0);
if is_converged
    fprintf('=== 自定义逆运动学求解成功 ===\n');
    fprintf('关节角（deg）:\n');
    disp(rad2deg(q_my_sol'));  % 显示为角度
    % 验证结果
    T_my_verify = fkine(q_my_sol);
    pos_my = T_my_verify(1:3,4);
    R_my = T_my_verify(1:3,1:3);
    fprintf('末端位置: [%.4f, %.4f, %.4f]\n', pos_my(1), pos_my(2), pos_my(3));
    fprintf('位置误差: %.6e\n', norm(pos_my - p));
    fprintf('姿态误差: %.6e\n\n', norm(R_my - R, 'fro'));
else
    warning('自定义逆运动学未收敛');
end

%% 逆运动学工具求解
% 逆运动学求解器
ik = inverseKinematics('RigidBodyTree', robot);
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
    fprintf('=== 逆运动学工具求解结果 ===\n');
    fprintf('找到的所有解: %d 个\n', size(solutions, 2));
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
    % 验证结果
    T_tool_verify = fkine(q_sol);
    pos_tool = T_tool_verify(1:3,4);
    R_tool = T_tool_verify(1:3,1:3);
    fprintf('末端位置: [%.4f, %.4f, %.4f]\n', pos_tool(1), pos_tool(2), pos_tool(3));
    fprintf('位置误差: %.6e\n', norm(pos_tool - p));
    fprintf('姿态误差: %.6e\n\n', norm(R_tool - R, 'fro'));
else
    warning('未找到有效解');
    q_sol = zeros(6,1);
end

%% 绘制结果 - 自定义逆运动学方法
if is_converged
    figure('Name', '自定义逆运动学求解结果', 'NumberTitle', 'off');
    ax1 = gca;
    % 绘制基座长方体
    drawBox(ax1);
    hold(ax1, 'on');
    config_my = q_my_sol;  % 使用自定义逆运动学求解得到的构型
    show(robot, config_my, 'Parent', ax1);
    axis(ax1, 'equal');

    grid(ax1, 'on');
    box(ax1, 'on');
    view(ax1, view_Az, view_El);
    % 计算末端位姿并在图中显示位置与欧拉角
    tform_my = fkine(config_my);
    pos_my = tform_my(1:3,4);
    R_my = tform_my(1:3,1:3);
    % 使用 rotm2eul 获取 ZYX 顺序欧拉角（弧度），再转为角度
    eul_my = rotm2eul(R_my, 'ZYX');
    eul_deg_my = rad2deg(eul_my);
    q_my_deg = rad2deg(config_my');
    text_str_my = sprintf(['【自定义方法】\n', ...
        '末端位置: [%.2f, %.2f, %.2f] m\n', ...
        '末端欧拉角 (ZYX): [%.2f°, %.2f°, %.2f°]\n', ...
        '关节角: [%.2f°, %.2f°, %.2f°, %.2f°, %.2f°, %.2f°]'], ...
        pos_my(1), pos_my(2), pos_my(3), eul_deg_my(1), eul_deg_my(2), eul_deg_my(3), ...
        q_my_deg(1), q_my_deg(2), q_my_deg(3), q_my_deg(4), q_my_deg(5), q_my_deg(6));
    
    % 在图形正下方添加文本注释
    annotation('textbox', [0.1, 0.02, 0.8, 0.1], 'String', text_str_my, ...
        'FontSize', 8, 'Color', 'b', 'BackgroundColor', 'none', ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FitBoxToText', 'off');
    
    hold(ax1, 'off');
end

%% 绘制结果 - 逆运动学工具方法
if ~isempty(solutions)
    figure('Name', '逆运动学工具求解结果', 'NumberTitle', 'off');
    ax2 = gca;
    %T_des(1:3,1:3)绘制基座长方体
    drawBox(ax2);
    hold(ax2, 'on');
    config_tool = q_sol;  % 使用逆运动学工具求解得到的构型
    show(robot, config_tool, 'Parent', ax2);
    axis(ax2, 'equal');
    
    grid(ax2, 'on');
    box(ax2, 'on');
    view(ax2, view_Az, view_El);
    % 计算末端位姿并在图中显示位置与欧拉角
    tform_tool = fkine(config_tool);
    pos_tool = tform_tool(1:3,4);
    R_tool = tform_tool(1:3,1:3);
    % 使用 rotm2eul 获取 ZYX 顺序欧拉角（弧度），再转为角度
    eul_tool = rotm2eul(R_tool, 'ZYX');
    eul_deg_tool = rad2deg(eul_tool);
    q_tool_deg = rad2deg(config_tool');
    text_str_tool = sprintf(['【逆运动学工具】\n', ...
        '末端位置: [%.2f, %.2f, %.2f] m\n', ...
        '末端欧拉角 (ZYX): [%.2f°, %.2f°, %.2f°]\n', ...
        '关节角: [%.2f°, %.2f°, %.2f°, %.2f°, %.2f°, %.2f°]'], ...
        pos_tool(1), pos_tool(2), pos_tool(3), eul_deg_tool(1), eul_deg_tool(2), eul_deg_tool(3), ...
        q_tool_deg(1), q_tool_deg(2), q_tool_deg(3), q_tool_deg(4), q_tool_deg(5), q_tool_deg(6));
    
    % 在图形正下方添加文本注释
    annotation('textbox', [0.1, 0.02, 0.8, 0.1], 'String', text_str_tool, ...
        'FontSize', 8, 'Color', 'r', 'BackgroundColor', 'none', ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FitBoxToText', 'off');
    
    hold(ax2, 'off');
end

%% 结果对比
fprintf('\n========== 求解结果对比 ==========\n');
fprintf('目标位置: [%.4f, %.4f, %.4f]\n', p(1), p(2), p(3));
fprintf('目标姿态 (绕Y轴旋转90度):\n');
disp(R);

if is_converged && ~isempty(solutions)
    fprintf('\n关节角对比 (deg):\n');
    fprintf('%10s', '关节');
    for i = 1:6
        fprintf('%12s', sprintf('q%d', i));
    end
    fprintf('\n');
    fprintf('%10s', '自定义:');
    fprintf('%12.4f', rad2deg(q_my_sol'));
    fprintf('\n');
    fprintf('%10s', '工具:');
    fprintf('%12.4f', rad2deg(q_sol'));
    fprintf('\n');
    fprintf('%10s', '差值:');
    fprintf('%12.4f', rad2deg(q_my_sol' - q_sol'));
    fprintf('\n');
    
    fprintf('\n精度对比:\n');
    T_my_final = fkine(q_my_sol);
    T_tool_final = fkine(q_sol);
    fprintf('自定义方法位置误差: %.6e\n', norm(T_my_final(1:3,4) - p));
    fprintf('工具方法位置误差:   %.6e\n', norm(T_tool_final(1:3,4) - p));
    fprintf('自定义方法姿态误差: %.6e\n', norm(T_my_final(1:3,1:3) - T_des(1:3,1:3), 'fro'));
    fprintf('工具方法姿态误差:   %.6e\n', norm(T_tool_final(1:3,1:3) - T_des(1:3,1:3), 'fro'));
end
fprintf('==================================\n');
