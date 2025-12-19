hll
%% 正运动学的验证：fkine vs rigidBodyTree
% 初始化机器人模型
robot = build_robot();

%% 对比fkine和robot计算末端位置
fprintf('========================================\n');
fprintf('对比 fkine 和 rigidBodyTree 的末端位置计算\n');
fprintf('========================================\n\n');

% 测试用例1: 零位姿
fprintf('测试用例1: 零位姿 (所有关节角度为0)\n');
config1 = zeros(6, 1);

% 使用fkine计算
T_fkine1 = fkine(config1);
pos_fkine1 = T_fkine1(1:3, 4);

% 使用robot计算
T_robot1 = getTransform(robot, config1, 'link6');
pos_robot1 = T_robot1(1:3, 4);

fprintf('fkine 末端位置: [%.4f, %.4f, %.4f]\n', pos_fkine1);
fprintf('robot 末端位置: [%.4f, %.4f, %.4f]\n', pos_robot1);
fprintf('位置误差: [%.6f, %.6f, %.6f]\n', abs(pos_fkine1 - pos_robot1));
fprintf('总误差(范数): %.8f\n\n', norm(pos_fkine1 - pos_robot1));

% 测试用例2: 随机位姿
fprintf('测试用例2: 随机关节角度\n');
config2 = deg2rad([30; -45; 60; -30; 45; -60]);

% 使用fkine计算
T_fkine2 = fkine(config2);
pos_fkine2 = T_fkine2(1:3, 4);

% 使用robot计算
T_robot2 = getTransform(robot, config2, 'link6');
pos_robot2 = T_robot2(1:3, 4);

fprintf('关节角度(度): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', rad2deg(config2));
fprintf('fkine 末端位置: [%.4f, %.4f, %.4f]\n', pos_fkine2);
fprintf('robot 末端位置: [%.4f, %.4f, %.4f]\n', pos_robot2);
fprintf('位置误差: [%.6f, %.6f, %.6f]\n', abs(pos_fkine2 - pos_robot2));
fprintf('总误差(范数): %.8f\n\n', norm(pos_fkine2 - pos_robot2));

% 测试用例3: 极限位姿
fprintf('测试用例3: 接近关节极限\n');
config3 = deg2rad([90; 45; -85; 90; -60; 90]);

% 使用fkine计算
T_fkine3 = fkine(config3);
pos_fkine3 = T_fkine3(1:3, 4);

% 使用robot计算
T_robot3 = getTransform(robot, config3, 'link6');
pos_robot3 = T_robot3(1:3, 4);

fprintf('关节角度(度): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', rad2deg(config3));
fprintf('fkine 末端位置: [%.4f, %.4f, %.4f]\n', pos_fkine3);
fprintf('robot 末端位置: [%.4f, %.4f, %.4f]\n', pos_robot3);
fprintf('位置误差: [%.6f, %.6f, %.6f]\n', abs(pos_fkine3 - pos_robot3));
fprintf('总误差(范数): %.8f\n\n', norm(pos_fkine3 - pos_robot3));

% 批量测试
fprintf('========================================\n');
fprintf('批量测试: 100个随机位姿\n');
fprintf('========================================\n');

n_tests = 100;
max_error = 0;
avg_error = 0;
errors = zeros(n_tests, 1);

for i = 1:n_tests
    % 在关节限位内随机生成关节角度
    config_random = [
        deg2rad(rand()*360 - 180);   % ±180°
        deg2rad(rand()*180 - 90);    % ±90°
        deg2rad(rand()*340 - 170);   % ±170°
        deg2rad(rand()*360 - 180);   % ±180°
        deg2rad(rand()*240 - 120);   % ±120°
        deg2rad(rand()*360 - 180)    % ±180°
    ];
    
    % 使用fkine计算
    T_fkine_rand = fkine(config_random);
    pos_fkine_rand = T_fkine_rand(1:3, 4);
    
    % 使用robot计算
    T_robot_rand = getTransform(robot, config_random, 'link6');
    pos_robot_rand = T_robot_rand(1:3, 4);
    
    % 计算误差
    error = norm(pos_fkine_rand - pos_robot_rand);
    errors(i) = error;
    avg_error = avg_error + error;
    if error > max_error
        max_error = error;
    end
end

avg_error = avg_error / n_tests;

fprintf('最大误差: %.8f\n', max_error);
fprintf('平均误差: %.8f\n', avg_error);
fprintf('最小误差: %.8f\n', min(errors));
fprintf('标准差: %.8f\n', std(errors));
fprintf('\n结论: ');
if max_error < 1e-6
    fprintf('两种方法计算结果完全一致！\n');
elseif max_error < 1e-3
    fprintf('两种方法计算结果基本一致，误差在可接受范围内。\n');
else
    fprintf('两种方法存在明显差异，需要检查D-H参数设置。\n');
end
fprintf('========================================\n\n');

%% 可视化对比
fprintf('========================================\n');
fprintf('可视化对比 fkine 和 rigidBodyTree\n');
fprintf('========================================\n\n');

% 定义多个测试位姿
test_configs = {
    [0; 0; 0; 0; 0; 0], '零位姿';
    deg2rad([30; -45; 60; -30; 45; 0]), '位姿1: [30°, -45°, 60°, -30°, 45°, 0°]';
    deg2rad([90; 0; 0; 0; 0; 0]), '位姿2: [90°, 0°, 0°, 0°, 0°, 0°]';
    deg2rad([0; 45; -45; 90; -45; 0]), '位姿3: [0°, 45°, -45°, 90°, -45°, 0°]';
    deg2rad([-60; 30; -60; -90; 60; 0]), '位姿4: [-60°, 30°, -60°, -90°, 60°, 0°]'
};

n_configs = size(test_configs, 1);

% 创建图形窗口
figure('Name', '末端位姿可视化对比', 'NumberTitle', 'off', 'Position', [50, 50, 1600, 900]);

for idx = 1:n_configs
    config = test_configs{idx, 1};
    config_name = test_configs{idx, 2};
    
    % 使用fkine计算
    T_fkine = fkine(config);
    pos_fkine = T_fkine(1:3, 4);
    R_fkine = T_fkine(1:3, 1:3);
    
    % 使用robot计算
    T_robot = getTransform(robot, config, 'link6');
    pos_robot = T_robot(1:3, 4);
    R_robot = T_robot(1:3, 1:3);
    
    % 计算误差
    pos_error = norm(pos_fkine - pos_robot);
    
    % 创建子图
    subplot(2, 3, idx);
    ax = gca;
    
    % 绘制机器人
    show(robot, config, 'Parent', ax, 'PreservePlot', false);
    hold(ax, 'on');
    
    % 绘制fkine计算的末端位置和姿态（红色坐标系）
    scale = 0.3; % 坐标轴长度
    % X轴（红色）
    quiver3(ax, pos_fkine(1), pos_fkine(2), pos_fkine(3), ...
            R_fkine(1,1)*scale, R_fkine(2,1)*scale, R_fkine(3,1)*scale, ...
            'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    % Y轴（绿色）
    quiver3(ax, pos_fkine(1), pos_fkine(2), pos_fkine(3), ...
            R_fkine(1,2)*scale, R_fkine(2,2)*scale, R_fkine(3,2)*scale, ...
            'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    % Z轴（蓝色）
    quiver3(ax, pos_fkine(1), pos_fkine(2), pos_fkine(3), ...
            R_fkine(1,3)*scale, R_fkine(2,3)*scale, R_fkine(3,3)*scale, ...
            'b', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    
    % 绘制fkine末端位置点（红色大圆点）
    plot3(ax, pos_fkine(1), pos_fkine(2), pos_fkine(3), ...
          'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    
    % 绘制robot计算的末端位置（如果有差异，用蓝色标记）
    if pos_error > 1e-6
        plot3(ax, pos_robot(1), pos_robot(2), pos_robot(3), ...
              'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % 绘制连接线显示误差
        plot3(ax, [pos_fkine(1), pos_robot(1)], ...
                  [pos_fkine(2), pos_robot(2)], ...
                  [pos_fkine(3), pos_robot(3)], ...
                  'm--', 'LineWidth', 2);
    end
    
    hold(ax, 'off');
    
    % 设置图形属性
    axis(ax, 'equal');
    grid(ax, 'on');
    box(ax, 'on');
    view(ax, -30, 30);
    
    % 标题显示位姿信息和误差
    title(ax, sprintf('%s\n误差: %.2e', config_name, pos_error), 'FontSize', 9);
    
    % 添加图例（仅第一个子图）
    if idx == 1
        if pos_error > 1e-6
            legend(ax, {'', 'fkine末端', 'robot末端', '位置误差'}, 'Location', 'best', 'FontSize', 8);
        else
            legend(ax, {'', 'fkine=robot末端'}, 'Location', 'best', 'FontSize', 8);
        end
    end
    
    % 打印详细信息
    fprintf('%s\n', config_name);
    fprintf('  fkine位置: [%.4f, %.4f, %.4f]\n', pos_fkine);
    fprintf('  robot位置: [%.4f, %.4f, %.4f]\n', pos_robot);
    fprintf('  位置误差: %.8f\n\n', pos_error);
end

% 添加总标题
sgtitle('fkine vs rigidBodyTree 末端位姿可视化对比 (红色坐标系=fkine)', 'FontSize', 12, 'FontWeight', 'bold');

fprintf('========================================\n');
fprintf('可视化完成！\n');
fprintf('说明: 红色坐标系和圆点表示fkine计算的末端位姿\n');
fprintf('     如果两种方法有差异，会显示蓝色方块和紫色虚线\n');
fprintf('========================================\n\n');


