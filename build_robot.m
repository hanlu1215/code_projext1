function robot = build_robot()
%% 初始化机器人模型：用于可视化
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
end