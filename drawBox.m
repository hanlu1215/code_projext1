% 绘制长方体
function drawBox(ax, params)
% 输入参数:
%   ax - 目标坐标轴
%   params - 结构体，包含:
%       length: 长度（x方向）
%       width: 宽度（y方向）
%       height: 高度（z方向）
%       center: 中心位置 [x, y, z]
%       可选: faceColor, faceAlpha, edgeColor, lineWidth

% 如果只有一个输入参数，使用默认的 base_params
if nargin == 1
    params = struct('length', 3.6, 'width', 2.36, 'height', 2.1, ...
        'center', [-3.6/2, 0, -2.1/2]);
    % 此时 ax 实际上是传入的坐标轴，params 使用默认值
end

% 提取参数
box_length = params.length;
box_width = params.width;
box_height = params.height;
box_center = params.center;

% 可选参数，设置默认值
if isfield(params, 'faceColor')
    faceColor = params.faceColor;
else
    faceColor = [0.5 0.5 0.5];
end

if isfield(params, 'faceAlpha')
    faceAlpha = params.faceAlpha;
else
    faceAlpha = 0.9;
end

if isfield(params, 'edgeColor')
    edgeColor = params.edgeColor;
else
    edgeColor = 'k';
end

if isfield(params, 'lineWidth')
    lineWidth = params.lineWidth;
else
    lineWidth = 1;
end

% 创建长方体的顶点（相对于中心点）
vertices = [
    -box_length/2, -box_width/2, -box_height/2;
    box_length/2, -box_width/2, -box_height/2;
    box_length/2,  box_width/2, -box_height/2;
    -box_length/2,  box_width/2, -box_height/2;
    -box_length/2, -box_width/2,  box_height/2;
    box_length/2, -box_width/2,  box_height/2;
    box_length/2,  box_width/2,  box_height/2;
    -box_length/2,  box_width/2,  box_height/2
    ] + box_center;

% 定义长方体的面
faces = [
    1 2 3 4;  % 底面
    5 6 7 8;  % 顶面
    1 2 6 5;  % 前面
    2 3 7 6;  % 右面
    3 4 8 7;  % 后面
    4 1 5 8   % 左面
    ];

% 绘制长方体
patch('Vertices', vertices, 'Faces', faces, ...
    'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
    'EdgeColor', edgeColor, 'LineWidth', lineWidth, 'Parent', ax);

% 检查是否需要绘制太阳能电池板（默认参数时绘制）
if isfield(params, 'drawSolarPanels')
    drawSolarPanels = params.drawSolarPanels;
else
    drawSolarPanels = (nargin == 1);  % 只有一个输入参数时绘制
end

if drawSolarPanels
    % 绘制两侧太阳能电池板
    drawSolarPanel(ax, box_center, box_width, box_height, 1);   % +y侧
    drawSolarPanel(ax, box_center, box_width, box_height, -1);  % -y侧
end
end

function drawSolarPanel(ax, center, box_width, box_height, side)
% 绘制太阳能电池板
% side: 1表示+y侧，-1表示-y侧
% 两侧电池板法向都指向(1,0,1)方向

% 太阳能电池板参数
panel_width = box_width * 2;      % 电池板宽度（y方向延展，为长方体width的2倍）
panel_height = box_height * 1.2;  % 电池板高度（z方向延展）
panel_thickness = 0.05;           % 电池板厚度

% 定义电池板初始顶点（在yz平面上的矩形，法向指向+x）
panel_vertices_local = [
    0, -panel_width/2, -panel_height/2;
    0,  panel_width/2, -panel_height/2;
    0,  panel_width/2,  panel_height/2;
    0, -panel_width/2,  panel_height/2;
    panel_thickness, -panel_width/2, -panel_height/2;
    panel_thickness,  panel_width/2, -panel_height/2;
    panel_thickness,  panel_width/2,  panel_height/2;
    panel_thickness, -panel_width/2,  panel_height/2
    ];

% 绕y轴旋转45度，使法向从(1,0,0)转到(1,0,1)方向
% 两侧电池板都使用相同的旋转角度
angle = pi/4;  % 45度，使法向指向(1, 0, 1)

Ry = [cos(angle)  0  sin(angle);
    0           1  0;
    -sin(angle)  0  cos(angle)];

% 计算电池板在y方向的偏移（距离原点为width的0.55倍）
y_offset = side * (box_width * 0.55+ panel_width/2);

% 应用旋转并平移到最终位置
panel_vertices = zeros(8, 3);
for i = 1:8
    rotated = Ry * panel_vertices_local(i, :)';
    panel_vertices(i, :) = rotated' + center + [0, y_offset, 0];
end

% 定义电池板的面
panel_faces = [
    1 2 3 4;  % 前面
    5 6 7 8;  % 后面
    1 2 6 5;  % 下边
    2 3 7 6;  % 右边
    3 4 8 7;  % 上边
    4 1 5 8   % 左边
    ];

% 绘制太阳能电池板（深蓝色，半透明）
patch('Vertices', panel_vertices, 'Faces', panel_faces, ...
    'FaceColor', [0.1 0.2 0.5], 'FaceAlpha', 0.7, ...
    'EdgeColor', [0.2 0.3 0.6], 'LineWidth', 1.5, 'Parent', ax);
end
