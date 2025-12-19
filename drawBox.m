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
end
