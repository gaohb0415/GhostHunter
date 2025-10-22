function rotated_gt = transform_ground_truth(original_gt, yaw_angle_deg)
% TRANSFORM_GROUND_TRUTH - 根据雷达的水平旋转角度，变换真值坐标。
%
% 功能:
%   接收一个在“世界坐标系”（雷达朝向正前方时测量）下的真值结构体，
%   以及雷达的水平旋转角度，输出一个新的、在雷达“新视角”下的真值结构体。
%
% 输入:
%   1. original_gt: 原始的真值结构体。
%      它应包含 .car.x, .car.y 和 .path.x, .path.y 字段。
%   2. yaw_angle_deg: 雷达的水平旋转角度（单位：度）。
%      约定：向左转（逆时针）为正角度，向右转（顺时针）为负角度。
%
% 输出:
%   1. rotated_gt: 一个新的结构体，其x, y坐标已被旋转到雷达的新坐标系下。
%
% 作者: Gemini
% 更新: 2025-10-22

%% 1. 初始化
rotated_gt = original_gt; % 先复制一份原始结构体，保留所有其他信息

%% 2. 准备旋转参数
% 将人类易于理解的角度（度）转换为数学计算需要的弧度
theta_rad = deg2rad(yaw_angle_deg);
c = cos(theta_rad);
s = sin(theta_rad);

%% 3. 对车辆边界框坐标进行旋转
if isfield(original_gt, 'car') && isfield(original_gt.car, 'x')
    % 从原始结构体中获取世界坐标
    x_world = original_gt.car.x;
    y_world = original_gt.car.y;
    
    % 应用2D旋转矩阵公式，计算在雷达新坐标系下的坐标
    rotated_gt.car.x = x_world * c + y_world * s;
    rotated_gt.car.y = -x_world * s + y_world * c;
end

%% 4. 对行人路径坐标进行旋转
if isfield(original_gt, 'path') && isfield(original_gt.path, 'x')
    % 从原始结构体中获取世界坐标
    x_world = original_gt.path.x;
    y_world = original_gt.path.y;
    
    % 应用2D旋转矩阵公式
    rotated_gt.path.x = x_world * c + y_world * s;
    rotated_gt.path.y = -x_world * s + y_world * c;
end

end