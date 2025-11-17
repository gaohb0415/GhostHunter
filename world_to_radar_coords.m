function [x_radar, y_radar] = world_to_radar_coords(x_world, y_world, ego_x, ego_y, ego_yaw_deg)
% WORLD_TO_RADAR_COORDS - 将世界坐标系下的点，转换到雷达坐标系下
%
% 输入:
%   x_world (向量): 真值点在世界坐标系下的X坐标
%   y_world (向量): 真值点在世界坐标系下的Y坐标
%   ego_x (标量):   雷达(小推车)在世界坐标系下的【当前X位置】
%   ego_y (标量):   雷达(小推车)在世界坐标系下的【当前Y位置】
%   ego_yaw_deg (标量): 雷达(小推车)的【当前航向角】(左转为正)
%
% 输出:
%   x_radar (向量): 真值点在雷达“第一人称视角”下的X坐标
%   y_radar (向量): 真值点在雷达“第一人称视角”下的Y坐标

%% 1. 平移 (Translation)
% 计算真值点相对于雷达(小推车)的相对位置
dx = x_world - ego_x;
dy = y_world - ego_y;

%% 2. 旋转 (Rotation)
% 将相对坐标，旋转到雷达的“第一人称视角”下
theta_rad = deg2rad(ego_yaw_deg);
c = cos(theta_rad);
s = sin(theta_rad);

% 这是标准的世界坐标系到车辆坐标系的旋转公式
x_radar = dx * c + dy * s;
y_radar = -dx * s + dy * c;

end