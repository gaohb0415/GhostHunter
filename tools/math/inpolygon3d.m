function [in,on] = inpolygon3d(polygon,points,varargin)
% Check if a point is within a planar polygon in threedimensional space. The procedure is to appropriately create projections of the polygon and the point and to apply the Matlab function "inpolygon"
% polygon: Enter as N-by-3 Matrix. There must be 3 rows, one for each coordinate. Number of columns N corresponds to the number of corners of the polygon
% points: Enter as M-by-3 Vector. Number of columns M corresponds to the number of points
% E-mail: dimitrij.chudinzow@gmail.com
% 使代码支持批量点输入, 添加了"是否在平面上"的启用设定, 并对代码进行了精简 刘涵凯 更新: 2023-11-11

%%
p = inputParser();
p.CaseSensitive = false;
p.addOptional('inPlaneDetEn', 0); % 是否开启"在平面上"判定. 0-不开启; 非零-以设定数值为阈值
p.parse(varargin{:});
inPlaneDetEn = p.Results.inPlaneDetEn;

%% Check number of corners
number_polygon_corners=size(polygon,1);
if number_polygon_corners<3 % Check if number of corners in polygon is sufficient
    error('Error: not enough corners in polygon (must be 3 at least)');
end

%% Calculations
polygon_vector_1=polygon(1,:)-polygon(2,:); % calculating first direction vector of polygon
polygon_vector_2=polygon(1,:)-polygon(3,:); % calculating second direction vector of polygon
polygon_normal_vector=cross(polygon_vector_1,polygon_vector_2); % normal vector standing orthogonal on oplygon plane
vector_polygon_corner_point_Q=polygon(1,:)-points; % 面某顶点-待检测点向量
number_points = size(points, 1); % 待检测点数量
in = zeros(number_points ,1);
on = zeros(number_points ,1);

if inPlaneDetEn
    idxInPlane = dot(repmat(polygon_normal_vector, number_points, 1),vector_polygon_corner_point_Q, 2) > inPlaneDetEn; % 在多边形平面内的点的索引
    if ~any(idxInPlane); return; end
else
    idxInPlane = 1 : number_points;
end

% point is within the area of the polygon
% first we check whether the normal vector of the polygon is parallel to XY-plane or XZ-plane
if norm(cross(polygon_normal_vector,[1,0,0]))==0 % polygon is parallel to YZ-plane --> set X-coordinates to Zero
    [in(idxInPlane),on(idxInPlane)]=inpolygon(points(idxInPlane, 2),points(idxInPlane, 3),polygon(:,2),polygon(:,3));
elseif norm(cross(polygon_normal_vector,[0,1,0]))==0 % polygon is parallel to XZ-plane --> set Y-coordinates to Zero
    [in(idxInPlane),on(idxInPlane)]=inpolygon(points(idxInPlane, 1),points(idxInPlane, 3),polygon(:,1),polygon(:,3));
else % Polygon is parallel to XY-plane or arbitralily tilted --> set Z-coordinates to Zero
    [in(idxInPlane),on(idxInPlane)]=inpolygon(points(idxInPlane, 1),points(idxInPlane, 2),polygon(:,1),polygon(:,2));
end