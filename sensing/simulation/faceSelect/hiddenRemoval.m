function [frontFaceIdx, rcs] = hiddenRemoval(vert, viewPoint, faceIdx, ct)
% 遮挡点/面滤除及RCS计算
% 输入:
% 1. vertic: 顶点坐标
% 2. viewPoint: 视点坐标
% 3. faceIdx: 面-顶点关系, [面索引, 顶点索引]
% 4. ct: 各面质心
% 输出:
% 1. frontFaceIdx: 正向面索引
% 2. rcs: 正向面反射强度
% 作者: 刘涵凯
% 更新: 2023-6-28

%% 滤除遮挡点
visibleVert = HPR(vert, viewPoint, 3); % 第三个参数影响该算法中投影圆的半径

%% 提取可见面
visibleFaceVert = ismember(faceIdx, visibleVert);
% visibleFace = find(bitand(bitand(visibleFaceVert(:, 1), visibleFaceVert(:, 2)), visibleFaceVert(:, 3))); % 三个顶点都可见的面
visibleFace = find(bitor(bitor(visibleFaceVert(:, 1), visibleFaceVert(:, 2)), visibleFaceVert(:, 3))); % 最少有一个顶点可见的面
faceIdx = faceIdx(visibleFace, :); % 可见面索引
face = permute(reshape(vert(faceIdx(:), :), [size(faceIdx, 1), 3, 3]), [1, 3, 2]); % 可见面顶点坐标, [面索引, 坐标, 顶点索引]
ct = ct(visibleFace, :); % 可见面质心坐标

%% 筛选正向面
vecFace2View = viewPoint - ct;
vecFace = cross(face(:, :, 2) - face(:, :, 1), face(:, :, 3) - face(:, :, 2));
vecFaceNorm = vecnorm(vecFace, 2, 2);
vecDot = dot(vecFace2View, vecFace, 2);
idxFrontInVisible = find(vecDot > 0); % 正向面在可见面中的索引
% 提取正向面的数据
frontFaceIdx = visibleFace(idxFrontInVisible);
vecFace2View = vecFace2View(idxFrontInVisible, :);
vecFaceNorm = vecFaceNorm(idxFrontInVisible);
vecDot = vecDot(idxFrontInVisible);
% 计算正向面的RCS
faceArea = 0.5 * vecFaceNorm; % 面积
vecCos = vecDot ./ (vecnorm(vecFace2View, 2, 2) .* vecFaceNorm);
rcs = faceArea .* vecCos;
