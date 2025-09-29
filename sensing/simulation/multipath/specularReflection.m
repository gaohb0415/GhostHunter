function [ptProj, ptMirror, viewpoint] = specularReflection(faceVert, pt, viewPoint)
% 计算点相对于平面的投影点及镜像点
% https://zhuanlan.zhihu.com/p/422696920
% 输入:
% 1. faceVert: 面的数个顶点
% 2. pt: 被反射点
% 3. viewPoint: 视点坐标
% 输出:
% 1. ptProj: 投影点
% 2. ptMirror: 镜像点
% 2. viewpoint: 反射视点, 即在平面上的反射位置
% 作者: 刘涵凯
% 更新: 2023-8-25

%% 平面法向量
vec1 = faceVert(1, :) - faceVert(2, :);
vec2 = faceVert(1, :) - faceVert(3, :);
faceVec = cross(vec1, vec2);

%% 投影点和镜像点
nPt = size(pt, 1);
t = -sum(repmat(faceVec, nPt, 1) .* (pt - faceVert(1, :)), 2) / sum(faceVec .^ 2);
ptProj = pt + repmat(faceVec, nPt, 1) .* repmat(t, 1, 3);
ptMirror = pt + 2 * repmat(faceVec, nPt, 1) .* repmat(t, 1, 3);

%% 反射点
viewVec = ptMirror - viewPoint;
distance = abs(dot((viewPoint - faceVert(1, :)), faceVec)) / norm(faceVec);
distanceMirrorProj = vecnorm(ptMirror - ptProj, 2, 2);
viewpoint = viewPoint + (distance ./ (distanceMirrorProj + distance)) .* viewVec;
