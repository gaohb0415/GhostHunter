function [trajectory, frame] = backtrackTracking(anchorPos, anchorFrm)
% 轨迹回溯
% 输入: 
% 1. anchorPos: 起点坐标
% 2. anchorFrm: 起点帧
% 输出: 
% 1. trajectory: 回溯轨迹
% 2. frame: 回溯帧
% 作者: 刘涵凯
% 更新: 2023-3-9

%% 初始化输出
trajectory = anchorPos;
frame = anchorFrm;
% 若起点为第一帧则直接退出
if anchorFrm == 1; return; end 

%% 参数对象及全局变量
p = trackParamShare.param;
global clusters

%% 回溯初始化
clustersSeg = fliplr(clusters(1 : anchorFrm - 1)); % 将聚类结果片段倒排
newPos = anchorPos;
nMiss = 0; % 连续关联失败数

%% 轨迹回溯
for iFrm = 1 : anchorFrm - 1
    % 提取新一帧聚类结果
    clusterNew.centroid = vertcat(clustersSeg(iFrm).cluster.centroid); % 该帧的所有点云簇的中心坐标
    % 计算cost
    nCluster = size(clusterNew.centroid, 1);
    cost = zeros(1, nCluster);
    for iCluster = 1 : size(clusterNew.centroid, 1)
        cost(iCluster) = norm(newPos - clusterNew.centroid(iCluster, :));
    end
    % 匈牙利算法关联
    [assignments, ~] = assignDetectionsToTracks(cost, p.costBacktrack);
    % 根据匹配结果进行更新
    if ~isempty(assignments)
        % 若关联成功, 则更新回溯轨迹和回溯帧
        newPos = clusterNew.centroid(assignments(2), :);
        trajectory = [newPos; trajectory];
        frame = [anchorFrm - iFrm, frame];
        nMiss = 0;
    else
        nMiss = nMiss + 1;
        if nMiss == p.nBacktrackMissTh
            % 若连续关联失败数达到上限, 则终止回溯
            break
        end
    end
end % for iFrm = 1 : anchorFrm - 1
