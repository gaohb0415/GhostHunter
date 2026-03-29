function assocRslt = trackAssociation(tracks, detections, ghostLabel, costTh, varargin)
% 航迹关联
% 输入:
% 1. tracks: 轨迹区
% 2. detections: 点云簇质心坐标
% 3. ghostLabel: 点云簇鬼影标记
% 4. costTh: 航迹关联cost阈值
% 5. varargin:
%     - .exchangeEn: 是否交换轨迹和点云簇在航迹关联时的身份. 0-不交换; 1-交换
% 输出:
% assocRslt: 关联结果
% - .cost: 航迹关联cost
% - .assignments: 关联成功的连线
% - .unassignedTracks: 关联失败的轨迹
% - .unassignedDetections: 关联失败的点云簇
% 作者: 刘涵凯
% 更新: 2023-3-10

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('exchangeEn', 0);
p.parse(varargin{:});
exchangeEn = p.Results.exchangeEn;

%% 参数对象及全局变量
p = trackParamShare.param;

%% 初始化输出
assocRslt = struct('cost', [], 'assignments', [], 'unassignedTracks', [], 'unassignedDetections', []);

%% 计算cost
nTrack = structLength(tracks, 'centroid');
nDet = size(detections, 1);
cost = [];
if nDet
    cost = zeros(nTrack, nDet);
    for iTrack = 1 : nTrack
        for iCluster = 1 : size(detections, 1)
            cost(iTrack, iCluster) = norm(tracks(iTrack).centroid - detections(iCluster, :));
        end
    end
end
% 增加鬼影簇的cost
if ~isempty(cost)
    cost(:, find(ghostLabel)) = cost(:, find(ghostLabel)) + p.extraCostGhost;
end

%% 匈牙利算法关联
if exchangeEn
    [assignments, unassignedDetections, unassignedTracks] = assignDetectionsToTracks(cost', costTh);
    assignments = fliplr(assignments);
else
    [assignments, unassignedTracks, unassignedDetections] = assignDetectionsToTracks(cost, costTh);
end
if isempty(cost)
    % 若cost为空, 则轨迹和点云簇至少有一个为空
    if isempty(tracks(1).centroid)
        % 轨迹为空时, 将全部点云簇设为关联失败
        unassignedDetections = (1 : nDet)'; % 这一句耗时有点高
    end
    if isempty(detections)
        % 点云簇为空时, 将全部轨迹设为关联失败
        unassignedTracks = (1 : nTrack)';
    end
end

%% 输出结果
assocRslt.cost = cost;
assocRslt.assignments = assignments;
assocRslt.unassignedTracks = unassignedTracks;
assocRslt.unassignedDetections = unassignedDetections;
