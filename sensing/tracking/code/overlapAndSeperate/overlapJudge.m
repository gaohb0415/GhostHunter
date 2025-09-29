function ovlpRec = overlapJudge(cluster, assocRslt)
% 对关联失败轨迹进行重叠判定, 以进行后续的GMM聚类和重叠处理
% 在调用函数前已确认存在"成功关联的轨迹-点云簇对"和关联失败轨迹
% 输入:
% 1. cluster: 聚类结果
%     - .pc: 簇内点云坐标
%     - .centroid: 簇质心坐标
%     - .ghostLabel: 鬼影标记
% 2. assocRslt: 关联结果
%     - .cost: 航迹关联cost
%     - .assignments: 关联成功的连线
%     - .unassignedTracks: 关联失败的轨迹
%     - .unassignedDetections: 关联失败的点云簇
% 输出:
% ovlpRec: 轨迹重叠记录
% - .idxSet: 重叠轨迹的iPeople对
% 作者: 刘涵凯
% 更新: 2023-3-12

%% 参数对象及全局变量
p = trackParamShare.param;
global trackConfirm

%% 初始化输出
ovlpRec = struct('idxSet', []);

%% 首先基于剩余轨迹坐标到点云簇质心的距离做重叠判断
% 关联成功簇的索引
idxDetAssign = assocRslt.assignments(:, 2);
% 关联失败轨迹与关联成功簇的cost
costUnassign = assocRslt.cost(assocRslt.unassignedTracks, idxDetAssign);
while min(costUnassign(:)) < p.thOvlp(1)
    % 重叠判定条件: cost小于阈值(1)
    % 获取"重叠的关联失败轨迹"与"重叠簇"在costUnassign中的索引
    [idxTrackUnassign, idxCluster] = find(costUnassign == min(min(costUnassign)));
    % 记录重叠
    ovlpRec = overlapping(ovlpRec, idxTrackUnassign, idxCluster, assocRslt);
    % 更新costUnassign和关联失败轨迹记录以进行下一轮重叠判断
    costUnassign(idxTrackUnassign, :) = [];
    assocRslt.unassignedTracks(idxTrackUnassign) = [];
    if isempty(assocRslt.unassignedTracks)
        break
    end
end

%% 然后基于剩余轨迹坐标到簇质心的距离和到点云的最小距离做判断
if ~isempty(assocRslt.unassignedTracks)
    % 关联失败轨迹与关联成功簇的cost
    costUnassign = assocRslt.cost(assocRslt.unassignedTracks, idxDetAssign);
    % 关联成功簇的点云
    pcAssign = cluster.pc(idxDetAssign);
    % 关联失败轨迹的数量
    nTrackUnassign = length(assocRslt.unassignedTracks);
    % 关联成功簇的数量
    nCluster = length(pcAssign);
    % 计算各轨迹到各簇内点云的最小距离
    dMin = zeros(nTrackUnassign, nCluster);
    for iTrack = 1 : nTrackUnassign
        for iCluster = 1 : nCluster
            posDif = cell2mat(pcAssign(iCluster)) - trackConfirm(assocRslt.unassignedTracks(iTrack)).centroid;
            dMin(iTrack, iCluster) = min(sqrt(posDif(:, 1) .^ 2 + posDif(:, 2) .^ 2));
        end
    end

    iDel = [];
    for iTrack = 1 : nTrackUnassign
        % 该轨迹到各簇内点云的最小距离及对应簇的索引
        [dMinTemp, idxCluster] = min(dMin(iTrack, :));
        if dMinTemp < p.thOvlp(2) || (dMinTemp < p.thOvlp(3) && costUnassign(iTrack, idxCluster) < p.thOvlp(4))
            % 重叠判定条件: 
            % 1. 与点云的最小距离小于阈值(2)
            % 2. 或: 与点云的最小距离小于阈值(3)且cost小于阈值(4)
            % 获取"重叠的关联失败轨迹"与"重叠簇"在costUnassign中的索引
            [idxTrackUnassign, idxCluster] = find(dMin == min(min(dMin)));
            % 记录重叠
            ovlpRec = overlapping(ovlpRec, idxTrackUnassign, idxCluster, assocRslt);
            iDel = [iDel; idxTrackUnassign];
        end
    end
    % 更新关联失败轨迹记录
    assocRslt.unassignedTracks(iDel) = [];
end
end

%% 重叠记录
function ovlpRec = overlapping(ovlpRec, idxTrackUnassign, idxCluster, assocRslt)
% 该函数与recordOverlap函数很像, 最主要的区别在于, overlapping的输出直接为GMM聚类服务
% 也就是说, overlapping提供的重叠记录主要用于指示要对哪些点云簇做GMM聚类
% 而不像一样直接记录在轨迹重合记录结构体中
% 这是因为, 有时重叠簇经过GMM聚类后, 点云簇被分割得很清晰, 质心相距也较远
% 所以没有必要作为重叠轨迹记录下来, 这可能造成追踪过程的混乱和平行宇宙的计算量增加
% 并且, 在renewConfirm函数中, 会对轨迹重叠进行统一判定

%% 参数对象及全局变量
global trackConfirm

%% 恢复身份索引
% 应对两个cost相等的情况
idxTrackUnassign = idxTrackUnassign(1);
idxCluster = idxCluster(1); % 应对两个cost相等的情况
% 将costExtract中的索引转化为assocRslt中的索引
idxTrackUnassign = assocRslt.unassignedTracks(idxTrackUnassign);
idxCluster = assocRslt.assignments(idxCluster, 2);
% 重叠的另一条轨迹的索引
idxTrackAssign = assocRslt.assignments(assocRslt.assignments(:, 2) == idxCluster, 1);
% 两条轨迹的身份索引
iPplOvlp1 = trackConfirm(idxTrackAssign).iPeople;
iPplOvlp2 = trackConfirm(idxTrackUnassign).iPeople;

%% 生成输出的重叠记录
% 检查待记录ID是否已记录在案
iSet1 = find(arrayfun(@(x) ismember(iPplOvlp1, x.idxSet), ovlpRec));
iSet2 = find(arrayfun(@(x) ismember(iPplOvlp2, x.idxSet), ovlpRec));
if isempty(iSet1) && isempty(iSet2)
    % 若两条轨迹都尚未被记录, 则创建新记录
    nRecOld = structLength(ovlpRec, 'idxSet');
    ovlpRec(nRecOld + 1).idxSet = [iPplOvlp1, iPplOvlp2];
elseif ~isempty(iSet1) && isempty(iSet2)
    % 若轨迹1已被记录, 轨迹2尚未被记录, 则在含轨迹1的记录中加入轨迹2
    ovlpRec(iSet1).idxSet = [ovlpRec(iSet1).idxSet, iPplOvlp2];
elseif isempty(iSet1) && ~isempty(iSet2)
    % 若轨迹2已被记录, 轨迹1尚未被记录, 则在含轨迹2的记录中加入轨迹1
    ovlpRec(iSet2).idxSet = [ovlpRec(iSet2).idxSet, iPplOvlp1];
end
end
