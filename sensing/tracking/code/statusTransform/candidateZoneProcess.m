function candidateZoneProcess(cluster, assocRslt)
% 根据候选区-点云簇航迹关联的结果更新候选区和确立区
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
% 作者: 刘涵凯
% 更新: 2023-3-9

%% 参数对象及全局变量
p = trackParamShare.param;
global trackCand iFrm

%% 关联成功的候选区更新
iCand2Confirm = []; % 由候选区转换到确立区的轨迹的索引
if ~isempty(assocRslt.assignments)
    for iAssign = 1 : size(assocRslt.assignments, 1)
        idxTrack = assocRslt.assignments(iAssign, 1);
        idxDet = assocRslt.assignments(iAssign, 2);
        % 更新候选区轨迹
        trackCand(idxTrack).centroid = cluster.centroid(idxDet, :);
        trackCand(idxTrack).presence = [trackCand(idxTrack).presence, 1];
        % 只记录最后candWin帧的存在状态
        trackCand(idxTrack).presence = trackCand(idxTrack).presence(...
            max(1, length(trackCand(idxTrack).presence) - p.candWin + 1) : end);
        trackCand(idxTrack).ghostLabel = [trackCand(idxTrack).ghostLabel, cluster.ghostLabel(idxDet)];
        trackCand(idxTrack).age = trackCand(idxTrack).age + 1;
        trackCand(idxTrack).trajectory = [trackCand(idxTrack).trajectory; trackCand(idxTrack).centroid];
        trackCand(idxTrack).frame = [trackCand(idxTrack).frame; p.iFrmLoad(iFrm)];
        % 判断是否允许转入确立区
        % 三个条件必须同时满足:
        % 1. 存在时间达到阈值
        % 2. "存在占比"达到阈值
        % 3. 最后nFrmNotGhost个鬼影标记记录为0
        if trackCand(idxTrack).age >= p.candWin && sum(trackCand(idxTrack).presence) / ...
                min(trackCand(idxTrack).age, p.candWin) >= p.presRatioNew ...
                && ~sum(trackCand(idxTrack).ghostLabel(max(1, length(trackCand(idxTrack).ghostLabel) - p.nFrmNotGhost + 1) : end))
            iCand2Confirm = [iCand2Confirm; idxTrack];
            % 轨迹续接
            if p.linkEn
                connectFlag = connectTrajectory(idxTrack, cell2mat(cluster.pc(idxDet)));
            else
                connectFlag = 0;
            end
            if ~connectFlag
                % 轨迹回溯
                if p.backtrackEn
                    [backtrackRslt.trajectory, backtrackRslt.frame] = ...
                        backtrackTracking(trackCand(idxTrack).trajectory(1, :), trackCand(idxTrack).frame(1));
                    trackCand(idxTrack).trajectory = [backtrackRslt.trajectory; trackCand(idxTrack).trajectory(2 : end, :)];
                    trackCand(idxTrack).frame = [backtrackRslt.frame; trackCand(idxTrack).frame(2 : end)];
                end
                % 将候选区轨迹添加到确立区
                newConfirm(idxTrack, cell2mat(cluster.pc(idxDet)))
                % 将轨迹起始帧作为回溯标记
                if ~p.backtrackFlag || p.backtrackFlag > trackCand(idxTrack).frame(1)
                    p.backtrackFlag = trackCand(idxTrack).frame(1);
                end
            end
        end
    end
end

%% 关联失败的候选区更新
% 更新候选区旧轨迹
renewCandidate(assocRslt.unassignedTracks, iCand2Confirm(:));
% 将候选区-点云簇航迹关联的剩余点云簇转入候选区
if ~isempty(assocRslt.unassignedDetections)
    for iCluster = 1 : length(assocRslt.unassignedDetections)
        idxDet = assocRslt.unassignedDetections(iCluster);
        nOldTraj = structLength(trackCand, 'centroid');
        trackCand(nOldTraj + 1) = struct('centroid', cluster.centroid(idxDet, :), ...
            'kalmanFilter', createNewKF(cluster.centroid(idxDet, :), 'motionType', p.motionType), ...
            'presence', 1, ...
            'ghostLabel', cluster.ghostLabel(iCluster), ...
            'age', 1, ...
            'trajectory', cluster.centroid(idxDet, :), ...
            'frame', p.iFrmLoad(iFrm));
    end
end
