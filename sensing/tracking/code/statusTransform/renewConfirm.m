function renewConfirm(cluster, assocRslt)
% 根据确立区-点云簇航迹关联的结果更新确立区
% 注意trajectory, frame, statusRecord属性在本函数中部更新, 而是更新于renewTrajectory
% 因为关联失败的轨迹也会有卡尔曼预测, 所以关联成功与否都会进行新一帧的信息更新
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
% 更新: 2023-3-10

%% 参数对象及全局变量
p = trackParamShare.param;
global iFrm trackConfirm trackWait trackLost ovlpRec

%% 关联成功部分
nAssign = size(assocRslt.assignments, 1);

%% 更新位置和点云
for iAssign = 1 : nAssign
    idxTrack = assocRslt.assignments(iAssign, 1);
    idxDet = assocRslt.assignments(iAssign, 2);
    trackConfirm(idxTrack).pc = cell2mat(cluster.pc(idxDet));
    trackConfirm(idxTrack).centroid = cluster.centroid(idxDet, :);
    % 注意这里不更新trackConfirm的轨迹
end

%% 重叠记录
if structLength(trackConfirm, 'centroid') >= 2
    posAll = vertcat(trackConfirm.centroid);
    iPplAll = vertcat(trackConfirm.iPeople)';
    nPpl = length(iPplAll);
    % 对关联成功的确立区轨迹排列组合
    combo = nchoosek(1 : nPpl, 2);
    % 计算相互距离
    d = sqrt((posAll(combo(:, 1), 1) - posAll(combo(:, 2), 1)) .^ 2 + ...
        (posAll(combo(:, 1), 2) - posAll(combo(:, 2), 2)) .^ 2);
    % 找出相互距离小于阈值的轨迹
    idxComboOvlp = find(d < p.distOvlp);
    % 记录重叠
    for iidx = 1 : length(idxComboOvlp)
        recordOverlap(iFrm, iPplAll(combo(idxComboOvlp(iidx), :)));
    end
end

%% 更新状态等
for iAssign = 1 : nAssign
    idxTrack = assocRslt.assignments(iAssign, 1);
    idxDet = assocRslt.assignments(iAssign, 2);
    if any(isSubMemberOfStruct(ovlpRec(iFrm).ovlp, 'idxSet', trackConfirm(idxTrack).iPeople))
        if strcmp(trackConfirm(idxTrack).status, "overlap")
            trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
        else
            trackConfirm(idxTrack).status = "overlap";
            trackConfirm(idxTrack).statusAge = 1;
        end
        if ~sum(arrayfun(@(x) ismember(trackConfirm(idxTrack).iPeople, x.iPeople), trackWait))
            % 若等待区中无此轨迹, 则新建
            nOldTraj = structLength(trackWait, 'iPeople');
            trackWait(nOldTraj + 1) = struct('centroid', trackConfirm(idxTrack).trajectory(end - 1, :), ...
                'kalmanFilter', trackConfirm(idxTrack).kalmanFilter, ...
                'iPeople', trackConfirm(idxTrack).iPeople, ...
                'name', trackConfirm(idxTrack).name, ...
                'age', 1);
        end
    else
        if ~cluster.ghostLabel(idxDet)
            % 若无鬼影嫌疑
            switch trackConfirm(idxTrack).status
                case "active"
                    % 若原状态为存在, 则状态年龄++
                    trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
                case "miss"
                    % 若原状态为失迹, 则转换为存在
                    trackConfirm(idxTrack).status = "active";
                    trackConfirm(idxTrack).statusAge = 1;
                case {"deviate", "overlap"}
                    % 若原状态为偏航或重叠, 则转换为存在, 并删除等待区
                    trackConfirm(idxTrack).status = "active";
                    trackConfirm(idxTrack).statusAge = 1;
                    iDel = find(arrayfun(@(x) ismember(trackConfirm(idxTrack).iPeople, x.iPeople), trackWait));
                    trackWait = structRowDelete(trackWait, iDel);
            end
        else
            % 若有鬼影嫌疑
            switch trackConfirm(idxTrack).status
                case "active"
                    % 若原状态为存在, 则转换为偏航, 并新建等待区
                    trackConfirm(idxTrack).status = "deviate";
                    trackConfirm(idxTrack).statusAge = 1;
                    if ~sum(arrayfun(@(x) ismember(trackConfirm(idxTrack).iPeople, x.iPeople), trackWait))
                        % 若等待区中无此轨迹, 则新建
                        nOldTraj = structLength(trackWait, 'iPeople');
                        trackWait(nOldTraj + 1) = struct('centroid', trackConfirm(idxTrack).trajectory(end - 1, :), ...
                            'kalmanFilter', trackConfirm(idxTrack).kalmanFilter, ...
                            'iPeople', trackConfirm(idxTrack).iPeople, ...
                            'name', trackConfirm(idxTrack).name, ...
                            'age', 1);
                    end
                case "miss"
                    % 若原状态为失迹, 则转换为偏航, 并新建等待区, 但不重置statusAge(防止轨迹续接等发生错误)
                    trackConfirm(idxTrack).status = "deviate";
                    trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
                    if ~sum(arrayfun(@(x) ismember(trackConfirm(idxTrack).iPeople, x.iPeople), trackWait))
                        % 若等待区中无此轨迹, 则新建
                        nOldTraj = structLength(trackWait, 'iPeople');
                        trackWait(nOldTraj + 1) = struct('centroid', trackConfirm(idxTrack).trajectory(end - 1, :), ...
                            'kalmanFilter', trackConfirm(idxTrack).kalmanFilter, ...
                            'iPeople', trackConfirm(idxTrack).iPeople, ...
                            'name', trackConfirm(idxTrack).name, ...
                            'age', 1);
                    end
                case "overlap"
                    % 若原状态为重叠, 则转换为偏航, 但不重置statusAge
                    trackConfirm(idxTrack).status = "deviate";
                    trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
                case "deviate"
                    % 若原状态为偏航, 则状态年龄++
                    trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
                    iDel = find(arrayfun(@(x) ismember(trackConfirm(idxTrack).iPeople, x.iPeople), trackWait));
            end
        end
    end
end

%% 关联失败部分
iConfirm2Lost = []; % 由确立区转入丢失区的轨迹的索引
if ~isempty(assocRslt.unassignedTracks)
    for iUnassign = 1 : length(assocRslt.unassignedTracks)
        idxTrack = assocRslt.unassignedTracks(iUnassign);
        trackConfirm(idxTrack).pc = [];
        if strcmp(trackConfirm(idxTrack).status, "miss")
            % 若原状态为失迹, 则更新失迹状态
            trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
            if trackConfirm(idxTrack).statusAge == p.nFrmLost
                % 若连续失迹帧数达到阈值, 则转入丢失区
                nOldTraj = structLength(trackLost, 'centroid');
                trackLost(nOldTraj + 1) = struct('centroid', trackConfirm(idxTrack).centroid, ...
                    'iPeople', trackConfirm(idxTrack).iPeople, ...
                    'name', trackConfirm(idxTrack).name, ...
                    'trajectory', trackConfirm(idxTrack).trajectory, ...
                    'frame', trackConfirm(idxTrack).frame);
                iConfirm2Lost = [iConfirm2Lost; idxTrack];
            end
        else
            % 若原状态非失迹, 则转换为失迹
            trackConfirm(idxTrack).status = "miss";
            trackConfirm(idxTrack).statusAge = 1;
        end
    end
end

%% 从确立区中删除轨迹
trackConfirm = structRowDelete(trackConfirm, iConfirm2Lost);
