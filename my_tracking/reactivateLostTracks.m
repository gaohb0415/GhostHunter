function [cluster, assocRslt] = reactivateLostTracks(cluster, assocRslt, FRAME_PERIOD)
% 用 lost 轨迹去优先接回旧 ID
% 放在 confirm 关联之后、candidate 关联之前

p = trackParamShare.param;
global iFrm trackLost trackConfirm

if isempty(assocRslt.unassignedDetections) || structLength(trackLost, 'centroid') == 0
    return;
end

iDetAll = assocRslt.unassignedDetections(:);
iDetAll = iDetAll(iDetAll >= 1 & iDetAll <= size(cluster.centroid, 1));

if isempty(iDetAll)
    return;
end

% 只用非鬼影检测尝试重激活
iDetValid = iDetAll(cluster.ghostLabel(iDetAll) == 0);
if isempty(iDetValid)
    return;
end

nLost = structLength(trackLost, 'centroid');
nDet = length(iDetValid);
cost = inf(nLost, nDet);

predPos = zeros(nLost, 2);
predVel = zeros(nLost, 2);
lostGap = zeros(nLost, 1);

for iTrack = 1:nLost
    traj = trackLost(iTrack).trajectory;
    frm  = trackLost(iTrack).frame;

    if size(traj, 1) >= 2
        predVel(iTrack, :) = (traj(end, :) - traj(end-1, :)) / FRAME_PERIOD;
    else
        predVel(iTrack, :) = [0, 0];
    end

    lostGap(iTrack) = iFrm - frm(end);
    predPos(iTrack, :) = traj(end, :) + predVel(iTrack, :) * FRAME_PERIOD * lostGap(iTrack);
end

for iTrack = 1:nLost
    gateDist = min(2.5, 0.5 + 0.12 * lostGap(iTrack));   % 丢得越久，门稍微放大
    vLost = norm(predVel(iTrack, :));

    for j = 1:nDet
        idxDet = iDetValid(j);
        z = cluster.centroid(idxDet, :);
        dz = norm(z - predPos(iTrack, :));

        % 检测速度（径向）只作为弱约束
        vDet = abs(cluster.velocity(idxDet));
        vCost = abs(vDet - vLost);

        if dz <= gateDist && vCost <= 2.0
            cost(iTrack, j) = dz + 0.25 * vCost;
        end
    end
end

[assignments, unassignedLost, unassignedDetLocal] = assignDetectionsToTracks(cost, 2.5);

if isempty(assignments)
    return;
end

reactivatedLostIdx = [];
reactivatedDetIdx = [];

for k = 1:size(assignments, 1)
    idxLost = assignments(k, 1);
    idxDet = iDetValid(assignments(k, 2));

    oldTrack = trackLost(idxLost);

    % 用线性插值把中间缺的几帧补起来
    gapFrames = (oldTrack.frame(end) + 1 : iFrm - 1)';
    bridgeTraj = [];
    if ~isempty(gapFrames)
        nGap = length(gapFrames);
        p0 = oldTrack.trajectory(end, :);
        p1 = cluster.centroid(idxDet, :);
        alpha = (1:nGap)' / (nGap + 1);
        bridgeTraj = p0 + alpha .* (p1 - p0);
    end

    nOldConfirm = structLength(trackConfirm, 'centroid');
    trackConfirm(nOldConfirm + 1) = struct( ...
        'centroid', cluster.centroid(idxDet, :), ...
        'kalmanFilter', createNewKF(cluster.centroid(idxDet, :), 'motionType', p.motionType), ...
        'iPeople', oldTrack.iPeople, ...
        'name', oldTrack.name, ...
        'pc', cell2mat(cluster.pc(idxDet)), ...
        'status', "active", ...
        'statusAge', 1, ...
        'trajectory', [oldTrack.trajectory; bridgeTraj], ...
        'frame', [oldTrack.frame; gapFrames]);

    reactivatedLostIdx = [reactivatedLostIdx; idxLost];
    reactivatedDetIdx = [reactivatedDetIdx; idxDet];
end

% 删除已重激活的 lost 轨迹
reactivatedLostIdx = unique(reactivatedLostIdx);
reactivatedLostIdx = reactivatedLostIdx(reactivatedLostIdx >= 1 & reactivatedLostIdx <= structLength(trackLost, 'centroid'));
if ~isempty(reactivatedLostIdx)
    trackLost = structRowDelete(trackLost, reactivatedLostIdx);
end

% 从 unassignedDetections 中移除已重激活检测
assocRslt.unassignedDetections = setdiff(assocRslt.unassignedDetections, unique(reactivatedDetIdx), 'stable');

end