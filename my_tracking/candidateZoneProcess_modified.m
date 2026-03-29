function candidateZoneProcess_modified(cluster, assocRslt)
% 根据候选区-点云簇航迹关联的结果更新候选区和确立区

p = trackParamShare.param;
global trackCand iFrm

iCand2Confirm = [];
if ~isempty(assocRslt.assignments)
    for iAssign = 1 : size(assocRslt.assignments, 1)
        idxTrack = assocRslt.assignments(iAssign, 1);
        idxDet = assocRslt.assignments(iAssign, 2);

        z = cluster.centroid(idxDet, :);
        if strcmp(p.trackAlgo, 'KF')
            trackCand(idxTrack).centroid = correct(trackCand(idxTrack).kalmanFilter, z);
        else
            trackCand(idxTrack).centroid = z;
        end

        trackCand(idxTrack).presence = [trackCand(idxTrack).presence, 1];
        trackCand(idxTrack).presence = trackCand(idxTrack).presence(max(1, length(trackCand(idxTrack).presence) - p.candWin + 1) : end);
        trackCand(idxTrack).ghostLabel = [trackCand(idxTrack).ghostLabel, cluster.ghostLabel(idxDet)];
        trackCand(idxTrack).age = trackCand(idxTrack).age + 1;
        trackCand(idxTrack).trajectory = [trackCand(idxTrack).trajectory; trackCand(idxTrack).centroid];
        trackCand(idxTrack).frame = [trackCand(idxTrack).frame; p.iFrmLoad(iFrm)];

        if trackCand(idxTrack).age >= p.candWin && ...
                sum(trackCand(idxTrack).presence) / min(trackCand(idxTrack).age, p.candWin) >= p.presRatioNew && ...
                ~sum(trackCand(idxTrack).ghostLabel(max(1, length(trackCand(idxTrack).ghostLabel) - p.nFrmNotGhost + 1) : end))
            iCand2Confirm = [iCand2Confirm; idxTrack];
            newConfirm(idxTrack, cell2mat(cluster.pc(idxDet)));
        end
    end
end

renewCandidate(assocRslt.unassignedTracks, iCand2Confirm(:));

if ~isempty(assocRslt.unassignedDetections)
    for iCluster = 1 : length(assocRslt.unassignedDetections)
        idxDet = assocRslt.unassignedDetections(iCluster);
        nOldTraj = structLength(trackCand, 'centroid');
        trackCand(nOldTraj + 1) = struct('centroid', cluster.centroid(idxDet, :), ...
            'kalmanFilter', createNewKF(cluster.centroid(idxDet, :), 'motionType', p.motionType), ...
            'presence', 1, ...
            'ghostLabel', cluster.ghostLabel(idxDet), ...
            'age', 1, ...
            'trajectory', cluster.centroid(idxDet, :), ...
            'frame', p.iFrmLoad(iFrm));
    end
end
end
