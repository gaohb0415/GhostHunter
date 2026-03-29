function renewConfirm_modified(cluster, assocRslt)
% 根据确立区-点云簇航迹关联的结果更新确立区

p = trackParamShare.param;
global iFrm trackConfirm trackWait trackLost ovlpRec

nAssign = size(assocRslt.assignments, 1);

for iAssign = 1 : nAssign
    idxTrack = assocRslt.assignments(iAssign, 1);
    idxDet = assocRslt.assignments(iAssign, 2);
    trackConfirm(idxTrack).pc = cell2mat(cluster.pc(idxDet));
    trackConfirm(idxTrack).centroid = cluster.centroid(idxDet, :);
end

if structLength(trackConfirm, 'centroid') >= 2
    posAll = vertcat(trackConfirm.centroid);
    iPplAll = vertcat(trackConfirm.iPeople)';
    nPpl = length(iPplAll);
    combo = nchoosek(1 : nPpl, 2);
    d = sqrt((posAll(combo(:, 1), 1) - posAll(combo(:, 2), 1)).^2 + ...
        (posAll(combo(:, 1), 2) - posAll(combo(:, 2), 2)).^2);
    idxComboOvlp = find(d < p.distOvlp);
    for iidx = 1 : length(idxComboOvlp)
        recordOverlap(iFrm, iPplAll(combo(idxComboOvlp(iidx), :)));
    end
end

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
    else
        if ~cluster.ghostLabel(idxDet)
            switch trackConfirm(idxTrack).status
                case "active"
                    trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
                case "miss"
                    trackConfirm(idxTrack).status = "active";
                    trackConfirm(idxTrack).statusAge = 1;
                case {"deviate", "overlap"}
                    trackConfirm(idxTrack).status = "active";
                    trackConfirm(idxTrack).statusAge = 1;
                    iDel = find(arrayfun(@(x) ismember(trackConfirm(idxTrack).iPeople, x.iPeople), trackWait));
                    trackWait = structRowDelete(trackWait, iDel);
            end
        else
            switch trackConfirm(idxTrack).status
                case "active"
                    trackConfirm(idxTrack).status = "deviate";
                    trackConfirm(idxTrack).statusAge = 1;
                case "miss"
                    trackConfirm(idxTrack).status = "deviate";
                    trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
                case "overlap"
                    trackConfirm(idxTrack).status = "deviate";
                    trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
                case "deviate"
                    trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
            end
        end
    end
end

iConfirm2Lost = [];
iConfirmDelete = [];
if ~isempty(assocRslt.unassignedTracks)
    for iUnassign = 1 : length(assocRslt.unassignedTracks)
        idxTrack = assocRslt.unassignedTracks(iUnassign);
        trackConfirm(idxTrack).pc = [];

        if strcmp(trackConfirm(idxTrack).status, "miss")
            trackConfirm(idxTrack).statusAge = trackConfirm(idxTrack).statusAge + 1;
        else
            trackConfirm(idxTrack).status = "miss";
            trackConfirm(idxTrack).statusAge = 1;
        end

        if size(trackConfirm(idxTrack).trajectory, 1) < 8 && trackConfirm(idxTrack).statusAge >= 2
            iConfirmDelete = [iConfirmDelete; idxTrack];
            continue;
        end

        if trackConfirm(idxTrack).statusAge >= p.nFrmLost
            % 续桥失败：把最后这段 miss 期间补进去的预测桥回滚掉
            nRollback = min(trackConfirm(idxTrack).statusAge, size(trackConfirm(idxTrack).trajectory, 1));
            if nRollback > 0
                trackConfirm(idxTrack).trajectory(end-nRollback+1:end, :) = [];
                trackConfirm(idxTrack).frame(end-nRollback+1:end) = [];
            end

            if ~isempty(trackConfirm(idxTrack).trajectory)
                trackConfirm(idxTrack).centroid = trackConfirm(idxTrack).trajectory(end, :);
            end

            nOldTraj = structLength(trackLost, 'centroid');
            trackLost(nOldTraj + 1) = struct( ...
                'centroid', trackConfirm(idxTrack).centroid, ...
                'iPeople', trackConfirm(idxTrack).iPeople, ...
                'name', trackConfirm(idxTrack).name, ...
                'trajectory', trackConfirm(idxTrack).trajectory, ...
                'frame', trackConfirm(idxTrack).frame);

            iConfirm2Lost = [iConfirm2Lost; idxTrack];
        end
    end
end

idxDel = unique([iConfirmDelete(:); iConfirm2Lost(:)]);
idxDel = idxDel(idxDel >= 1 & idxDel <= structLength(trackConfirm, 'centroid'));
if ~isempty(idxDel)
    trackConfirm = structRowDelete(trackConfirm, idxDel);
end
end
