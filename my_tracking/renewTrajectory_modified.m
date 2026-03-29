function renewTrajectory_modified
% 轨迹平滑及轨迹结构体更新

p = trackParamShare.param;
global iFrm trackCand trackConfirm trackLost trajectory

for iTrack = 1 : structLength(trackCand, 'centroid')
    if size(trackCand(iTrack).trajectory, 1) >= p.smthLen
        trackCand(iTrack).trajectory(end - p.smthLen + 1 : end, :) = ...
            smoothdataV3(trackCand(iTrack).trajectory(end - p.smthLen + 1 : end, :), 1, p.smthMeth, p.smthWin);
        trackCand(iTrack).trajectory(end, :) = trackCand(iTrack).centroid;
    end
end

%% 确立区
MAX_BRIDGE_FRM = 5;   % 最多补5帧预测桥

for iTrack = 1 : structLength(trackConfirm, 'centroid')
    nTrajOld = structLength(trajectory(iFrm).track, 'iPeople');

    hasMeasurement = ~isempty(trackConfirm(iTrack).pc);

    if hasMeasurement
        % 有真实观测：正常追加
        newTraj = [trackConfirm(iTrack).trajectory; trackConfirm(iTrack).centroid];
        newFrame = [trackConfirm(iTrack).frame; p.iFrmLoad(iFrm)];

        % 只对真实观测更新做平滑
        iLeft = max(1, length(newFrame) - p.smthLen + 1);
        newTraj(iLeft:end, :) = smoothdataV3(newTraj(iLeft:end, :), 1, p.smthMeth, p.smthWin);

        if norm(newTraj(end, :) - trackConfirm(iTrack).centroid) > p.smthDifTh
            newTraj(end, :) = trackConfirm(iTrack).centroid;
        end
    else
        % 无观测：短时补桥，超过桥长不再继续补
        if strcmp(trackConfirm(iTrack).status, "miss") && trackConfirm(iTrack).statusAge <= MAX_BRIDGE_FRM
            newTraj = [trackConfirm(iTrack).trajectory; trackConfirm(iTrack).centroid];
            newFrame = [trackConfirm(iTrack).frame; p.iFrmLoad(iFrm)];
        else
            newTraj = trackConfirm(iTrack).trajectory;
            newFrame = trackConfirm(iTrack).frame;
        end
    end

    trajectory(iFrm).track(nTrajOld + 1) = struct( ...
        'iPeople', trackConfirm(iTrack).iPeople, ...
        'name', trackConfirm(iTrack).name, ...
        'trajectory', newTraj, ...
        'frame', newFrame, ...
        'status', trackConfirm(iTrack).status, ...
        'pcLast', trackConfirm(iTrack).pc, ...
        'kalmanFilter', trackConfirm(iTrack).kalmanFilter, ...
        'statusAge', trackConfirm(iTrack).statusAge);

    trackConfirm(iTrack).trajectory = newTraj;
    trackConfirm(iTrack).frame = newFrame;
end



for iTrack = 1 : structLength(trackLost, 'centroid')
    nTrajOld = structLength(trajectory(iFrm).track, 'iPeople');
    trajectory(iFrm).track(nTrajOld + 1) = struct('iPeople', trackLost(iTrack).iPeople, ...
        'name', trackLost(iTrack).name, ...
        'trajectory', trackLost(iTrack).trajectory, ...
        'frame', trackLost(iTrack).frame, ...
        'status', "lost", ...
        'pcLast', [], ...
        'kalmanFilter', [], ...
        'statusAge', []);
end
end
