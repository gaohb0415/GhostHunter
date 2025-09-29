function renewTrajectory
% 轨迹平滑及轨迹结构体更新
% 作者: 刘涵凯
% 更新: 2023-3-9

%% 参数对象及全局变量
p = trackParamShare.param;
global iFrm trackCand trackConfirm trackLost trackWait trajectory

%% 候选区
for iTrack = 1 : structLength(trackCand, 'centroid')
    if size(trackCand(iTrack).trajectory, 1) >= p.smthLen
        % 候选区轨迹长度达到smthLen后再平滑
        trackCand(iTrack).trajectory(end - p.smthLen + 1 : end, :) = ...
            smoothdataV3(trackCand(iTrack).trajectory(end - p.smthLen + 1 : end, :), 1, p.smthMeth, p.smthWin);
        % KF校正
        if strcmp(p.trackAlgo, 'KF')
            trackCand(iTrack).centroid = correct(trackCand(iTrack).kalmanFilter, trackCand(iTrack).centroid);
        end
        % KF校正后更新最后位置
        trackCand(iTrack).trajectory(end, :) = trackCand(iTrack).centroid;
    end
end

%% 确立区
for iTrack = 1 : structLength(trackConfirm, 'centroid')
    nTrajOld = structLength(trajectory(iFrm).track, 'iPeople');
    % 将轨迹写入轨迹结构体
    trajectory(iFrm).track(nTrajOld + 1) = struct('iPeople', trackConfirm(iTrack).iPeople, ...
        'name', trackConfirm(iTrack).name, ...
        'trajectory', [trackConfirm(iTrack).trajectory; trackConfirm(iTrack).centroid], ...
        'frame', [trackConfirm(iTrack).frame; p.iFrmLoad(iFrm)], ...
        'status', trackConfirm(iTrack).status, ...
        'pcLast', trackConfirm(iTrack).pc, ...
        'kalmanFilter', trackConfirm(iTrack).kalmanFilter, ...
        'statusAge', trackConfirm(iTrack).statusAge);
    % 平滑
    iLeft = max(1, length(trajectory(iFrm).track(nTrajOld + 1).frame) - p.smthLen + 1);
    trajectory(iFrm).track(nTrajOld + 1).trajectory(iLeft : end, :) = ...
        smoothdataV3(trajectory(iFrm).track(nTrajOld + 1).trajectory(iLeft : end, :), 1, p.smthMeth, p.smthWin);
    % 当平滑前后的最后位置相距过大时, 说明可能发生了续接或等待区切换
    % 此时若采纳平滑, 则可能导致后续航迹关联中的问题
    % 所以, 当发生这种情况时, 保留原有的最后位置
    if norm(trajectory(iFrm).track(nTrajOld + 1).trajectory(end, :) - trackConfirm(iTrack).centroid) > p.smthDifTh
        trajectory(iFrm).track(nTrajOld + 1).trajectory(end, :) = trackConfirm(iTrack).centroid;
    end
    % 平滑后, 更新确立区
    trackConfirm(iTrack).centroid = trajectory(iFrm).track(nTrajOld + 1).trajectory(end, :);
    trackConfirm(iTrack).frame = [trackConfirm(iTrack).frame; p.iFrmLoad(iFrm)];
    % KF校正
    if strcmp(p.trackAlgo, 'KF')
        trackConfirm(iTrack).centroid = correct(trackConfirm(iTrack).kalmanFilter, trackConfirm(iTrack).centroid);
    end
    % KF校正后再进行一次轨迹结构体更新和最后位置更新
    trajectory(iFrm).track(nTrajOld + 1).kalmanFilter = trackConfirm(iTrack).kalmanFilter;
    trajectory(iFrm).track(nTrajOld + 1).trajectory(end, :) = trackConfirm(iTrack).centroid;
    trackConfirm(iTrack).trajectory = trajectory(iFrm).track(nTrajOld + 1).trajectory;
end

%% 丢失区
% 直接写入轨迹结构体即可
for iTrack = 1 : structLength(trackLost, 'centroid')
    nTrajOld = structLength(trajectory(iFrm).track, 'iPeople');
    iTraj = find(arrayfun(@(x) ismember(x.iPeople, trackLost(iTrack).iPeople), trajectory(iFrm - 1).track));
    trajectory(iFrm).track(nTrajOld + 1) = struct('iPeople', trackLost(iTrack).iPeople, ...
        'name', trackLost(iTrack).name, ...
        'trajectory', trackLost(iTrack).trajectory, ...
        'frame', trackLost(iTrack).frame, ...
        'status', "lost", ...
        'pcLast', [], ...
        'kalmanFilter', [], ...
        'statusAge', []);
end

%% 等待区
% for iTrack = 1 : structLength(trackWait, 'centroid')
%         if strcmp(p.trackAlgo, 'KF')
%             trackWait(iTrack).centroid = correct(trackWait(iTrack).kalmanFilter, trackWait(iTrack).centroid);
%         end
% end
