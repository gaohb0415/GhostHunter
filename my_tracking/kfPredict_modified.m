function kfPredict
% 对候选区、确立区进行KF预测

global trackCand trackConfirm
p = trackParamShare.param;

if strcmp(p.trackAlgo, 'KF')
    for iTrack = 1 : structLength(trackCand, 'centroid')
        trackCand(iTrack).centroid = predict(trackCand(iTrack).kalmanFilter);
    end
    for iTrack = 1 : structLength(trackConfirm, 'centroid')
        trackConfirm(iTrack).centroid = predict(trackConfirm(iTrack).kalmanFilter);
    end
end
end
