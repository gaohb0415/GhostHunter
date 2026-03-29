function kfPredict
% 对候选区、确立区进行KF预测
% 作者: 刘涵凯
% 更新: 2023-3-7

%% 全局变量
global trackCand trackConfirm

%% KF预测
% 候选区
% for iTrack = 1 : structLength(trackCand, 'centroid')
%     trackCand(iTrack).centroid = predict(trackCand(iTrack).kalmanFilter);
% end
% 确立区
for iTrack = 1 : structLength(trackConfirm, 'centroid')
    trackConfirm(iTrack).centroid = predict(trackConfirm(iTrack).kalmanFilter);
end
