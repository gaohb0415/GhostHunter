function renewCandidate(iUnassign, iDel)
% 更新关联失败的候选区轨迹
% 输入:
% 1. iUnassign: 关联失败的轨迹的索引
% 2. iDel: 需删除的轨迹的索引
% 作者: 刘涵凯
% 更新: 2023-3-8

%% 参数对象及全局变量
p = trackParamShare.param;
global trackCand

%% 候选区更新
if isempty(iUnassign)
    % 若无关联失败的候选区轨迹, 则不进行候选区更新, 直接删除转移到确立区的轨迹
    trackCand = structRowDelete(trackCand, iDel);
    return
elseif iUnassign == 0
    % 若关联1无剩余点云簇, 则将候选区全部轨迹视为关联失败
    iUnassign = 1 : structLength(trackCand, 'centroid');
end

iCandDel = []; % 从候选区中删除的轨迹的索引
for iiUnassign = 1 : length(iUnassign)
    iCand = iUnassign(iiUnassign);
    trackCand(iCand).age = trackCand(iCand).age + 1;
    trackCand(iCand).presence = [trackCand(iCand).presence, 0]; % 添加最新存在状态为否
    % 只记录最后candWin帧的存在状态
    trackCand(iCand).presence = trackCand(iCand).presence(...
            max(1, length(trackCand(iCand).presence) - p.candWin + 1) : end);

    if trackCand(iCand).age >= round(p.candWin / 2) && sum(trackCand(iCand).presence) / ...
            min(trackCand(iCand).age, p.candWin) <= p.presRatioDel
        % 若轨迹age>=某值, 且存在占比<=某值, 则删除此轨迹
        iCandDel = [iCandDel; iCand];
    end
end

%% 从候选区中删除轨迹
trackCand = structRowDelete(trackCand, [iDel; iCandDel]);
