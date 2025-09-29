function identification
% IMU辅助雷达目标身份识别
% 作者: 刘涵凯
% 更新: 2023-3-15

%% 参数对象及全局变量
p = trackParamShare.param;
global iFrm multivRec trajectory assocRec trackConfirm trackLost

%% 若无轨迹, 则直接退出
if ~structLength(multivRec(iFrm).multiv(1).track, 'iPeople')
    return
end

%% 提取该帧的平行宇宙
trackMultiv = multivRec(iFrm).multiv;
nMultiv = structLength(trackMultiv, 'iMultiverse');

%% 身份匹配
sumCost = zeros(nMultiv, 1);
for iMultiv = 1 : nMultiv
    % 匹配
    matchRslt(iMultiv).result = trajectoryIdMatch(trackMultiv(iMultiv).track, ...
        max(1, min(p.identLockFrm + 1, iFrm - p.identWin + 1)) : iFrm);
    % 该平行宇宙的平均cost, 将用于选择新的主宇宙
    sumCost(iMultiv) = matchRslt(iMultiv).result.sumCost;
    % 更新平行宇宙的身份匹配
    for iAssoc = 1 : size(matchRslt(iMultiv).result.match, 1)
        idxTrack = find(arrayfun(@(x) ismember(str2double(matchRslt(iMultiv).result.match(iAssoc, 1)), x.iPeople), ...
            trackMultiv(iMultiv).track));
        trackMultiv(iMultiv).track(idxTrack).name = matchRslt(iMultiv).result.match(iAssoc, 2);
    end
    % 对于匹配失败的轨迹, 若其原身份未被其他轨迹匹配, 则保留原身份
    % 未匹配成功的轨迹在最佳多元宇宙中的iPeople
    idxUnmatch = find(strcmp(matchRslt(iMultiv).result.match(:, 2), "N"));
    for iUnmatch = 1 : length(idxUnmatch)
        % 未匹配成功的iPeople
        iPpl = str2double(matchRslt(iMultiv).result.match(idxUnmatch(iUnmatch), 1));
        % 原身份
        idxTrack = find(arrayfun(@(x) ismember(iPpl, x.iPeople), multivRec(iFrm).multiv(iMultiv).track));
        nameOld = multivRec(iFrm).multiv(iMultiv).track(idxTrack).name;
        %保留原身份
        if ~strcmp(nameOld, "N") && ~any(strcmp(matchRslt(iMultiv).result.match(:, 2), nameOld))
            trackMultiv(iMultiv).track(idxTrack).name = nameOld;
        end
    end
end
% 将更新后的平行宇宙写入记录
multivRec(iFrm).multiv = trackMultiv;

%% 选择新的主宇宙
% meanCost最小的宇宙为新的主宇宙
[minCost, iMatchBest] = min(sumCost);
% 更新轨迹记录
trajectory(iFrm).track = trackMultiv(iMatchBest).track;
% 更新关联表记录
nAssocOld = structLength(assocRec, 'frame');
assocRec(nAssocOld + 1) = struct('frame', iFrm, 'association', assocRec(nAssocOld).association);
for iAssoc = 1 : size(trackMultiv(iMatchBest).association, 1)
    assocRec(nAssocOld + 1).association(assocRec(nAssocOld).association == trackMultiv(iMatchBest).association(iAssoc, 1)) = ...
        trackMultiv(iMatchBest).association(iAssoc, 2);
end

%% 更新各轨迹区
% 更新的过程就是根据新宇宙的轨迹记录重新写入的过程
% 初始化
trackConfirm = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], 'name', [], 'pc', [], ...
    'status', [], 'statusAge', [], 'trajectory', [], 'frame', []);
trackLost = struct('centroid', [], 'iPeople', [], 'name', [], 'trajectory', [], 'frame', []);
% 写入
for iTraj = 1 : structLength(trajectory(iFrm).track, 'iPeople')
    if strcmp(trajectory(iFrm).track(iTraj).status, 'lost')
        % 丢失区
        nTrajOld = structLength(trackLost, 'centroid');
        trackLost(nTrajOld + 1) = struct('centroid', trajectory(iFrm).track(iTraj).trajectory(end, :), ...
            'iPeople', trajectory(iFrm).track(iTraj).iPeople, ...
            'name', trajectory(iFrm).track(iTraj).name, ...
            'trajectory', trajectory(iFrm).track(iTraj).trajectory, ...
            'frame', trajectory(iFrm).track(iTraj).frame);
    else
        % 确立区
        nTrajOld = structLength(trackConfirm, 'centroid');
        trackConfirm(nTrajOld + 1) = struct('centroid', trajectory(iFrm).track(iTraj).trajectory(end, :), ...
            'kalmanFilter', trajectory(iFrm).track(iTraj).kalmanFilter, ...
            'iPeople', trajectory(iFrm).track(iTraj).iPeople, ...
            'name', trajectory(iFrm).track(iTraj).name, ...
            'pc', trajectory(iFrm).track(iTraj).pcLast, ...
            'status', trajectory(iFrm).track(iTraj).status, ...
            'statusAge', trajectory(iFrm).track(iTraj).statusAge, ...
            'trajectory', trajectory(iFrm).track(iTraj).trajectory, ...
            'frame', trajectory(iFrm).track(iTraj).frame);
    end
end

%% 更新等待区、重叠记录和分离记录
if any(assocRec(end).association - assocRec(end - 1).association)
    global ovlpRec sepRec trackWait
    % 更新等待区iPeople
    % name不重要
    for iWait = 1 : structLength(trackWait, 'iPeople')
        idxAssoc = find(assocRec(end - 1).association == trackWait(iWait).iPeople);
        trackWait(iWait).iPeople = assocRec(end).association(idxAssoc);
    end
    % 更新重叠记录和分离记录中的索引
    for iiFrm = 1 : iFrm
        for iOvlp = 1 : structLength(ovlpRec(iiFrm), 'ovlp')
            [~, iAssoc] = ismember(ovlpRec(iiFrm).ovlp(iOvlp).idxSet, assocRec(end - 1).association);
            ovlpRec(iiFrm).ovlp(iOvlp).idxSet = assocRec(end).association(iAssoc);
        end
        for iSep = 1 : structLength(sepRec(iiFrm), 'sep')
            [~, iAssoc] = ismember(sepRec(iiFrm).sep(iSep).idxSet, assocRec(end - 1).association);
            sepRec(iiFrm).sep(iSep).idxSet = assocRec(end).association(iAssoc);
        end
    end
    %% 更换全部平行宇宙的主宇宙iPeople
    for iMultiv = 1 : structLength(multivRec(iFrm).multiv, 'iMultiverse')
        multivRec(iFrm).multiv(iMultiv).association(:, 1) = multivRec(iFrm).multiv(iMatchBest).association(:, 2);
    end
end

%% 删除cost过高的平行宇宙
multivRec(iFrm).multiv = structRowDelete(multivRec(iFrm).multiv, find(sumCost > 5 * minCost));

%% 更新身份识别锁定帧
if structLength(multivRec(iFrm).multiv, 'iMultiverse') == 1 && structLength(trackMultiv, 'iMultiverse') > 1
    % 更新条件: 有平行宇宙被删除, 且删除后仅剩余一个平行宇宙
    % 其实可以不将锁定帧设为当前帧, 可以多保留一些时间用于之后的身份识别, 以后有需要再改
    % p.identLockFrm = iFrm;
end
