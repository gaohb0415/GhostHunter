function overlapBacktrack
% 重叠记录回溯
% 当发生轨迹回溯、续接、等待区覆盖等事件时, 对回溯部分重新进行重叠记录
% 作者: 刘涵凯
% 更新: 2023-3-11

%% 参数对象及全局变量
p = trackParamShare.param;
global iFrm ovlpRec trajectory

%% 删除回溯部分的旧记录
for iFrmTemp = p.backtrackFlag : structLength(ovlpRec, 'ovlp')
    ovlpRec(iFrmTemp).ovlp = struct('idxSet', []);
end

%% 重叠记录
for iFrmTemp = p.backtrackFlag : iFrm
    % 在iFrmTemp帧有记录的轨迹的索引
    idxTraj = find(arrayfun(@(x) ismember(iFrmTemp, x.frame), trajectory(iFrm).track));
    nTraj = length(idxTraj);
    if nTraj > 1
        % 轨迹数>1时, 进行重叠判定
        iPpl = []; % 各轨迹的iPeople
        pos = []; % 各轨迹的坐标
        for iTraj = 1 : length(idxTraj)
            iPpl = [iPpl, trajectory(iFrm).track(idxTraj(iTraj)).iPeople];
            pos = [pos; trajectory(iFrm).track(idxTraj(iTraj)).trajectory( ...
                trajectory(iFrm).track(idxTraj(iTraj)).frame == iFrmTemp, :)];
        end
        % 对轨迹进行排列组合
        combo = nchoosek(1 : nTraj, 2);
        % 计算距离
        d = sqrt((pos(combo(:, 1), 1) - pos(combo(:, 2), 1)) .^ 2 + (pos(combo(:, 1), 2) - pos(combo(:, 2), 2)) .^ 2);
        % 对距离小于阈值的轨迹组合进行重叠记录
        idxComboOvlp = find(d < p.distOvlp);
        for iidx = 1 : length(idxComboOvlp)
            recordOverlap(iFrmTemp, iPpl(combo(idxComboOvlp(iidx), :)));
        end
    end
end
