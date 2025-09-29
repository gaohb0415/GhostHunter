function backtrack(frame, varargin)
% 对有关轨迹ID的变量进行ID回溯
% 输入: 
% 1. frame: 回溯帧
% 2. varargin
%     -. trajectoryFrame: 决定对trajectory的哪一帧进行回溯. 0-当前帧; 1-上一帧
% 作者: 刘涵凯
% 更新: 2023-3-18

p = inputParser();
p.CaseSensitive = false;
p.addOptional('trajectoryFrame', 0);
p.parse(varargin{:});
trajectoryFrame = p.Results.trajectoryFrame;

%% 全局变量
global iFrm ovlpRec sepRec assocRec trackConfirm trackLost trackWait trajectory

%% 回溯关联表
% 提取当前关联表
assocLast = assocRec(end).association;
% 删除回溯部分的关联表记录
iDel = find(arrayfun(@(x) x.frame >= max(2, frame), assocRec));
assocRec = structRowDelete(assocRec, iDel);
% 提取回溯点关联表
assocBacktrack = assocRec(end).association;
%% 回溯ID
if any(assocLast - assocBacktrack)
    % 回溯轨迹iPeople
    % 对于轨迹记录结构体, 仅更新其最后一帧(对于等待区的该函数调用而言, 需更新的是当前帧的上一帧)
    trajFrm = iFrm + trajectoryFrame;
    for iTraj = 1 : structLength(trajectory(trajFrm).track, 'iPeople')
        trajectory(trajFrm).track(iTraj).iPeople = assocBacktrack(assocLast == trajectory(trajFrm).track(iTraj).iPeople);
    end
    % 回溯确立区iPeople
    for iTraj = 1 : structLength(trackConfirm, 'iPeople')
        trackConfirm(iTraj).iPeople = assocBacktrack(assocLast == trackConfirm(iTraj).iPeople);
    end
    % 回溯丢失区iPeople
    for iTraj = 1 : structLength(trackLost, 'iPeople')
        trackLost(iTraj).iPeople = assocBacktrack(assocLast == trackLost(iTraj).iPeople);
    end
    % 回溯等待区iPeople
    for iTraj = 1 : structLength(trackWait, 'iPeople')
        trackWait(iTraj).iPeople = assocBacktrack(assocLast == trackWait(iTraj).iPeople);
    end
    % 回溯重叠记录和分离记录中的索引
    for iiFrm = 1 : frame - 1
        for iOvlp = 1 : structLength(ovlpRec(iiFrm), 'ovlp')
            [~, iAssoc] = ismember(ovlpRec(iiFrm).ovlp(iOvlp).idxSet, assocLast);
            ovlpRec(iiFrm).ovlp(iOvlp).idxSet = assocBacktrack(iAssoc);
        end
        for iSep = 1 : structLength(sepRec(iiFrm), 'sep')
            [~, iAssoc] = ismember(sepRec(iiFrm).sep(iSep).idxSet, assocLast);
            sepRec(iiFrm).sep(iSep).idxSet = assocBacktrack(iAssoc);
        end
    end
end
