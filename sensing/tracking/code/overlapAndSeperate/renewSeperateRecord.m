function renewSeperateRecord(frame, backtrackFlag)
% 更新轨迹分离记录
% 重叠记录
% 输入: 
% 1. frame: 待更新的帧
% 2. backtrackFlag: 回溯标记. 0-不回溯; 非0-重新记录从标记点开始的分离记录
% 作者: 刘涵凯
% 更新: 2023-3-12

%% 参数对象及全局变量
p = trackParamShare.param;
global ovlpRec sepRec

%% 当发生回溯时, 以递归的方式重新记录轨迹分离
if p.backtrackEn && backtrackFlag
    % 对有关轨迹ID的变量进行ID回溯
    backtrack(backtrackFlag)
    % 对重叠记录进行回溯
    overlapBacktrack
    % 删除回溯部分的旧轨迹分离记录
    for iFrm = backtrackFlag : structLength(sepRec, 'sep')
        sepRec(iFrm).sep = struct('idxSet', [], 'nSeperate', []);
    end
    % 重新记录轨迹分离
    for iFrmTemp = max(2, backtrackFlag) : frame
        renewSeperateRecord(iFrmTemp, 0);
    end
else

    %% 删除冗余轨迹分离记录
    % 上一帧(Old)和本帧(New)的重叠记录
    ovlpRecOld = ovlpRec(frame - 1).ovlp;
    ovlpRecNew = ovlpRec(frame).ovlp;
    nOvlpOld = structLength(ovlpRecOld, 'idxSet');
    nOverlapNew = structLength(ovlpRecNew, 'idxSet');
    % 若发现已记录的分离轨迹在空闲窗内又重叠, 则删除该分离记录
    for iFrm = max(1, frame - p.sepIdleWin) : frame - 1
        iDel = [];
        for iSep = 1 : structLength(sepRec(iFrm), 'sep')
            % 该条分离记录的轨迹数
            lenIdxSet = length(sepRec(iFrm).sep(iSep).idxSet);
            isOvlp = zeros(lenIdxSet, 1);
            for iTrack = 1 : lenIdxSet
                % 观察分离轨迹是否重新重叠
                isOvlp(iTrack) = isSubMemberOfStruct(ovlpRecNew, 'idxSet', sepRec(iFrm).sep(iSep).idxSet(iTrack));
            end
            isOvlpUniq = unique(isOvlp);
            if any(isOvlpUniq) && length(isOvlpUniq) == 1
                % 1. isOvlpUniq非零, 表示分离的轨迹又参与了某次重叠(但不一定是重叠在一起)
                % 2. isOvlpUniq为1, 表示分离的轨迹重叠在了一起
                iDel = [iDel, iSep];
                % 以删除分离记录的帧作为分离标记
                if ~p.sepDelFlag || p.sepDelFlag > iFrm
                    p.sepDelFlag = iFrm;
                end
            end
        end
        % 删除分离记录
        sepRec(iFrm).sep = structRowDelete(sepRec(iFrm).sep, iDel);
    end

    %% 新增轨迹分离记录
    % 对重叠记录的各idxSet进行排序, 使其顺序整齐
    for iOverlap = 1 : nOvlpOld
        ovlpRecOld(iOverlap).idxSet = sort(ovlpRecOld(iOverlap).idxSet);
    end
    for iOverlap = 1 : nOverlapNew
        ovlpRecNew(iOverlap).idxSet = sort(ovlpRecNew(iOverlap).idxSet);
    end
    % 观察旧重叠记录中的各idxSet是否在新重叠纪录中依然存在
    for iOverlap = 1 : nOvlpOld
        findRslt = isSubMemberOfStruct(ovlpRecNew, 'idxSet', ovlpRecOld(iOverlap).idxSet);
        if ~any(findRslt)
            % 若不存在, 则说明重叠的轨迹发生了分离
            % 需要根据人数考虑分离情况, 这里暂时只考虑2人和3人
            % 2人分离
            if length(ovlpRecOld(iOverlap).idxSet) == 2
                nRecOld = structLength(sepRec(frame).sep, 'idxSet');
                % 记录分离轨迹的ID和分离股数
                sepRec(frame).sep(nRecOld + 1).idxSet = ovlpRecOld(iOverlap).idxSet;
                sepRec(frame).sep(nRecOld + 1).nSeperate = 2;
            end
            % 3人分离
            if length(ovlpRecOld(iOverlap).idxSet) == 3
                nRecOld = structLength(sepRec(frame).sep, 'idxSet');
                % 记录分离轨迹的ID
                sepRec(frame).sep(nRecOld + 1).idxSet = ovlpRecOld(iOverlap).idxSet;
                % 需要确认是分为3股还是分为2股
                % 旧的共同轨迹可能在分离的瞬间又与其他轨迹重合, 所以需要特殊判断
                idx1 = isSubMemberOfStruct(ovlpRecNew, 'idxSet', ovlpRecOld(iOverlap).idxSet(1));
                idx2 = isSubMemberOfStruct(ovlpRecNew, 'idxSet', ovlpRecOld(iOverlap).idxSet(2));
                idx3 = isSubMemberOfStruct(ovlpRecNew, 'idxSet', ovlpRecOld(iOverlap).idxSet(3));
                if idx1 == idx2 || idx1 == idx3 || idx2 == idx3
                    % 分为2股
                    sepRec(frame).sep(nRecOld + 1).nSeperate = 2;
                else
                    % 分为3股
                    sepRec(frame).sep(nRecOld + 1).nSeperate = 3;
                end
            end
        end
    end
end
