function recordOverlap(frame, idx)
% 重叠记录
% 输入: 
% 1. frame: 要记录的帧的索引
% 2. idx: 待记录的2条ID
% 作者: 刘涵凯
% 更新: 2023-3-9

% 全局变量
global ovlpRec

% 检查待记录ID是否已记录在案
iSet1 = find(arrayfun(@(x) ismember(idx(1), x.idxSet), ovlpRec(frame).ovlp));
iSet2 = find(arrayfun(@(x) ismember(idx(2), x.idxSet), ovlpRec(frame).ovlp));

if isempty(iSet1) && isempty(iSet2)
    % 若两条轨迹都尚未被记录, 则创建新记录
    nOldRec = structLength(ovlpRec(frame).ovlp, 'idxSet');
    ovlpRec(frame).ovlp(nOldRec + 1).idxSet = idx;
elseif ~isempty(iSet1) && isempty(iSet2)
    % 若轨迹1已被记录, 轨迹2尚未被记录, 则在含轨迹1的记录中加入轨迹2
    ovlpRec(frame).ovlp(iSet1).idxSet = [ovlpRec(frame).ovlp(iSet1).idxSet, idx(2)];
elseif isempty(iSet1) && ~isempty(iSet2)
    % 若轨迹2已被记录, 轨迹1尚未被记录, 则在含轨迹2的记录中加入轨迹1
    ovlpRec(frame).ovlp(iSet2).idxSet = [ovlpRec(frame).ovlp(iSet2).idxSet, idx(1)];
end
