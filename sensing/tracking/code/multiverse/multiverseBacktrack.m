function multiverseBacktrack(frame)
% 平行宇宙回溯
% 输入: 
% frame: 回溯起始帧
% 作者: 刘涵凯
% 更新: 2023-3-13

%% 参数对象及全局变量
p = trackParamShare.param;
global multivRec

%% 删除回溯起始帧后的平行宇宙
for iFrm = frame : structLength(multivRec, 'multiv')
    multivRec(iFrm).multiv = struct('iMultiverse', 1, 'track', struct('iPeople', [], 'name', [], ...
        'trajectory', [], 'frame', [], 'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []), ...
        'brother', [], 'parent', [], 'association', []);
end

%% 更新参数
p.anchorFrmMultiv = frame - 1;
p.identLockFrm = min(frame - 1, p.identLockFrm);
p.nMultiv = max(vertcat(multivRec(max(1, frame - 1)).multiv.iMultiverse));
% 将回溯标记和分离标记置为0, 因为它们已经完成了它们的使命
p.backtrackFlag = 0;
p.sepDelFlag = 0;
