function [ori, idxEff] = trajOrientation(traj, varargin)
% 由轨迹计算朝向(右手螺旋)
% 输入:
% 1. traj: 轨迹
% 2. varargin:
%     - dispTh: 对dispIntvl帧间位移大于dispTh m的部分进行匹配
%     - dispIntvl: 
%     - effWin: 有效区间筛选窗长度, 帧
%     - effWinSlide: 有效区间筛选窗滑动距离, 帧
%     - effWinGuard: 有效区间筛选保护时间, 帧
%     - effRatio: 比例阈值
%     - smoothWin: 平滑窗口宽度, 帧
% 输出:
% 1. ori: unwrap后的朝向(右手螺旋)
% 2. idxEff: 有效帧的索引
% 作者: 刘涵凯
% 更新: 2023-11-22

%% 默认参数
trackParam = trackParamConfig; % 默认采用追踪代码所用参数, 为方便调用, 将此处更改为了参数初始化形式
load('config.mat', 'fFrm')
p = inputParser();
p.CaseSensitive = false;
p.addOptional('dispTh', trackParam.dispTh);
p.addOptional('dispIntvl', trackParam.dispIntvl);           
p.addOptional('effWin', trackParam.effWin);
p.addOptional('effWinSlide', trackParam.effWinSlide);
p.addOptional('effWinGuard', trackParam.effWinGuard);
p.addOptional('effRatio', trackParam.effRatio);
p.addOptional('smoothWin', fFrm);
p.parse(varargin{:});
dispTh = p.Results.dispTh;
dispIntvl = p.Results.dispIntvl;
effWin = p.Results.effWin;
effWinSlide = p.Results.effWinSlide;
effWinGuard = p.Results.effWinGuard;
effRatio = p.Results.effRatio;
smoothWin = p.Results.smoothWin;

%% 计算朝向
nFrm = size(traj, 1);
trans = traj(dispIntvl + 1 : end, :) - traj(1 : end - dispIntvl, :);
disp = sqrt(trans(:, 1) .^ 2 + trans(:, 2) .^ 2);
middleFrm = (dispIntvl + 1 : nFrm)' - round(dispIntvl / 2); % 用计算速度的帧窗的中间作为该速度的帧索引
ori = atan2(trans(:, 2), trans(:, 1));
ori = smoothdata(unwrap(ori), 'movmean', smoothWin); % 平滑

%% 筛选有效区间
% 初步筛选
idxEff = disp > dispTh;
% 以"有效帧在窗口中的占比"进行进一步筛选
if any(idxEff)
    iDel = [];
    for iWin = 1 : effWinSlide : length(idxEff) - 1
        iEffWin = iWin : min(length(idxEff), iWin + effWin);
        effTemp = idxEff(iEffWin);
        if length(effTemp) > effWinGuard
            if ~any([effTemp(1 : effWinGuard); effTemp(end - length(effTemp) + 1 : end)]) && ...
                    sum(effTemp) / length(effTemp) < effRatio
                % 若窗口的首尾保护区无有效点, 且窗口内的有效点比例小于阈值, 将该窗口设为无效
                iDel = [iDel, iEffWin];
            end
        end
    end
    % 对整个数据的首尾进一步筛选有效时间, 即滑动窗首尾保护区不保护整体的首尾
    if length(idxEff) > effWinGuard
        % 首
        iEffWin = 1 : min(length(idxEff), effWin);
        effTemp = idxEff(iEffWin);
        if sum(effTemp) / length(effTemp) < effRatio
            iDel = [iDel, iEffWin];
        end
        % 尾
        iEffWin = length(idxEff) - min(length(effTemp), effWin) + 1 : length(idxEff);
        effTemp = idxEff(iEffWin);
        if sum(effTemp) / length(effTemp) < effRatio
            iDel = [iDel, iEffWin];
        end
    end
    idxEff(iDel) = 0;
end

%% 移动与静止的帧索引
idxEff = find(idxEff);
% 移动索引
iFrmFast = middleFrm(idxEff);

%% 对静止区间进行插值
if isempty(idxEff)
    ori = interp1([0; middleFrm; nFrm + 1], [ori(1); ori; ori(end)], (1 : nFrm)');
else
    ori = interp1([0; iFrmFast; nFrm + 1], [ori(idxEff(1)); ori(idxEff); ori(idxEff(end))], (1 : nFrm)');
end
