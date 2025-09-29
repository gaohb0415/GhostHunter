function [vert, joint, vertDif, jointDif] = bodyMotion(traj, varargin)
% 将自定义移动轨迹赋予SMPL数据
% 输入:
% 1. traj: 预设轨迹
%     - .pos: 坐标
%     - .ori: 朝向
% 2. varargin:
%     - iSmpl: 选择的SMPL数据的编号
% 输出:
% 1. vert: 加入运动后的网格顶点坐标
% 2. joint: 加入运动后的关节点坐标
% 3. vertDif: 网格顶点坐标的每帧偏移
% 4. jointDif: 关节点坐标的每帧偏移
% 作者: 刘涵凯
% 更新: 2024-3-28

%% 默认参数
param = inputParser();
param.CaseSensitive = false;
param.addOptional('iSmpl', []);
param.parse(varargin{:});
iSmpl = param.Results.iSmpl;

%% 参数对象
p = simParamShare.param;

%% 轨迹补全
if size(traj.pos, 1) < p.nFrm + 1
    nExtra = p.nFrm - length(traj.ori) + 1;
    traj.pos(end + nExtra, :) = repmat(traj.pos(end, :), nExtra, 1) + 0.001 * rand(nExtra, 2);
    traj.ori(end + nExtra) = ones(nExtra, 1) * traj.ori(end) + 0.001 * rand(nExtra, 1);
end
traj.pos = traj.pos(1 : p.nFrm + 1, :);
traj.ori = traj.ori(1 : p.nFrm + 1);

%% 计算预设轨迹的累积位移
dist = vecnorm(traj.pos(2 : end, :) - traj.pos(1 : end - 1, :), 2, 2); % 位移
distDif = dist(2 : end) - dist(1 : end - 1); % 位移的帧间变化
% 重要的异常点滤除要滤三遍
distDif = hampel(distDif, 5, 2);
distDif = hampel(distDif, 5, 2);
distDif = hampel(distDif, 5, 2);
% 以某帧为基准, 重新计算位移
idxBase = 30;
dist = [flipud(cumsum([dist(idxBase); -distDif(idxBase - 1 : -1 : 1, :)])); cumsum([dist(idxBase); distDif(idxBase : end)])];
dist(idxBase) = [];
dist(dist < 0.02) = 0.02; % 维持位移下限
distCumTraj = [0; cumsum(dist)];

%% 选择最适配预设轨迹的SMPL数据
load distCum.mat
if isempty(iSmpl)
    distDif = distCum(:, p.nFrm) - distCumTraj(end);
    [~, iSmpl] = min(abs(distDif));
end

%% 提取SMPL数据
smplInfo = smplInfo(iSmpl);
smplInfo.iExtr = smplInfo.iExtr(1 : p.nFrm + 20); % 多提取一些, 以保证滤波正常进行
smplInfo.link(smplInfo.link > p.nFrm + 5) = []; % 同样多取一点
[vert, joint] = smplForMove([p.handleSMPL, 'walking\CMU\mesh_walking_CMU_', num2str(smplInfo.iModel), '.mat'], smplInfo);

%% 根据累积位移对预设轨迹进行插值
distCum = distCum(iSmpl, 1 : p.nFrm + 1);
distCum = distCum / max(distCum); % 归一化
distCumTraj = distCumTraj / max(distCumTraj); % 归一化
trajPos(:, 1) = interp1(distCumTraj, traj.pos(:, 1), distCum, 'spline');
trajPos(:, 2) = interp1(distCumTraj, traj.pos(:, 2), distCum, 'spline');
trajOri = interp1(distCumTraj, traj.ori, distCum, 'spline');

%% 修正过快的转向
oriDif = trajOri(2 : end) - trajOri(1 : end - 1);
while max(abs(oriDif)) > 0.1
    [~, idxMax] = max(abs(oriDif));
    % 将过快的朝向分摊到附近的帧中
    nFrmWin = 3; % 单边
    iWin = idxMax + (-nFrmWin : nFrmWin); 
    wt = ones(1, length(iWin));
    while iWin(1) < 1; iWin = iWin + 1; end
    while iWin(end) > p.nFrm + 1; iWin = iWin - 1; end
    wt([1, end]) = wt([1, end]) + 0.2 * (0.5 - rand(1)) * [1, -1]; % 给分摊窗的两端加一个随机量
    oriDif(iWin) = mean(oriDif(iWin)) * wt; % 分摊
    % 平滑一下轨迹坐标
    iLeft = iWin(1) + nFrmWin - 1;
    iRight = iWin(end) - nFrmWin + 1;
    if iRight >= iLeft
        trajPos(iLeft : iRight, :) = movmean(trajPos(iLeft - 2 : iRight + 2, :), 5, 1, 'Endpoints', 'discard');
    end
end
trajOri = trajOri(1) + cumsum([0, oriDif]); % 重新生成朝向
trajOri = smoothdata(trajOri, 'movmean', 5);

%% % 加入位移与转向
for iFrm = 1 : p.nFrm + 1
    vert(:, :, iFrm) = coordTransform2D(vert(:, :, iFrm), rad2deg(trajOri(iFrm)), trajPos(iFrm, :));
    joint(:, :, iFrm) = coordTransform2D(joint(:, :, iFrm), rad2deg(trajOri(iFrm)), trajPos(iFrm, :));
end

%% 计算帧间变化, 用于后续帧间插值
vertDif = vert(:, :, 2 : end) - vert(:, :, 1 : end - 1);
jointDif = joint(:, :, 2 : end) - joint(:, :, 1 : end - 1);
% 对连接的时隙进行帧间变化平滑
smplInfo.link = smplInfo.link(smplInfo.link <= p.nFrm);
smthTimeWinLink = floor(p.smthTimeWinLink * p.fFrm);
if mod(smthTimeWinLink, 2) == 0; smthTimeWinLink = smthTimeWinLink + 1; end % 将窗口帧数设定为奇数
smthFrmLink = ceil(p.smthTimeLink * p.fFrm);
for iLink = 1 : length(smplInfo.link)
    idxLinkFrm = smplInfo.link(iLink) + (-smthFrmLink : smthFrmLink); % 被平滑的帧
    idxLinkWin = idxLinkFrm(1) - floor(smthTimeWinLink / 2) : idxLinkFrm(end) + floor(smthTimeWinLink / 2); % 平滑窗内的帧
    if idxLinkWin(end) <= p.nFrm
        % 网格点处理
        vertSeg = movmean(vertDif(:, :, idxLinkWin), smthTimeWinLink, 3, 'Endpoints', 'discard');
        vertTail = vertDif(:, :, idxLinkFrm(end) : end);
        vertDif(:, :, idxLinkFrm) = vertSeg - vertSeg(:, :, 1) + vertDif(:, :, idxLinkFrm(1));
        vertDif(:, :, idxLinkFrm(end) : end) = vertTail - vertTail(:, :, 1) + vertDif(:, :, idxLinkFrm(end));
        % 关节点处理
        jointSeg = movmean(jointDif(:, :, idxLinkWin), smthTimeWinLink, 3, 'Endpoints', 'discard');
        jointTail = jointDif(:, :, idxLinkFrm(end) : end);
        jointDif(:, :, idxLinkFrm) = jointSeg - jointSeg(:, :, 1) + jointDif(:, :, idxLinkFrm(1));
        jointDif(:, :, idxLinkFrm(end) : end) = jointTail - jointTail(:, :, 1) + jointDif(:, :, idxLinkFrm(end));
    else % 如果平滑窗超出了帧限制, 就简单平滑一下
        idxLinkFrm = idxLinkFrm(idxLinkFrm <= p.nFrm);
        idxLinkWin = idxLinkWin(idxLinkWin <= p.nFrm);
        vertDif = reshape(vertDif, [6890 * 3, numel(vertDif) / (6890 * 3)])';
        vertDif(idxLinkFrm, :) = smoothdataV3(vertDif(idxLinkFrm, :), 1, 'movmean', smthTimeWinLink);
        vertDif = reshape(vertDif', [6890, 3, numel(vertDif) / (6890 * 3)]);
        jointDif = reshape(jointDif, [52 * 3, numel(jointDif) / (52 * 3)])';
        jointDif(idxLinkFrm, :) = smoothdataV3(jointDif(idxLinkFrm, :), 1, 'movmean', smthTimeWinLink);
        jointDif = reshape(jointDif', [52, 3, numel(jointDif) / (52 * 3)]);
    end
end
