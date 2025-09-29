function [ptGhost, amplGhost] = multipath(vert, ct, pos)
% 多径反射模拟
% 输入:
% 1. vert: 网格顶点坐标
% 2. ct: 面质心坐标
% 3. pos: 人体坐标 目前在调用时用的是根关节点的坐标
% 4. faceIdx: 本体的LOS可见面在全部面中的索引 注意, 该输入只在启用双视角HPR处理时才使用
% 输出:
% 1. ptGhost: 鬼影点坐标
% 2. amplGhost: 鬼影点信号幅度
% 作者: 刘涵凯
% 更新: 2024-3-26

%% 参数对象
p = simParamShare.param;
load('mesh0.mat', 'faces')

%%
ptGhost = [];
amplGhost = [];
for iRfl = 1 : size(p.reflector, 2)
    %% 若反射点在反射面范围外, 或二阶鬼影-雷达距离比本体-雷达距离近, 则此反射面不参与多径计算
    [~, ghostBody2, ghostViewPoint] = specularReflection(p.reflector(iRfl).vert, pos, p.posRadar);
    [isInRfl, ~] = inpolygon3d(p.reflector(iRfl).vert, mean(ghostViewPoint, 1));
    if ~isInRfl || norm(ghostBody2 - p.posRadar) < norm(pos - p.posRadar); continue; end

    %% HPR与降采样
    % 不同视角能看到的面. 参与多径反射的面应同时被LOS径和NLOS径看到
    % [faceIdxNlos, rcsNlos] = hiddenRemoval(vert, ghostViewPoint, faces, ct); % HPR
    % idxNlosInLos = ismember(faceIdxNlos, faceIdx);
    % idxLosInNlos = ismember(faceIdx, faceIdxNlos);
    % faceIdxGhost = faceIdxNlos(idxNlosInLos);
    % rcsNlos = rcsNlos(idxNlosInLos);
    % rcsLos = rcs(idxLosInNlos);
    % rcsGhost = (rcsNlos + rcsLos) / 2;
    % faceIdxGhost = faceIdxNlos;
    % rcsGhost = rcsNlos;
    % 忘记了因为什么原因, 没有用上面的逻辑. 此时代码可以简化为:
    [faceIdxGhost, rcsGhost] = hiddenRemoval(vert, ghostViewPoint, faces, ct); % HPR
    [faceIdxGhost, rcsGhost] = faceDownsample(faceIdxGhost, rcsGhost, 'normal'); % 降采样
    ptDs = ct(faceIdxGhost, :); % 可见面质心坐标

    %% 鬼影点坐标
    difPos = ptDs - p.posRadar; % 本体-雷达坐标差
    dist = vecnorm(difPos, 2, 2); %本体-雷达距离
    [~, pt2, ~] = specularReflection(p.reflector(iRfl).vert, ptDs, p.posRadar); % 2阶鬼影坐标
    difPosGhost = pt2 - p.posRadar; % 2阶鬼影-雷达坐标差
    dist2 = vecnorm(difPosGhost, 2, 2); % 2阶鬼影-雷达距离
    dist1 = (dist + dist2) / 2; % 1阶鬼影-雷达距离
    pt11 = p.posRadar + (ptDs - p.posRadar) .* (dist1 ./ dist); % 1型1阶鬼影坐标
    pt12 = p.posRadar + (pt2 - p.posRadar) .* (dist1 ./ dist2); % 2型1阶鬼影坐标
    
    %% 信号幅度
    [ampl1, ampl2] = amplNlos(difPos, difPosGhost, dist1, dist2, rcsGhost, p.reflector(iRfl).atten); % 1阶和2阶鬼影的幅度
    % 这里额外加入了一个衰减, 依据是反射视点到人体的距离. 记不清为什么要这么设定了, 不深究
    distRflBody = norm(ghostViewPoint - pos, 2);
    lossRfl = (1 / (1 + distRflBody)) ^ sqrt(1.5);
    ampl1 = ampl1 * lossRfl;
    ampl2 = ampl2 * lossRfl ^ 2;

    %% 再进行一次降采样
    gridStep = 0.2;
    pt11Ds = pcDsFast(pt11, gridStep);
    pt12Ds = pcDsFast(pt12, gridStep);
    pt2Ds = pcDsFast(pt2, gridStep);
    ptGhostNew = [pt11Ds; pt12Ds; pt2Ds];
    % 降采样后, 不太好找对应的点, 所以这里使用一种部分随机的形式
    rand11 = ceil(linspace(1, size(pt11, 1), size(pt11Ds, 1)));
    rand12 = ceil(linspace(1, size(pt12, 1), size(pt12Ds, 1)));
    rand2 = ceil(linspace(1, size(pt2, 1), size(pt2Ds, 1)));
    ampl11Ds = ampl1(rand11) / sum(ampl1(rand11)) * sum(ampl1);
    ampl12Ds = ampl1(rand12) / sum(ampl1(rand12)) * sum(ampl1);
    ampl2Ds = ampl2(rand2) / sum(ampl2(rand2)) * sum(ampl2);
    amplGhostNew = [ampl11Ds; ampl12Ds; ampl2Ds];

    %% 将新鬼影点加入到全部鬼影点中
    ptGhost = [ptGhost; ptGhostNew];
    amplGhost = [amplGhost; amplGhostNew];
end

%% 删除较弱鬼影点
% idxWeak = amplGhost < p.ghostWeakRatio * max(amplGhost);
% ptGhost(idxWeak, :) = [];
% amplGhost(idxWeak) = [];

%% 限制鬼影点数量
if length(amplGhost) > p.ghostNumLmt 
    [~, idx] = sort(amplGhost, 'descend');
    idx = idx(1 : p.ghostNumLmt);
    ptGhost = ptGhost(idx, :);
    amplGhost = amplGhost(idx);
end
