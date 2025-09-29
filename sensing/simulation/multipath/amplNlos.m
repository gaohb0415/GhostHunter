function [ampl1, ampl2] = amplNlos(difPos, difPosGhost, dist1, dist2, rcs, atten)
% 鬼影路径幅度计算, 含天线增益、路径损耗
% 修改参数时, 记得一并修改amplLos中的参数
% 输入:
% 1. difPos: 本体-雷达坐标差
% 2. difPosGhost: 2阶鬼影-雷达坐标差
% 3. dist1: 1阶鬼影-雷达距离
% 4. dist2: 2阶鬼影-雷达距离
% 5. rcs: 本体各点反射强度
% 6. atten: 反射衰减系数
% 输出:
% ampl1: 1阶鬼影点反射信号幅度
% ampl2: 2阶鬼影点反射信号幅度
% 作者: 刘涵凯
% 更新: 2023-8-25

%% 参数对象
p = simParamShare.param;

%% 天线增益
% 角度
az = atan2d(difPos(:, 1), difPos(:, 2));
el = atan2d(difPos(:, 3), difPos(:, 2));
azGhost = atan2d(difPosGhost(:, 1), difPosGhost(:, 2));
elGhost = atan2d(difPosGhost(:, 3), difPosGhost(:, 2));
% 天线增益
gainAz = 50 * cosd(az) .^ 0.25 - 40;
gainEl = 50 * cosd(el) .^ 3.5 - 40;
gainGhostAz = 50 * cosd(azGhost) .^ 0.25 - 40;
gainGhostEl = 50 * cosd(elGhost) .^ 3.5 - 40;
% 增益相乘
gain = gainAz + gainEl;
gainGhost = gainGhostAz + gainGhostEl;
% 由dB转换为倍数
gain = 10 .^ (gain / 20);
gainGhost = 10 .^ (gainGhost / 20);

%% 路径损耗
pathLoss1 = (p.lambda ./ (2 * dist1)) .^ p.friisFactor; % 1阶鬼影
pathLoss2 = (p.lambda ./ (2 * dist2)) .^ p.friisFactor; % 2阶鬼影

%% 计算幅度
ampl1 = p.txAmpl * sqrt(gain .* gainGhost) .* pathLoss1 .* sqrt(rcs) * atten; % 1阶鬼影
ampl2 = p.txAmpl * gainGhost .* pathLoss2 .* sqrt(rcs) * atten .^ 1 * 3; % 2阶鬼影 实测二阶鬼影的衰减没有那么大 所以最后没有^2, 而且甚至还要补偿一个倍数

%% 以下代码用于观察增益设置是否拟真
% close all
% x = (-90 : 90) / 180 * pi;
% gainAz = 50 * (cos(x)) .^ 0.25 - 40;
% polarplot(x, gainAz)
% rlim([-40, 20])
% ax = gca;
% ax.RTick = -40 : 5 : 20;
% ax.ThetaDir = 'clockwise';
% ax.ThetaZeroLocation = 'top';
% hold on
% x = (-90 : 90) / 180 * pi;
% gainEl = 50 * (cos(x)) .^ 3.5 - 40;
% polarplot(x, gainEl)
% rlim([-40, 20])
% ax = gca;
% ax.RTick = -40 : 5 : 20;
% ax.ThetaDir = 'clockwise';
% ax.ThetaZeroLocation = 'top';
