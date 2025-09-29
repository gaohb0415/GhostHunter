function ampl = amplLos(pt, rcs)
% 直视路径幅度计算, 含天线增益、路径损耗
% 修改参数时, 记得一并修改amplNlos中的参数
% 输入:
% 1. pt: 反射点坐标
% 2. rcs: 各点反射强度
% 输出:
% ampl: 各点反射信号幅度
% 作者: 刘涵凯
% 更新: 2023-6-28

%% 参数对象
p = simParamShare.param;

%% 天线增益
% 距离与角度
difPos = pt - p.posRadar;
dist = sqrt(sum(difPos .^2, 2));
az = atan2d(difPos(:, 1), difPos(:, 2));
el = atan2d(difPos(:, 3), difPos(:, 2));
% 天线增益
gainAz = 50 * cos(az) .^ 0.25 - 40;
gainEl = 50 * cos(el) .^ 3.5 - 40;
% 增益相乘
gain = gainAz + gainEl;
% 由dB转换为倍数
gain = 10 .^ (gain / 20);

%% 路径损耗
pathLoss = (p.lambda ./ (2 * dist)) .^ p.friisFactor;

%% 计算幅度
ampl = p.txAmpl * gain .* pathLoss .* sqrt(rcs); % 注意这里的gain后面没有.^2, 因为gain实际上要先开根号

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
