function pt = ptChirpInterp(ptNow, ptNext)
% 一帧中各chirp时刻的反射点坐标插值
% 输入:
% 1. ptNow: 本帧反射点坐标
% 2. ptNext: 下一帧反射点坐标
% 输出:
% pt: 插值后的反射点坐标, [点索引, 点坐标, chirp索引]
% 作者: 刘涵凯
% 更新: 2023-6-28

%% 参数对象
p = simParamShare.param;

%% 插值
nVert = size(ptNow, 1);
nCor = nVert * 3; % 坐标数
[X, Y] = meshgrid(1 : p.nTransmit, 1 : nCor); % 此处的XY无具体含义
idxNext = p.tFrm / p.tChirp; % 下一帧的第一个chirp相当于本帧的第idxNext个chirp
pt = reshape(interp2([ones(nCor, 1), idxNext * ones(nCor, 1)], [1 : nCor; 1 : nCor]', ...
    [ptNow(:), ptNext(:)], X, Y), [nVert, 3, p.nTransmit]);
