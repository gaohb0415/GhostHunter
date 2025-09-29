function sigRA = dbfRecons(radarData, RAPair)
% 1D/2D DBF信号重构
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC/Range, Chirp, Rx, Tx]
% 2. RAPair: 待重构距离-角度对, N*2(距离-水平角)或N*3(距离-水平角-俯仰角)矩阵, 其中距离单位为在第一维度的索引, 角度单位为°
% 输出:
% sigRA: 重构信号, [Chirp, RAPair], 其中RAPair顺序与输入相同
% 作者: 刘涵凯
% 更新: 2024-6-20

%% 雷达参数
load('config.mat', 'spacingCal')

%% 生成虚拟阵列
[nPair, nDim] = size(RAPair);
switch nDim
    case 2
        antArray = virtualArray1D(radarData, 'DBF');
    case 3
        antArray = virtualArray2D(radarData, 'DBF');
end

%% RAPair预处理
rgIdx = unique(RAPair(:, 1)); % 出现在RAPair中的距离的索引
sigRA= zeros(size(radarData, 2), nPair); % 初始化重构信号矩阵

%% 信号重构
for iRg = 1 : length(rgIdx)
    iPair = find(RAPair(:, 1) == rgIdx(iRg)); % 该距离上的Pair索引
    % 对该距离上的Pair进行信号重构
    switch nDim
        case 2
            [~, sigRA(:, iPair)] = dbf(RAPair(iPair, 2), [], antArray.signal(:, :, rgIdx(iRg)), ...
                antArray.arrayPos, [], 'spacingCal', spacingCal, 'pwEn', 0, 'sigReconsEn', 1);
        case 3
            [~, sigRA(:, iPair)] = dbf(RAPair(iPair, 2), RAPair(iPair, 3), antArray.signal(:, :, rgIdx(iRg)), ...
                antArray.arrayPosX, antArray.arrayPosZ, 'spacingCal', spacingCal, 'pwEn', 0, 'sigReconsEn', 1);
    end
end
