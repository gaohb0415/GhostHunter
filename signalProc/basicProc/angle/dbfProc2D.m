function [pwRAE, heatmapAE] = dbfProc2D(radarData, varargin)% 注意参数2、3、4为FOI的范围, 不是DBF或CFAR的范围
% 2D DBF处理, 含2D Angle Heatmap生成
% 输入: 
% 1. radarData: 雷达数据矩阵, [ADC/Range, Chirp, Rx, Tx]
% 2. varargin:
%     - limitR: 距离范围. []-不设范围
%     - limitAz: 水平角范围
%     - limitEl: 俯仰角范围
%     - resAz: 水平角步进间隔
%     - resEl: 俯仰角步进间隔
%     - heatmapEn: 是否绘制2D角度heatmap. 0-否; 1-是
% 输出: 
% 1. pwRAE: 反射强度矩阵, [ADC/Range, Azimuth, Elevation]
% 2. heatmapAE: Azimuth-Elevation heatmap
% 作者: 刘涵凯
% 更新: 2022-8-1

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('limitR', []);
p.addOptional('limitAz', [-30, 30]);
p.addOptional('limitEl', [-30, 30]);
p.addOptional('resAz', 1);
p.addOptional('resEl', 1);
p.addOptional('heatmapEn', 1);
p.parse(varargin{:});
limitR = p.Results.limitR;
limitAz = p.Results.limitAz;
limitEl = p.Results.limitEl;
resAz = p.Results.resAz;
resEl = p.Results.resEl;
heatmapEn = p.Results.heatmapEn;

%% 雷达参数
load('config.mat', 'resR', 'spacingCal')
nRg = size(radarData, 1);

%% 提取所选距离范围内的数据
if any(limitR)
    rg = resR * (0 : nRg - 1); % 总距离刻度
    aoiRg = [max(limitR(1), rg(1)), min(limitR(2), rg(end))];  % 距离AOI(area of interest)
    iAoiRg = rg >= aoiRg(1) & rg <= aoiRg(2);
    radarData = radarData(iAoiRg, :, :, :);
end

%% 生成虚拟阵列
antArray = virtualArray2D(radarData, 'DBF');

%% 获取数据尺寸与刻度
nRg = size(antArray.signal, 3);
az = (max(limitAz(1), -90) : resAz : min(limitAz(end), 90))'; % 扫描角度-水平角
el = (max(limitEl(1), -90) : resEl : min(limitEl(end), 90))'; % 扫描角度-俯仰角
nAz = length(az);
nEl = length(el);
nAngPair = nAz * nEl; % 角度组合数
angPairAz = repelem(az, nEl, 1); % 所有角度组合中的水平角
angPairEl = repmat(el, [nAz, 1]); % 所有角度组合中的俯仰角

%% 执行2D DBF
% 初始化反射强度和权重结果
pwRAE = zeros(nAngPair * nRg, 1);
% 提取各距离的信号进行DBF计算
tic
for iRg = 1 : nRg
    % 将[nAnglePair, nRange]矩阵"拉伸"成[nAnglePair * nRange]向量后, 第iRange个距离的角度在该向量中的索引
    iBin = (iRg - 1) * nAngPair + 1 : iRg * nAngPair;
    % 执行DBF, 获得反射强度
    [pwRAE(iBin), ~] = dbf(angPairAz, angPairEl, antArray.signal(:, :, iRg), ...
        antArray.arrayPosX, antArray.arrayPosZ, 'spacingCal', spacingCal);
end
toc
% 注意下面的维度顺序
pwRAE = reshape(pwRAE, [nEl, nAz, nRg]);
pwRAE = permute(pwRAE, [3, 2, 1]); % 将维度转换为为[Range, Az, El]

%% Heatmap
heatmapAE = [];
if heatmapEn
    heatmapAE = permute(max(pwRAE, [], 1), [2, 3, 1]); % 各角度沿距离维度取最大值
    % 绘图
    drawAEHeatmap(heatmapAE, az, el, 'limitAz', limitAz, 'limitEl', limitEl);
end
