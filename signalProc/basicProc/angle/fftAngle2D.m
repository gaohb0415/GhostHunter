function [fftRsltAng2D, heatmapAE] = fftAngle2D(radarData, varargin)
% 2D Angle FFT, 含2D Angle Heatmap生成
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC/Range, Chirp/Velocity, Rx, Tx]
% 2. varargin:
%     - windowEn: FFT时是否加窗. 0-否; 1-是
%     - limitR: 距离范围. []-不设范围
%     - limitAz: 水平角范围
%     - limitEl: 俯仰角范围
%     - nAz: 水平角FFT点数
%     - nEl: 俯仰角FFT点数
%     - drawEn: 是否绘图. 0-否; 1-是
% 输出:
% 1. fftRsltAng2D: 雷达数据矩阵, [ADC/Range, Chirp/Velocity, Azimuth, Elevation]
% 2. heatmapAE: Azimuth-Elevation heatmap
% 作者: 刘涵凯
% 更新: 2022-7-5

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('windowEn', 0);
p.addOptional('limitR', []);
p.addOptional('limitAz', [-30, 30]);
p.addOptional('limitEl', [-30, 30]);
p.addOptional('nAz', 128);
p.addOptional('nEl', 64);
p.addOptional('drawEn', 0);
p.parse(varargin{:});
windowEn = p.Results.windowEn;
limitR = p.Results.limitR;
limitAz = p.Results.limitAz;
limitEl = p.Results.limitEl;
nAz = p.Results.nAz;
nEl = p.Results.nEl;
drawEn = p.Results.drawEn;

%% 雷达参数
load('config.mat', 'resR', 'spacingCal')
nRg = size(radarData, 1);
rg = resR * (0 : nRg - 1)'; % 总距离刻度

%% 提取所选距离范围内的数据
if any(limitR)
    aoiRg = [max(limitR(1), rg(1)), min(limitR(2), rg(end))];  % 距离AOI(area of interest)
    iAoiRg = rg >= aoiRg(1) & rg <= aoiRg(2);
    radarData = radarData(iAoiRg, :, :, :);
end

%% 生成虚拟阵列
antArray = virtualArray2D(radarData, 'FFT');

%% 水平角 Angle FFT
if windowEn % 加窗(汉宁窗)
    [nRg, nChirp, nAntAz, nAntEl] = size(antArray.signal); % 获取数据矩阵尺寸
    antArray.signal = antArray.signal .* permute(repmat(hanning(nAntAz), [1, nRg, nChirp, nAntEl]), [2, 3, 1, 4]);
end
fftRsltAng1D = fftshift(fft(antArray.signal, nAz, 3), 3);
fftRsltAng1D = flip(fftRsltAng1D, 3); % 角度翻转

%% 垂直角 Angle FFT
if windowEn % 加窗(汉宁窗)
    [nRg, nChirp, nAntAz, nAntEl] = size(fftRsltAng1D); % 获取数据矩阵尺寸
    fftRsltAng1D = fftRsltAng1D .* permute(repmat(hanning(nAntEl), [1, nRg, nChirp, nAntAz]), [2, 3, 4, 1]);
end
fftRsltAng2D = fftshift(fft(fftRsltAng1D, nEl, 4), 4);
fftRsltAng2D = flip(fftRsltAng2D, 4); % 角度翻转

%% Heatmap
pwRAE = matExtract(abs(fftRsltAng2D), [1, 3, 4], [0, 0]); % 提取结果矩阵
heatmapAE = squeeze(max(pwRAE, [], 1)); % 各角度沿距离维度取最大值
az = asind((0 : nAz - 1) / (nAz / 2) - 1)'; % 水平角坐标
az = ([az(2 : end); 90] + az) / 2; % 以每个angle bin的中心作为其刻度
az = az / spacingCal; % 天线间距校准
el = asind((0 : nEl - 1) / (nEl / 2) - 1)'; % 俯仰角坐标
el = ([el(2 : end); 90] + el) / 2; % 以每个angle bin的中心作为其刻度
el = el / spacingCal; % 天线间距校准

%% 绘图
if drawEn; drawAEHeatmap(heatmapAE, az, el, 'limitAz', limitAz, 'limitEl', limitEl); end
