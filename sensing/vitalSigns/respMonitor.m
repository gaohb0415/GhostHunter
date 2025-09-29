function respMonitor(varargin)
% 呼吸监测
% 输入:
% varargin:
% - dataType: 数据类型. 'real'; 'sim'
% - dataInfo: 仿真数据信息
%   * .handle: 地址
%   * .frmMode: 存储模式. 'allFrm'; '1Frm'
% - locMode: 定位及信号提取模式. 'range'; 'RA'
% 作者: 刘涵凯
% 更新: 2024-3-29

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('dataType', 'real');
p.addOptional('dataInfo', []);
p.addOptional('locMode', 'RA');
p.parse(varargin{:});
dataType = p.Results.dataType;
dataInfo = p.Results.dataInfo;
locMode = p.Results.locMode;

%% 载入配置
% config22431T1R
config2243mmFilter
load('config.mat', 'nChirp1Frm', 'tFrm', 'nFrm', 'resR', 'lambda')

%% 参数设置
limitR = [0.2, 3]; % 距离限制
fUpLmtResp = 30 / 60; % 呼吸频率上限, 用于波峰搜索限制 Hz
peakWidth = 1; % 频谱尖峰单边宽度, 单位为"1个频率bin", 用于计算置信度
nDifIntvl = 100; % 差分间隔 % 1
facWin = 5; % 窗系数
smthWinIQ = 5 * facWin; % IQ平滑窗
difTh = [0.5, 0.05, 0.02] * nDifIntvl; % 波动代替阈值
smthWinCurve = nChirp1Frm / tFrm * 0.1 * facWin; % 曲线平滑窗
movWinCurve = nChirp1Frm / tFrm * facWin; % movemean窗
dsIntvl = 10; % 降采样间隔

%% 帧选取与时间设置
limitT = [0, 30];
iFrmStart = max(1, floor(limitT(1) / tFrm)); % 起始帧
iFrmEnd = min(nFrm, ceil(limitT(2) / tFrm) + 1); % 结束帧
iFrmLoad = iFrmStart : iFrmEnd; % 选取的帧
nFrmLoad = length(iFrmLoad); % 帧数
fFrmFin = (nChirp1Frm + 1)  / dsIntvl / tFrm; % 最终呈现出的帧频, 即呼吸数据的采样频率
axisT = linspace(0, nFrmLoad, fFrmFin * nFrmLoad + 1); axisT(end - 1 : end) = []; % 以fFrmFin为频率的时间轴
axisTChirp = linspace(0, nFrmLoad, (nChirp1Frm + 1) * nFrmLoad + 1); axisTChirp(end - 1 : end) = []; % 以chirp频率为频率的时间轴
axisTChirpActual = axisTChirp; axisTChirpActual((nChirp1Frm + 1) : (nChirp1Frm + 1) : end) = []; % 实际的chirp时间轴(在axisTChirp中删除了帧间空白)

%% 获取点云
radarDataTemp = readBin(iFrmLoad(1), 0, dataType = dataType, dataInfo = dataInfo); % 取第一帧用于获得点云
switch locMode
    case 'range'
        [~, pcRg] = fftRange(radarDataTemp, 'pcEn', 1, 'drawEn', 0); drawnow; % Range FFT并获得点云
        pcRg.iRange((pcRg.range < limitR(1)) | (pcRg.range > limitR(2))) =[]; % 删除超出范围的点云
        nPc = length(pcRg.iRange); % 点云数
    case 'RA'
        [fftRsltRg, ~] = fftRange(radarDataTemp, 'cfarEn', 0, 'drawEn', 0); % Range FFT
        [~, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', limitR, 'resAng', 1, 'drawEn', 0); % 1D DBF
        nPc = length(pcRA.iRange); % 点云数
end

%% IQ提取
iqActual = zeros(nPc, nChirp1Frm * nFrmLoad);
for iFrm = 1 : nFrmLoad
    radarDataTemp = readBin(iFrmLoad(iFrm), 0, dataType = dataType, dataInfo = dataInfo);
    switch locMode
        case 'range'
            radarDataTemp = radarDataTemp(:, :, 1, 1);
            [fftRsltRg, ~] = fftRange(radarDataTemp);
            iqActual(:, (iFrm - 1) * nChirp1Frm + (1 : nChirp1Frm)) = fftRsltRg(pcRg.iRange, :);
        case 'RA'
            [fftRsltRg, ~] = fftRange(radarDataTemp);
            iqActual(:, (iFrm - 1) * nChirp1Frm + (1 : nChirp1Frm)) = dbfRecons(fftRsltRg, [pcRA.iRange, pcRA.angle']).';
    end
end

%% IQ空隙插值
iq = zeros(nPc, (nChirp1Frm + 1) * nFrmLoad - 1);
for iPc = 1 : nPc; iq(iPc, :) = interp1(axisTChirpActual, iqActual(iPc, :), axisTChirp); end

%% IQ平滑
iqSmth = zeros(size(iq));
for iPc = 1 : nPc; iqSmth(iPc, :) = smooth(iq(iPc, :), smthWinIQ); end
close all; for i = 10 : 11; polarplot(iq(2, (i - 1) * nChirp1Frm + 1  : i * nChirp1Frm)); pause(0.5); hold on; end % 这句可以看一下IQ圆

%% IQ差分
iqDif = zeros(size(iqSmth));
iqDif(:, nDifIntvl + 1 : end) = iqSmth(:, nDifIntvl + 1 : end) - iqSmth(:, 1 : end - nDifIntvl);
iqDif(:, 1 : nDifIntvl) = repmat(iqDif(:, nDifIntvl + 1), 1, nDifIntvl);

%% 计算相位并Unwrap
phRaw = unwrap(angle(iq), [], 2); % 坐标相位
phRaw = unwrap(2 * phRaw, [], 2) / 2;
phTan = unwrap(angle(iqDif), [], 2); % 切线相位
phTan = unwrap(2 * phTan, [], 2) / 2;

%% 相位差分
phDifRaw = phRaw(:, 2 : end) - phRaw(:, 1 : end - 1);
phDifTan = phTan(:, 2 : end) - phTan(:, 1 : end - 1);

%% tan波动剧烈时, 用raw相位代替
% for iPc = 1 : nPc
%     % 波动绝对值
%     difRawAbs = abs(phDifRaw(iPc, :));
%     difTanAbs = abs(phDifTan(iPc, :));
%     % 判断条件
%     idx1 = (difTanAbs > difTh(1));
%     idx2 = ((difTanAbs > difTh(2)) & (difRawAbs < difTh(3)));
%     idx = find(idx1 | idx2);
%     % 差值代替
%     for i = 1 : length(idx)
%         phTan(iPc, idx(i) + 1 : end) = phTan(iPc, idx(i) + 1 : end) - phDifTan(idx(i)) + phDifRaw(idx(i));
%     end
% end
ph = phTan;

%% 曲线平滑, 去趋势, 降采样
ph = smoothdata(ph, 2, "movmean", smthWinCurve);
ph = ph - movmean(ph, movWinCurve, 2);
ph = ph(:, 1 : dsIntvl : end);

%% 异常值平滑
for iPc = 1 : nPc
    phTemp = ph(iPc, :);
    [hampelDiff, idxErr] = hampel(diff(phTemp), 5, 3);
    diffOfDiff = hampelDiff - diff(phTemp);
    idxErr = find(idxErr) + 1;
    for iErr = 1 : length(idxErr)
        ph(iPc, idxErr(iErr) : end) = ph(iPc, idxErr(iErr) : end) + diffOfDiff(idxErr(iErr) - 1);
    end
end

%% 胸腔信号提取
confidence = zeros(nPc, 1); % 置信度, 指频谱尖峰能量在频谱总能量中的占比
for iPc = 1 : nPc
    fftRslt = abs(fft(ph(iPc, :))); % FFT获得频谱
    fftRslt = fftRslt(1 : floor(length(fftRslt) / 2)); % 只提取频谱前半部分, 即正频率部分
    [~, iFreqStr] = max(fftRslt(peakWidth + 1 : end)); % 频谱尖峰的索引. peakWidth+1指给尖峰宽度留出空间
    confidence(iPc) = sum(fftRslt(iFreqStr : iFreqStr + 2 * peakWidth)) / sum(fftRslt); % 计算置信度
end
[~, iPcChest] = fmax(confidence); % 选择置信度最大的点云代表胸腔
phFin = ph(iPcChest, :); % 提取胸腔相位曲线
switch locMode
    case 'range'
disp(['胸腔距离: ', num2str(round((pcRg.iRange(iPcChest) - 1) * resR * 100) / 100), 'm']) % 保留两位小数
    case 'RA'
disp(['胸腔距离: ', num2str(round((pcRA.iRange(iPcChest) - 1) * resR * 100) / 100), 'm']) % 保留两位小数
end

%% 波谷搜索
% phFin = phFin(1 : 1201);
axisT = axisT(1 : min(length(axisT), length(phFin)));
phFin = phFin(1 : min(length(axisT), length(phFin)));
[peak.value, peak.time] = findpeaks(-phFin, axisT, 'MinPeakDistance', 1/ fUpLmtResp, 'MinPeakProminence', 0.25 * max(abs(phFin)));
peak.value = -peak.value;
[~, peak.idx] = ismember(peak.time, axisT);

%% 转换成位移
ampl = phFin * lambda / 4 / pi;
peak.value = peak.value * lambda / 4 / pi;

%% 绘图
close all; drawRespCurve(ampl, axisT, peak); drawnow;

