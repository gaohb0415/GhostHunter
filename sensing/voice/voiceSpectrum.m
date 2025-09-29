function stm = voiceSpectrum(varargin)
% 通过探测喉咙振动计算声谱
% 输入: 
% varargin:
% - limitT: 时间范围, 决定读取哪些帧, s
% - limitFre: 声谱图频率范围, Hz
% - nFrmWindow: 窗口帧数
% - drawEn: 是否绘图. 0-否; 1-是
% 输出: 
% stm: 声谱图
% -.map: 频率-时间矩阵
% -.axisT: 时间轴
% -.axisFreq: 频率轴
% 作者: 刘涵凯
% 更新: 2022-8-15

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('limitT', [0.5, 6]);
p.addOptional('limitFreq', [50, 1000]);
p.addOptional('nFrmWindow', 4);
p.addOptional('drawEn', 1);
p.parse(varargin{:});
limitT = p.Results.limitT;
limitFreq = p.Results.limitFreq;
nFrmWindow = p.Results.nFrmWindow;
drawEn = p.Results.drawEn;

%% 频率设置
load('config.mat', 'tChirpIntvl', 'nChirp1Frm', 'tFrm', 'nFrm', 'resR')
maxFreq =  1 / tChirpIntvl; % 最大可分辨频率
limitFreq = [max(0, limitFreq(1)), min(maxFreq, limitFreq(2))]; % 感兴趣的频率范围
resFreq = 1 / (nFrmWindow * tFrm); % 频率分辨率
% 注意, 下面的Chirp数计算, 假设了两帧间的空白时间恰好是一个Chirp的时间
nChirp = (nChirp1Frm + 1) * nFrmWindow - 1;
% FFT结果的频率轴
if mod(nChirp, 2) == 0 % 若nChirp为偶数
    axisFreq = resFreq * (-nChirp / 2 : nChirp / 2 - 1);
else % 若nChirp为奇数
    axisFreq = resFreq * (-(nChirp + 1) / 2 : (nChirp + 1) / 2); 
end
iAoiFreq = find(axisFreq >= limitFreq(1) & axisFreq <= limitFreq(2)); % 感兴趣的频率的索引
stm.axisFreq = axisFreq(iAoiFreq); % 感兴趣的频率轴

%% 帧选取与时间设置
iFrmStart = max(1, floor(limitT(1) / tFrm)); % 起始帧
iFrmEnd = min(nFrm, ceil(limitT(2) / tFrm)); % 结束帧
iFrmLoad = iFrmStart : iFrmEnd; % 选取的帧
nFrmLoad = length(iFrmLoad); % 帧数
stm.axisT = tFrm * (iFrmStart + (-1 : nFrmLoad - 2)); % 时间轴

%% 喉咙定位
radarDataTemp = readBin(iFrmLoad(1), 0);
fftRsltRg = fftRange(radarDataTemp);
[iRgDet, ~] = bodyDetection(matExtract(abs(fftRsltRg), 1, [0, 0, 0]));
iRgDet.strongest = iRgDet.strongest;
disp(['喉咙距离: ', num2str(round((iRgDet.strongest) * resR * 100 * 100) / 100), 'cm']) % 保留两位小数

%% 计算声谱
stm.map = zeros(nChirp, nFrmLoad);
for iFrm = 1 : nFrmLoad
    % 载入数据
    radarDataTemp = readBin(iFrmStart + iFrm - 1 : iFrmStart + iFrm + nFrmWindow - 2 , 0);
    % Range FFT
    fftRsltRg = fftRange(radarDataTemp);
    % 在Tx和Rx维度平均
    fftRsltRg = squeeze(mean(fftRsltRg(iRgDet.strongest, :, :, :, :), [3, 4]));
    % 多帧窗口的帧间插值
    if nFrmWindow ~= 1
        fftRsltRgConnect = fftRsltRg(:);
        iChirp = 1 : nChirp;
        iChirp((mod(iChirp, nChirp1Frm + 1) == 0) & (iChirp ~= 1)) = [];
        fftRsltRg = interpn(iChirp, fftRsltRgConnect, 1 : nChirp, 'spline');
    end
    fftRsltRg = fftRsltRg(:); % 这里重复一遍是为了在不同长度的窗时保持维度一致
    % FFT
    stm.map(:, iFrm) = fftDoppler(fftRsltRg.'); % 注意转置符号
end
stm.map = stm.map(iAoiFreq, :);
stm.map = abs(stm.map);

%% 删去异常值
iRow = round(size(stm.map, 1) * (1 / 2)); % 以频谱的某行为对象
iDelete = find(stm.map(iRow, :) > 10 * median(stm.map(iRow, :))); % 寻找异常大的值
stm.map(:, iDelete) = [];
stm.axisT(iDelete) = [];

%% 写入结果
save('.\postProc\voice\voiceResults.mat', 'stm')

%% 绘图
if drawEn
    drawVoiceSpectrum
end
