function [fftRsltDop, pcRD]= fftDoppler(radarData, varargin)
% Doppler FFT, 含点云生成
% FFT点数自动选择为一帧中的Chirp数
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC/Range, Chirp, Rx, Tx]
% 2. varargin:
%     - windowEn: FFT时是否加窗. 0-否; 1-是
%     - limitR: 距离范围. []-不设范围
%     - pcEn: 是否计算点云. 0-否; 1-是
%     - drawEn: 是否绘图. 0-否; 1-是
%     - logEn: 是否将RAM颜色幅度设为dB. 0-否; 1-是
% 输出:
% 1. fftRsltDop: 雷达数据矩阵, [ADC/Range, Velocity, Rx, Tx]
% 2. pcRD: RD点云
%     - .iRange: Range bin索引
%     - .iVelocity: Velocity bin索引
%     - .range: 距离
%     - .velocity: 速度
%     - .power: 反射强度
% 作者: 刘涵凯
% 更新: 2022-7-19

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('windowEn', 1);
p.addOptional('limitR', []);
p.addOptional('pcEn', 0);
p.addOptional('drawEn', 0);
p.addOptional('logEn', 1);
p.parse(varargin{:});
windowEn = p.Results.windowEn;
limitR = p.Results.limitR;
pcEn = p.Results.pcEn;
drawEn = p.Results.drawEn;
logEn = p.Results.logEn;

%% 雷达参数
load('config.mat', 'resR', 'resV')
nRg = size(radarData, 1);
rg = resR * (0 : nRg - 1)'; % 总距离刻度

%% 提取所选距离范围内的数据
if any(limitR)
    aoiRg = [max(limitR(1), rg(1)), min(limitR(2), rg(end))];  % 距离AOI(area of interest)
    iAoiRg = rg >= aoiRg(1) & rg <= aoiRg(2);
    rg = rg(iAoiRg);
    radarData = radarData(iAoiRg, :, :, :);
end

%% 获取数据矩阵尺寸
[nRg, nChirp, nRx, nTx, nFrm] = size(radarData);

%% Doppler FFT
if windowEn % 加窗(汉宁窗)
    radarData = radarData .* permute(repmat(hanning(nChirp), [1, nRg, nRx, nTx, nFrm]), [2, 1, 3, 4, 5]);
end
fftRsltDop = fftshift(fft(radarData, nChirp, 2), 2);

%% 点云生成
pcRD = struct('iRange', [], 'iVelocity', [], 'range', [], 'velocity', [], 'power', []);
if pcEn
    % 2D CFAR
    load('config.mat', 'cfarParamRD')
    pwRD = matExtract(abs(fftRsltDop), [1, 2], [0, 0, 0]); % 提取RD矩阵
    [pcRD.iRange, pcRD.iVelocity, ~] = cfar2D(pwRD, cfarParamRD); % 执行CFAR
    if isempty(pcRD.iRange); warning('未检测到RD点云'); end
    % 计算点云信息
    vel = resV * (-nChirp / 2 : nChirp / 2 - 1)'; % 计算速度坐标刻度. nChirp为偶数时, 以第 nChirp/2+1个点为0
    pcRD.range = rg(pcRD.iRange);
    pcRD.velocity = vel(pcRD.iVelocity);
    pwRDVec = pwRD(:); % 将RD矩阵转换为向量, 用于获得点云反射强度
    iRD = sub2ind([nRg, nChirp], pcRD.iRange, pcRD.iVelocity);
    pcRD.power = pwRDVec(iRD);
end

%% 绘图
if drawEn
    if ~exist('pwRD', 'var')
        pwRD = matExtract(abs(fftRsltDop), [1, 2], [0, 0, 0]); % 提取结果矩阵
    end
    drawRDM(pwRD, 'rg', rg, 'pcRD', pcRD, 'logEn', logEn)
end
