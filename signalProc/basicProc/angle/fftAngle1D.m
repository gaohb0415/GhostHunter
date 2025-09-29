function [fftRsltAng1D, pcRA] = fftAngle1D(radarData, varargin)
% 1D Angle FFT, 含点云生成
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC/Range, Chirp/Velocity, Rx, Tx]
% 2. varargin:
%     - windowEn: FFT时是否加窗. 0-否; 1-是
%     - limitR: 距离范围. []-不设范围
%     - nAngle: FFT点数
%     - pcEn: 是否计算点云. 0-否; 1-是
%     - drawEn: 是否绘图. 0-否; 1-是
%     - logEn: 是否将RAM颜色幅度设为dB. 0-否; 1-是
% 输出:
% 1. fftRsltAng1D: 雷达数据矩阵, [ADC/Range, Chirp/Velocity, Angle]
% 2. pcRA: RA点云
%     - .iRange: range bin索引
%     - .iAngle: angle bin索引
%     - .range: 距离
%     - .angle: 角度
%     - .x: x坐标
%     - .y: y坐标
%     - .power: 反射强度
% 作者: 刘涵凯
% 更新: 2022-7-5

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('windowEn', 0);
p.addOptional('limitR', []);
p.addOptional('nAngle', 256);
p.addOptional('pcEn', 0);
p.addOptional('drawEn', 0);
p.addOptional('logEn', 0);
p.parse(varargin{:});
windowEn = p.Results.windowEn;
limitR = p.Results.limitR;
nAngle = p.Results.nAngle;
pcEn = p.Results.pcEn;
drawEn = p.Results.drawEn;
logEn = p.Results.logEn;

%% 雷达参数
load('config.mat', 'resR', 'spacingCal')
nRg = size(radarData, 1);
rg = resR * (0 : nRg - 1)'; % 总距离刻度

%% 提取所选距离范围内的数据
if any(limitR)
    aoiRg = [max(limitR(1), rg(1)), min(limitR(2), rg(end))];  % 距离AOI(area of interest)
    iAoiRg = rg >= aoiRg(1) & rg <= aoiRg(2);
    rg = rg(iAoiRg);
    radarData = radarData(iAoiRg, :, :, :);
end

%% 生成虚拟阵列
antArray = virtualArray1D(radarData, 'FFT');

%% 获取数据矩阵尺寸
[nRg, nChirp, nRx] = size(antArray.signal);

%% Angle FFT
if windowEn % 加窗(汉宁窗)
    antArray.signal = antArray.signal .* permute(repmat(hanning(nRx), [1, nRg, nChirp]), [2, 3, 1]);
end
fftRsltAng1D = fftshift(fft(antArray.signal, nAngle, 3), 3);
fftRsltAng1D = flip(fftRsltAng1D, 3); % 角度翻转

%% 点云生成
pcRA = struct('iRange', [], 'iAngle', [], 'range', [], 'angle', [], 'x', [], 'y', [], 'power', []);
if pcEn
% 2D CFAR
    load('config.mat', 'cfarParamRA')
    pwRA = matExtract(abs(fftRsltAng1D), [1, 3], [0, 0, 0]); % 提取RA矩阵
    [pcRA.iRange, pcRA.iAngle, ~] = cfar2D(pwRA, cfarParamRA); % 执行CFAR
    if isempty(pcRA.iRange); warning('未检测到RA点云'); end
    % 计算点云信息
    ang = asind((0 : nAngle - 1) / (nAngle / 2) - 1)'; % 计算角度坐标刻度. 不以angle为变量名, 因为可能与自带angle函数冲突
    ang = ([ang(2 : end); 90] + ang) / 2; % 以每个angle bin的中心作为其刻度
    ang = ang / spacingCal; % 天线间距校准
    pcRA.range = rg(pcRA.iRange);
    pcRA.angle = ang(pcRA.iAngle);
    pcRA.x = pcRA.range .* sind(pcRA.angle);
    pcRA.y = pcRA.range .* cosd(pcRA.angle);
    fftResultRAVec = pwRA(:); % 将RA矩阵转换为向量, 用于获得点云反射强度
    iRA = sub2ind([nRg, nAngle], pcRA.iRange, pcRA.iAngle);
    pcRA.power = fftResultRAVec(iRA);
end

%% 绘图
if drawEn
    % 提取结果矩阵
    if ~exist('fftResultRA', 'var')
        pwRA = matExtract(abs(fftRsltAng1D), [1, 3], [0, 0, 0]);
    end
    if ~exist('ang', 'var')
        ang = asind((0 : nAngle - 1) / (nAngle / 2) - 1)';
        ang = ([ang(2 : end); 90] + ang) / 2; % 以每个angle bin的中心作为其刻度
        ang = ang / spacingCal; % 天线间距校准
    end
    % 解决使用polarPcolor绘图不完整问题
    pwRA = repelem(pwRA, 1, 2);
    ang = [-90; repelem(ang(2 : end), 2, 1); 90];
    % 绘图
    drawRAM(pwRA, rg, ang, 'pcRA', pcRA, 'logEn', logEn)
end
