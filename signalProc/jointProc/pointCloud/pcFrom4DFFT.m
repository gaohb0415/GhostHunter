function pc3D = pcFrom4DFFT(radarData, varargin)
% 由4D FFT获得3D点云
% 各3D点云含速度信息, 所以也可以视为4D点云
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC, Chirp, Rx, Tx]
% 2. varargin:
%     - limitR: 距离范围. []-不设范围
%     - limitX, limitY, limitZ: 点云范围. [0, 0]-不设范围
%     - nAz: 水平角FFT点数
%     - nEl: 俯仰角FFT点数
%     - nPeakAz: 水平角峰数
%     - nPeakEl: 俯仰角峰数
%     - drawEn: 是否绘图. 0-否; 1-是
% 输出:
% pc3D: 3D点云
% - .x
% - .y
% - .z
% - .vel
% - .pw
% 作者: 刘涵凯
% 更新: 2023-6-30

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('limitR', []);
p.addOptional('limitX', [0, 0]);
p.addOptional('limitY', [0, 0]);
p.addOptional('limitZ', [0, 0]);
p.addOptional('nAz', 128);
p.addOptional('nEl', 128);
p.addOptional('nPeakAz', 1);
p.addOptional('nPeakEl', 1);
p.addOptional('drawEn', 0);
p.parse(varargin{:});
limitR = p.Results.limitR;
limitX = p.Results.limitX;
limitY = p.Results.limitY;
limitZ = p.Results.limitZ;
nAz = p.Results.nAz;
nEl = p.Results.nEl;
nPeakAz = p.Results.nPeakAz;
nPeakEl = p.Results.nPeakEl;
drawEn = p.Results.drawEn;

%% Range FFT
[fftRsltRg, ~] = fftRange(radarData);

%% 雷达参数
load('config.mat', 'resR', 'resV', 'spacingCal', 'posRadar')
nRg = size(fftRsltRg, 1);
rg = resR * (0 : nRg - 1)'; % 总距离刻度

%% 提取所选距离范围内的数据
if any(limitR)
    aoiRg = [max(limitR(1), rg(1)), min(limitR(2), rg(end))];  % 距离AOI(area of interest)
    iAoiRg = rg >= aoiRg(1) & rg <= aoiRg(2);
    rg = rg(iAoiRg);
    fftRsltRg = fftRsltRg(iAoiRg, :, :, :);
end

%% 计算坐标轴
% 速度
nChirp = size(fftRsltRg, 2);
vel = resV * (-nChirp / 2 : nChirp / 2 - 1)'; % nChirp为偶数时, 以第 nChirp/2+1个点为0
% 水平角
az = asind((0 : nAz - 1) / (nAz / 2) - 1); % 水平角
az = ([az(2 : end), 90] + az) / 2; % 以每个angle bin的中心作为其刻度
az = az' / spacingCal; % 天线间距校准
% 俯仰角
el = asind((0 : nEl - 1) / (nEl / 2) - 1); % 俯仰角
el = ([el(2 : end), 90] + el) / 2; % 以每个angle bin的中心作为其刻度
el = el' / spacingCal; % 天线间距校准

%% Doppler FFT
[fftRsltRD, pcRD] = fftDoppler(fftRsltRg, 'pcEn', 1);

%% 对fftResultRD进行提取与重组
sizeRD = size(fftRsltRD);
nPc = length(pcRD.iRange); % RD点云的数量
% RD矩阵->RD向量, 注意第一维保留. 该操作是为了将提取的RD点云信号输入到virtualArray2D函数中
fftRsltRD = reshape(fftRsltRD, [1, sizeRD(1) * sizeRD(2), sizeRD(3 : end)]);
pcRDSig = fftRsltRD(1, sub2ind(sizeRD([1, 2]), pcRD.iRange, pcRD.iVelocity), :, :, :); % 提取RD点云信号

%% 生成虚拟阵列
antArray = virtualArray2D(pcRDSig, 'FFT');
antArray.signal = permute(antArray.signal, [4, 3, 2, 1]); % 删除刚才保留的第一维, 并将维度转换为[Elevation, Azimuth, RD]

%% 水平角 Angle FFT
fftRsltAng1D = fftshift(fft(antArray.signal, nAz, 2), 2);
fftRsltAng1D = flip(fftRsltAng1D, 2);

%% 根据nPeakAz扩展点云数量
fftRsltAng1D =  repelem(fftRsltAng1D, 1, 1, nPeakAz);
pcRD.iRange = repelem(pcRD.iRange, nPeakAz, 1);
pcRD.iVelocity = repelem(pcRD.iVelocity, nPeakAz, 1);
iPcAz = zeros(nPc * nPeakAz, 1); % 每个点云的az峰值在az中的索引

%% 水平角峰值搜索
pwAz = matExtract(abs(fftRsltAng1D), [2, 3], 0);
iDel = []; % 用于删除寻得峰数小于设定峰数时的冗余
for iPc = 1 : nPc
    [~, idxPeak] = findpeaks(pwAz(:, iPc), 'SortStr', 'descend', 'NPeaks', nPeakAz);
    nPeak = length(idxPeak);
    if nPeak ~= nPeakAz
        iDel = [iDel, iPc * nPeakAz - (nPeakAz - nPeak) + 1 : iPc * nPeakAz]; % 点云冗余
    end
    for iPeak = 1 : nPeak
        iPcAz((iPc - 1) * nPeakAz + (1 : nPeak)) = idxPeak;
    end
end
% 删除冗余
fftRsltAng1D(:, :, iDel) = [];
pcRD = structElementDelete(pcRD, iDel, 'fieldNames', ["iRange", "iVelocity"]);
iPcAz(iDel) = [];

%% 计算二维坐标
pcRA.x = rg(pcRD.iRange) .* sind(az(iPcAz));
pcRA.y = rg(pcRD.iRange) .* cosd(az(iPcAz));

%% 根据先验XY范围删除超出界限的点云
if any(limitX)
    iDelX = pcRA.x(:) < limitX(1) | pcRA.x(:) > limitX(2);
end
if any(limitY)
    iDelY = pcRA.y(:) < limitY(1) | pcRA.y(:) > limitY(2);
end
iDel = iDelX | iDelY;
fftRsltAng1D(:, :, iDel) = [];
pcRD = structElementDelete(pcRD, iDel, 'fieldNames', ["iRange", "iVelocity"]);
iPcAz(iDel) = [];

%% 提取RDA点云信号
nPc = length(iPcAz);
pcRDASig = reshape(fftRsltAng1D, [], nAz * nPc);
pcRDASig = pcRDASig(:, sub2ind([nAz, nPc], iPcAz', 1 : nPc));

%% 俯仰角 Angle FFT
fftRsltAng2D = fftshift(fft(pcRDASig, nEl, 1), 1);
fftRsltAng2D = flip(fftRsltAng2D, 1);

%% 根据nPeakEl扩展点云数量
pcRD.iRange = repelem(pcRD.iRange, nPeakEl, 1);
pcRD.iVelocity = repelem(pcRD.iVelocity, nPeakEl, 1);
iPcAz = repelem(iPcAz, nPeakEl, 1);
iPcEl = zeros(nPc * nPeakEl, 1); % 每个点云的el峰值在el中的索引
pc3D.pw = zeros(nPc * nPeakEl, 1);

%% 俯仰角峰值搜索
pwEl = abs(fftRsltAng2D);
iDel = []; % 用于删除寻得峰数小于设定峰数时的冗余
for iPc = 1 : nPc
    [~, idxPeak] = findpeaks(pwEl(:, iPc), 'SortStr', 'descend', 'NPeaks', nPeakEl);
    nPeak = length(idxPeak);
    if nPeak ~= nPeakEl
        iDel = [iDel, iPc * nPeakEl - (nPeakEl - nPeak) + 1 : iPc * nPeakEl]; % 点云冗余
    end
    for iPeak = 1 : nPeak
        iPcEl((iPc - 1) * nPeakEl + (1 : nPeak)) = idxPeak;
        pc3D.pw((iPc - 1) * nPeakEl + (1 : nPeak)) = pwEl(idxPeak);
    end
end
% 删除冗余
pc3D.pw(iDel) = [];
pcRD = structElementDelete(pcRD, iDel, 'fieldNames', ["iRange", "iVelocity"]);
iPcAz(iDel) = [];
iPcEl(iDel) = [];

%% 计算三维坐标/速度
pc3D.x = rg(pcRD.iRange) .* sind(az(iPcAz)) .* cosd(el(iPcEl));
pc3D.y = rg(pcRD.iRange) .* cosd(az(iPcAz)) .* cosd(el(iPcEl));
pc3D.z = rg(pcRD.iRange) .* sind(el(iPcEl)) + posRadar(3);
pc3D.vel= vel(pcRD.iVelocity);

%% 根据先验Z范围删除超出界限的点云
if any(limitZ)
    iDelZ = pc3D.z(:) < limitZ(1) | pc3D.z(:) > limitZ(2);
end
pc3D = structElementDelete(pc3D, iDelZ);

%% 绘图
if drawEn
    drawPc3D([pc3D.x, pc3D.y, pc3D.z], 'vel', pc3D.vel, 'pw', [], 'limitX', limitX, 'limitY', limitY, 'limitZ', limitZ)
end
