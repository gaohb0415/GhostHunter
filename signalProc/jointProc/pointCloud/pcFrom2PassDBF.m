function pc3D = pcFrom2PassDBF(radarData, varargin)
% 由两次DBF获得3D点云, 首先获得RA点云, 然后获得每个点云的俯仰角峰值
% 各3D点云含速度信息, 所以也可以视为4D点云
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC, Chirp, Rx, Tx]
% 2. varargin:
%     - limitR: 距离范围. []-不设范围
%     - limitX, limitY, limitZ: 点云范围. [0, 0]-不设范围
%     - limitAz: 水平角范围
%     - limitEl: 俯仰角范围
%     - resAz: 水平角步进间隔
%     - resEl: 俯仰角步进间隔
%     - nPeakEl: 俯仰角峰数
%     - drawEn: 是否绘图. 0-否; 1-是
% 输出:
% pc3D: 3D点云
% - .x
% - .y
% - .z
% - .vel
% - .pw:
% 作者: 刘涵凯
% 更新: 2024-6-20

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('limitR', []);
p.addOptional('limitX', [0, 0]);
p.addOptional('limitY', [0, 0]);
p.addOptional('limitZ', [0, 0]);
p.addOptional('limitAz', [-60, 60]);
p.addOptional('limitEl', [-30, 30]);
p.addOptional('resAz', 1);
p.addOptional('resEl', 1);
p.addOptional('nPeakEl', 1);
p.addOptional('drawEn', 0);
p.parse(varargin{:});
limitR = p.Results.limitR;
limitX = p.Results.limitX;
limitY = p.Results.limitY;
limitZ = p.Results.limitZ;
limitAz = p.Results.limitAz;
limitEl = p.Results.limitEl;
resAz = p.Results.resAz;
resEl = p.Results.resEl;
nPeakEl = p.Results.nPeakEl;
drawEn = p.Results.drawEn;

%% Range FFT
[fftRsltRg, ~] = fftRange(radarData);

%% 雷达参数
load('config.mat', 'resR', 'resV', 'spacingCal', 'posRadar')
[nRg, nChirp]= size(fftRsltRg, [1, 2]);
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
vel = resV * (-nChirp / 2 : nChirp / 2 - 1)'; % nChirp为偶数时, 以第 nChirp/2+1个点为0
% 水平角
az = (max(limitAz(1), -90) : resAz : min(limitAz(end), 90))'; % 扫描角度-水平角
% 俯仰角
el = (max(limitEl(1), -90) : resEl : min(limitEl(end), 90))'; % 扫描角度-俯仰角
nEl = length(el);

%% 获取RAM及点云索引
[~, pcRA] = dbfProc1D(fftRsltRg, 'rg', rg, 'pcEn', 1, 'limitAng', limitAz, 'resAng', resAz); % 1D DBF

%% 根据先验XY范围删除超出界限的点云
if any(limitX)
    iDelX = pcRA.x(:) < limitX(1) | pcRA.x(:) > limitX(2);
end
if any(limitY)
    iDelY = pcRA.y(:) < limitY(1) | pcRA.y(:) > limitY(2);
end
iDel = iDelX | iDelY;
pcRA = structElementDelete(pcRA, iDel, 'fieldNames', ["iRange", "iAngle"]);

%% 提取含点云的Range Bin的信号
nPc = length(pcRA.iRange);
iRgContainPc = unique(pcRA.iRange); % 含点云的range bin索引
fftRsltRgPc = fftRsltRg(iRgContainPc, :, :, :); % 含点云的range bin信号
nRg = length(iRgContainPc); % 含点云的range bin数量

%% 获取每个点云的俯仰角度谱
antArray = virtualArray2D(fftRsltRgPc, 'DBF'); % 生成虚拟阵列
pwEl = zeros(nPc * nEl, 1);
for iRg = 1 : nRg
    iPc = find(pcRA.iRange == iRgContainPc(iRg));
    azPc = az(pcRA.iAngle(iPc)); % 该range下存在点云的水平角
    nAzPc = length(iPc); % 为了在角度组合中容易理解, 用水平角数量表示点云数量
    % 角度组合
    angPairAz = repelem(azPc, nEl, 1); % 所有角度组合中的水平角
    angPairEl = repmat(el, [nAzPc, 1]); % 所有角度组合中的俯仰角
    % 2D DBF
    [pwEl((iPc(1) - 1) * nEl + 1 : iPc(end) * nEl), ~] = dbf(angPairAz, angPairEl, antArray.signal(:, :, iRg), ...
        antArray.arrayPosX, antArray.arrayPosZ, 'spacingCal', spacingCal);
end
pwEl = reshape(pwEl, [nEl, nPc]);

%% 根据nPeakEl扩展点云数量
pcRAE.iRg = repelem(pcRA.iRange, nPeakEl, 1);
pcRAE.rg = repelem(rg(pcRA.iRange), nPeakEl, 1);
pcRAE.az = repelem(az(pcRA.iAngle), nPeakEl, 1);
pcRAE.el = zeros(nPc * nPeakEl, 1);

%% 俯仰角谱峰搜索
iDel = []; % 用于删除寻, 得峰数小于设定峰数时的冗余
for iPc = 1 : nPc
    [~, idxPeak] = findpeaks(pwEl(:, iPc), 'SortStr', 'descend', 'NPeaks', nPeakEl);
    nPeak = length(idxPeak);
    if nPeak ~= nPeakEl
        iDel = [iDel, iPc * nPeakEl - (nPeakEl - nPeak) + 1 : iPc * nPeakEl]; % 点云冗余
    end
    for iPeak = 1 : nPeak
        pcRAE.el((iPc - 1) * nPeakEl + (1 : nPeak)) = el(idxPeak);
    end
end
% 删除冗余
pcRAE = structElementDelete(pcRAE, iDel);

%% 计算三维坐标
pc3D.x = pcRAE.rg .* sind(pcRAE.az) .* cosd(pcRAE.el);
pc3D.y = pcRAE.rg .* cosd(pcRAE.az) .* cosd(pcRAE.el);
pc3D.z = pcRAE.rg .* sind(pcRAE.el) + posRadar(3);

%% 根据先验Z范围删除超出界限的点云
if any(limitZ)
    iDelZ = pc3D.z(:) < limitZ(1) | pc3D.z(:) > limitZ(2);
end
pc3D = structElementDelete(pc3D, iDelZ);
% 在pcRAE中也删除相应点云, 以减小速度计算时的计算量
pcRAE = structElementDelete(pcRAE, iDelZ);
nPc = length(pcRAE.iRg);

%% 计算点云速度
sigRecons = dbfRecons(fftRsltRg, [pcRAE.iRg, pcRAE.az, pcRAE.el]); % 信号重构
[fftRsltDop, ~] = fftDoppler(sigRecons.'); % 计算速度, 注意非共轭转置
fftRsltDop = abs(fftRsltDop);
% 计算能量
[pc3D.pw, iVel] = max(fftRsltDop, [], 2);
pwMean = mean(fftRsltDop, 2);
% 计算速度
pc3D.vel = vel(iVel); % 速度初始化
% 若速度峰不够尖锐, 即强度占比低于某阈值, 则将点云速度设为0
pc3D.vel(pc3D.pw < pwMean * 5) = 0;
% 若0速度左右的两个速度是速度谱中最强的两个速度, 则将点云速度设为0
for iPc = 1 : nPc
    [~, velSort] = sort(fftRsltDop(iPc, :), 'descend');
    if ismember(nChirp / 2, velSort([1, 2])) && ismember(nChirp / 2 + 2, velSort([1, 2]))
        pc3D.vel(iPc) = 0;
    end
end

%% 绘图
if drawEn
    drawPc3D([pc3D.x, pc3D.y, pc3D.z], 'vel', pc3D.vel, 'pw', [], 'limitX', limitX, 'limitY', limitY, 'limitZ', limitZ)
    % drawPc3D([pc3D.x, pc3D.y, pc3D.z], 'vel', pc3D.vel, 'pw', [], 'limitX', limitX, 'limitY', [0, limitY(2)], 'limitZ', limitZ)
end
