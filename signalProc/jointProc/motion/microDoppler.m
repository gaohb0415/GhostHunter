function mdRslt = microDoppler(varargin)
% 微多普勒谱
% 支持距离和RA信号提取
% 支持轨迹辅助
% 输入:
% varargin:
% - dataType: 数据类型. 'real'; 'sim'
% - dataInfo: 仿真数据信息
%   * .handle: 地址
%   * .frmMode: 存储模式. 'allFrm'; '1Frm'
% - trajInfo: 轨迹追踪信息
%   * .handle: 地址
%   * .trajEn: 是否使用轨迹信息. 0; 1. 若不使用, 则默认场景中仅有一人
%   * .iTraj: 使用结构体中的第几条轨迹
% - locMode: 定位及信号提取模式. 'range'; 'RA'
% 作者: 刘涵凯
% 更新: 2024-6-21

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('dataType', 'real');
p.addOptional('dataInfo', struct('handle', [], 'frmMode', '1Frm'));
p.addOptional('trajInfo', struct('handle', 'trackingResults.mat', 'trajEn', 1, 'iTraj', 2));
p.addOptional('locMode', 'RA');
p.parse(varargin{:});
dataType = p.Results.dataType;
dataInfo = p.Results.dataInfo;
trajInfo = p.Results.trajInfo;
locMode = p.Results.locMode;

%% 参数设置
load('config.mat', 'nAdc1Chirp', 'nChirp1Frm', 'resV', 'resR', 'tFrm', 'nFrm')
iFrmLoad = 1 : 201; % 帧设定
winRg = 8; % 距离聚焦框, 单边
winAng = 5; % 角度聚焦框, 单边.  当前版本默认角度步进为1度, 如果不是的话需要小改一下

%% 读取轨迹信息并计算距离角度
if trajInfo.trajEn
    load(trajInfo.handle)
    iFrmLoad(iFrmLoad > length(trajectory)) = []; % 重设帧范围
    pos = trajectory(iFrmLoad(end)).track(trajInfo.iTraj).trajectory;
    d = vecnorm(pos, 2, 2);
    idxRgPkAll = ceil(d / resR);
    angPkAll = atan2d(pos(:, 1), pos(:, 2));
end

%% 计算微多普勒谱
mdRslt = zeros(nChirp1Frm, length(iFrmLoad)); % 初始化微多普勒谱
for iFrm = 1 : length(iFrmLoad)
    % 读取数据
    radarData= readBin(max(1, mod(iFrmLoad(iFrm), nFrm)), 0, dataType = dataType, dataInfo = dataInfo);
    [radarData, ~] = fftRange(radarData); % Range FFT

    %% 提取信号
    switch locMode
        case 'range'
            % 确认距离范围
            if trajInfo.trajEn
                idxRgPk = idxRgPkAll(iFrmLoad(iFrm));
            else
                [~, idxRgPk] = max(matExtract(abs(radarData), 1, [0, 0, 0]));
            end
            idxRgBody = idxRgPk + (-winRg : winRg); % 提取目标附近的range bin
            if idxRgBody(1) < 1; idxRgBody = idxRgBody - idxRgBody(1) + 1; end
            if idxRgBody(end) > nAdc1Chirp; idxRgBody = idxRgBody - idxRgBody(end) + nAdc1Chirp; end
            % 信号提取
            radarData = radarData(idxRgBody, :, :, :);
        case 'RA'
            if trajInfo.trajEn
                idxRgPk = idxRgPkAll(iFrmLoad(iFrm));
                angPk = angPkAll(iFrmLoad(iFrm));
            else
                limitAng = [-90, 90];
                [pwRA, ~] = dbfProc1D(radarData, 'limitAng', limitAng);
                % 确认距离和角度范围
                [idxRgPk, idxAngPk] = find(pwRA == max(max(pwRA)));
                angPk = limitAng(1) + idxAngPk - 1;
            end
            idxRgBody = idxRgPk + (-winRg : winRg);
            if idxRgBody(1) < 1; idxRgBody = idxRgBody - idxRgBody(1) + 1; end
            if idxRgBody(end) > nAdc1Chirp; idxRgBody = idxRgBody - idxRgBody(end) + nAdc1Chirp; end
            % 角度
            angBody = angPk + (-winAng : winAng);
            if angBody(1) < -90; angBody = angBody - angBody(1) - 90; end
            if angBody(end) > 90; angBody = angBody - angBody(end) + 90; end
            % 信号提取
            radarData = dbfRecons(radarData, [repelem(idxRgBody', 2 * winAng + 1, 1), repmat(angBody', [2 * winRg + 1, 1])]).';
    end

    %% Doppler FFT
    radarData = radarData .* permute(repmat(hanning(nChirp1Frm), [1, size(radarData, [1, 3, 4])]), [2, 1, 3, 4]); % 加汉宁窗
    fftRsltDop = fftshift(fft(radarData, nChirp1Frm, 2), 2); % Doppler FFT
    mdRslt(:, iFrm) = matExtract(abs(fftRsltDop), 2, [0, 0, 0]); % 提取结果矩阵
end

%% 保存数据
% save('.\signalProc\jointProc\motion\mdRslt.mat', 'mdRslt', 'iFrmLoad', 'nChirp1Frm', 'resV', 'tFrm')

%% 绘图
drawMicroDoppler('logEn', 1); drawnow
