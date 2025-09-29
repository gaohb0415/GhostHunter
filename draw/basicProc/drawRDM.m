function drawRDM(mapRD, varargin)
% 绘制Range-Doppler Map, 含点云
% 输入: 
% 1. mapRA: Range-Doppler实数矩阵
% 2. varargin
%     - rg: 距离刻度
%     - pcRD: CFAR阈值和点云索引
%       * .iRange: Range bin索引. []-未执行CFAR
%       * .iVelocity: Velocity bin索引
%     - logEn: 是否将RDM颜色幅度设为dB. 0-否; 1-是
% 作者: 刘涵凯
% 更新: 2022-6-24

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('rgTick', []);
p.addOptional('pcRD', struct('iRange', []));
p.addOptional('logEn', 1);
p.parse(varargin{:});
rg = p.Results.rgTick;
pcRD = p.Results.pcRD;
logEn = p.Results.logEn;

%% 图像参数
fontSize=15; % 12

%% 计算坐标刻度
load('config.mat', 'resR', 'resV')
[nRg, nVel] = size(mapRD);
if isempty(rg); rg = resR * (0 : nRg - 1); end % 若未输入距离刻度, 则计算默认刻度
vel = resV * (-nVel / 2 : nVel / 2 - 1); % nChirp为偶数时, 以第 nChirp/2+1个点为0

%% 绘图
figure
% RDM
h1 = imagesc(vel, rg, mapRD, 'CDataMapping', 'scaled');
% 点云
if ~isempty(pcRD.iRange)
    hold on
    h2 = plot(vel(pcRD.iVelocity), rg(pcRD.iRange), '.', 'Color', 'r');
end

%% 图像设置
% xlabel('Velocity (m/s)', 'fontsize', fontSize)
% ylabel('Range (m)', 'fontsize', fontSize)
xlabel('速度 (m/s)', 'fontsize', fontSize)
ylabel('距离 (m)', 'fontsize', fontSize)
set(gca, 'Xlim', [vel(1) - resV / 2, vel(end) + resV / 2])
set(gca, 'Ylim', [rg(1) - resR / 2, rg(end) + resR / 2])
set(gca,'YDir', 'normal')
if logEn % 将幅度转化为dB
    set(gca, 'ColorScale', 'log')
end
set(gca, 'Fontsize', fontSize)
set(gca, 'LooseInset', get(gca, 'TightInset'))
set(gcf, 'color', 'w')
set(gcf, 'Units', 'centimeters', 'Position', [14 2 12 9])
