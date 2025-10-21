function drawRangeFFTnew(fftRsltRg, varargin)
% 绘制Range-FFT结果, 含CFAR阈值和点云 (New Version)
%
% 功能更新:
% - 支持在指定的坐标系(axes)上绘图，避免创建新窗口。
% - 通过可选参数 'ax' 传入坐标系句柄。
% - 若不提供'ax'参数，则行为与旧版完全相同，保证向后兼容。
%
% 作者: 刘涵凯 (由Gemini根据需求修改)
% 更新: 2025-10-20

%% 1. 默认参数解析
p = inputParser();
p.CaseSensitive = false;
p.addOptional('pcIdx', []);
p.addOptional('cfarTh', []);
p.addOptional('logEn', 1);
% --- 新增核心参数: 'ax'，用于接收外部传入的坐标系句柄 ---
p.addParameter('ax', [], @(x) isempty(x) || isa(x, 'matlab.graphics.axis.Axes'));
p.parse(varargin{:});

pcIdx = p.Results.pcIdx;
cfarTh = p.Results.cfarTh;
logEn = p.Results.logEn;
ax = p.Results.ax;

%% 2. 准备绘图目标
% --- 核心逻辑: 判断在哪里绘图 ---
if isempty(ax)
    % 如果外部没有指定坐标系，则维持旧行为：自己创建新窗口。
    figure;
    ax = gca; % 获取这个新窗口的坐标系句柄
else
    % 如果外部指定了坐标系，则将其激活为当前绘图目标。
    axes(ax);
end
cla(ax, 'reset'); % 清空目标坐标系，为绘制新一帧做准备
hold(ax, 'on');   % 在目标坐标系上保持绘图

%% 3. 数据与图像参数准备
load colorLib.mat
lineWidth1 = 1.5;
lineWidth2 = 1;
markerSize1 = 5;
markerSize2 = 7;
fontSize = 15;
marker1 = '.';
marker2 = 'o';

if logEn; fftRsltRg = 20 * log10(fftRsltRg); end % 将幅度转化为dB

load('config.mat', 'resR');
rg = resR * (0 : length(fftRsltRg) - 1); % 计算距离坐标

%% 4. 绘图 (所有绘图操作都作用于 ax)
% FFT结果
h1 = plot(ax, rg, fftRsltRg, 'LineStyle', '-', 'LineWidth', lineWidth1, 'Marker', marker1, ...
    'MarkerSize', markerSize1, 'Color', colorBlue);
% CFAR阈值
if ~isempty(cfarTh)
    if logEn; cfarTh = 20 * log10(cfarTh); end % 将CFAR阈值转化为dB
    h2 = plot(ax, rg, cfarTh, 'LineStyle', '-.', 'LineWidth', lineWidth2, 'Color', colorYellow);
end
% 点云
if ~isempty(pcIdx)
    h3 = plot(ax, resR * (pcIdx - 1), fftRsltRg(pcIdx), 'LineStyle', 'none', 'LineWidth', lineWidth1, ...
        'Marker', marker2, 'Color', colorRed, 'MarkerSize', markerSize2);
end

%% 5. 图像设置 (所有设置都作用于 ax)
hold(ax, 'off');
if isempty(cfarTh)
    % legend(...)
elseif isempty(pcIdx)
    legend(ax, [h1, h2], 'FFT Result', 'CFAR Threshold', 'fontsize', fontSize);
else
    legend(ax, [h1, h2, h3], 'FFT Result', 'CFAR Threshold', 'CFAR Result', 'fontsize', fontSize);
end

xlabel(ax, 'Range (m)', 'fontsize', fontSize);
if logEn
    ylabel(ax, 'Amplitude (dB)', 'fontsize', fontSize);
else
    ylabel(ax, 'Amplitude', 'fontsize', fontSize);
end

set(ax, 'Xlim', [0, rg(end)]);
set(ax, 'Fontsize', fontSize);
set(ax, 'Box', 'on');
set(ax, 'LooseInset', get(ax, 'TightInset'));
grid(ax, 'on');

% 图形窗口的设置 (可选，因为主脚本会控制窗口)
fig = ax.Parent;
set(fig, 'color', 'w');
% set(fig, 'Units', 'centimeters', 'Position', [2 2 12 9]); % 可以注释掉，让主脚本决定窗口大小

end