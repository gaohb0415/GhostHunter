function drawRangeFFT(fftRsltRg, varargin)
% 绘制Range-FFT结果, 含CFAR阈值和点云
% 输入:
% 1. fftRsltRg: Range FFT结果, 实数向量
% 2. varargin
%     - pcIdx: range bin索引. []-未执行CFAR
%     - cfarTh: CFAR阈值
%     - logEn: 是否将纵坐标设为dB. 0-否; 1-是
% 作者: 刘涵凯
% 更新: 2022-6-24


%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('pcIdx', []);
p.addOptional('cfarTh', []);
p.addOptional('logEn', 1);
p.parse(varargin{:});
pcIdx = p.Results.pcIdx;
cfarTh = p.Results.cfarTh;
logEn = p.Results.logEn;

%% 图像参数
load colorLib.mat
lineWidth1 = 1.5;
lineWidth2 = 1;
markerSize1 = 5;
markerSize2 = 7;
fontSize = 15; % 12
marker1 = '.';
marker2 = 'o';

%% 将幅度转化为dB,雷达信号可视化中的标准操作
if logEn; fftRsltRg = 20 * log10(fftRsltRg); end

%% 计算坐标刻度
%% resR：距离分辨率，根据距离分辨率从fftRsltRg中无物理意义的数据映射到真实世界物理距离
load('config.mat', 'resR')
rg = resR * (0 : length(fftRsltRg) - 1); % 不以range为变量名, 因为可能与自带range函数冲突

%% 绘图
figure
hold on
% FFT结果
h1 = plot(rg, fftRsltRg, 'LineStyle', '-', 'LineWidth', lineWidth1, 'Marker', marker1, ...
    'MarkerSize', markerSize1, 'Color', colorBlue);
% CFAR阈值
if ~isempty(cfarTh)
    if logEn; cfarTh = 20 * log10(cfarTh); end % 将CFAR阈值转化为dB
    h2 = plot(rg, cfarTh, 'LineStyle', '-.', 'LineWidth', lineWidth2, 'Color', colorYellow);
end
% 点云
if ~isempty(pcIdx)
    h3 = plot(resR * (pcIdx - 1), fftRsltRg(pcIdx), 'LineStyle', 'none', 'LineWidth', lineWidth1, ...
        'Marker', marker2, 'Color', colorRed, 'MarkerSize', markerSize2);
end
% 图例
if isempty(cfarTh)
    % legend(h1, 'FFT Result', 'fontsize', fontSize);
elseif isempty(pcIdx)
    legend([h1, h2], 'FFT Result', 'CFAR Threshold', 'fontsize', fontSize);
else
    legend([h1, h2, h3], 'FFT Result', 'CFAR Threshold', 'CFAR Result', 'fontsize', fontSize);
end

%% 图像设置
%% 实验中真正需要修改的是这里
%% 在这个图像设置模块中可以通过set来手动框选信号的显示范围
%% 让数据显示的结果"放大"
xlabel('Range (m)', 'fontsize', fontSize)
if logEn
    ylabel('Amplitude (dB)', 'fontsize', fontSize) % 暂时没搞清楚单位
else
    ylabel('Amplitude', 'fontsize', fontSize)
end
% xlabel('距离 (m)', 'fontsize', fontSize)
% if logEn
%     ylabel('信号强度 (dB)', 'fontsize', fontSize) % 暂时没搞清楚单位
% else
%     ylabel('信号强度', 'fontsize', fontSize)
% end
set(gca, 'Xlim', [0, rg(end)])
set(gca, 'Fontsize', fontSize)
set(gca, 'Box', 'on')
set(gca, 'LooseInset', get(gca, 'TightInset'))
set(gcf, 'color', 'w')
set(gcf, 'Units', 'centimeters', 'Position', [2 2 12 9])
grid on
