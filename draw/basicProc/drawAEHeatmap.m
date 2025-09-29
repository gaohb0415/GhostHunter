function drawAEHeatmap(heatmap, az, el, varargin)
% 绘制Azimuth-Elevation Heatmap
% 输入: 
% 1. heatmap: Azimuth-Elevation Heatmap
% 2. varargin
%     - limitAz: 绘图水平角范围
%     - limitEl: 绘图俯仰角范围
%     - logEn: 是否将RDM颜色幅度设为dB. 0-否; 1-是
% 作者: 刘涵凯
% 更新: 2022-7-5

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('limitAz', az([1, end]));
p.addOptional('limitEl', el([1, end]));
p.addOptional('logEn', 0);
p.parse(varargin{:});
limitAz = p.Results.limitAz;
limitEl = p.Results.limitEl;
logEn = p.Results.logEn;

%% 图像参数
fontSize=12;

%% 绘图
figure
h = imagesc(az, el, heatmap', 'CDataMapping', 'scaled');

%% 图像设置
xlabel('Azimuth (°)', 'fontsize', fontSize);
ylabel('Elevation (°)', 'fontsize', fontSize);
set(gca, 'Xlim', limitAz)
set(gca, 'Ylim', limitEl)
set(gca,'YDir', 'normal')
if logEn
    set(gca, 'ColorScale', 'log')
end
set(gca, 'Fontsize', fontSize);
% set(gca, 'Box', 'off')
set(gca,'LooseInset', get(gca, 'TightInset'))
set(gcf, 'color', 'w')
set(gcf, 'Units', 'centimeters', 'Position', [14 2 12 9]);
