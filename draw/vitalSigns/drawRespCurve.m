function drawRespCurve(ampl, axisT, peak, varargin)
% 绘制呼吸波形
% 输入:
% 1. ampl: 呼吸幅度数组 m
% 2. axisT: 时间戳 s
% 3. peak: 波峰或波谷的索引
% 4. varargin:
%     - code: Scarecrow系统的身份编码
% 作者: 刘涵凯
% 更新: 2024-3-14

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('code', []);
p.parse(varargin{:});
code = p.Results.code;

%% 图像参数
load colorLib.mat
lineWidth = 1.5;
markerSize1 = 4;
markerSize2 = 4;
fontSize = 12;
txtSize = 15;
marker1 = '.';
marker2 = 'v';

%% 绘图
figure
hold on
h1 = plot(axisT, ampl * 1000);
h2 = plot(peak.time, peak.value * 1000);
set(h1, 'LineStyle', '-', 'LineWidth', lineWidth, 'Marker', marker1, 'MarkerSize', markerSize1, 'Color', colorBlue);
set(h2, 'LineStyle', 'none', 'LineWidth', lineWidth, 'Marker', marker2, 'MarkerSize', markerSize2, 'Color', colorRed);
% 显示身份编码
idxCode = ~isnan(code);
if any(idxCode)
    code(~idxCode) = [];
    idxCode = find(idxCode);
    codePos = [peak.time(idxCode)' + peak.time(idxCode + 1)', ...
        (peak.value(idxCode)' + peak.value(idxCode + 1)')  / pi] / 2;
    code = string(code);
    text(codePos(:, 1) - txtSize / 50, codePos(:, 2), code, 'FontSize', txtSize)
end

%% 图像设置
xlabel('Time (s)', 'fontsize', fontSize);
ylabel('Amplitude (mm)', 'fontsize', fontSize);
set(gca, 'Xlim', axisT([1, end]))
set(gca, 'Ylim', 10 * [-1, 1])
set(gca, 'FontSize', fontSize)
set(gca, 'Box', 'off')
set(gcf, 'color', 'w')
width = 0.4 * (axisT(end) - axisT(1)); % 根据时间调整图像宽度
set(gcf, 'Units', 'centimeters', 'Position', [2 2 width 9])
% set(gcf, 'Units', 'centimeters', 'Position', [14 2 12 9]);
set(gca,'LooseInset', get(gca, 'TightInset'))
grid on
