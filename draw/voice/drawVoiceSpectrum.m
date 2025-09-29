function drawVoiceSpectrum
% 绘制声谱图
% 作者: 刘涵凯
% 更新: 2022-8-15

%% 载入感知结果
load voiceResults.mat
load colorLib.mat

%% 图像参数
fontSize = 9;

%% 绘图
stm.map(1, :) = 0;
stm.map = stm.map .^ 2.5; % 用于调整图像表现
figure
h = imagesc(stm.axisT, stm.axisFreq, stm.map, 'CDataMapping', 'scaled');

xlabel('Time (s)', 'fontsize', fontSize);
ylabel('Frequency (Hz)', 'fontsize', fontSize);
set(gca, 'Xlim', stm.axisT([1, end]))
set(gca, 'Ylim', stm.axisFreq([1, end]))
set(gca,'YDir', 'normal')
set(gca, 'ColorScale', 'log')
set(gca, 'Fontsize', fontSize);
set(gca,'LooseInset', get(gca, 'TightInset'))
set(gcf, 'color', 'w')
set(gcf, 'Units', 'centimeters', 'Position', [2 10 12 6]);

% xlabel([], 'fontsize', fontSize);
% ylabel([], 'fontsize', fontSize);
% set(gca, 'xTick', [])
% set(gca, 'yTick', [])
% set(gca,'YDir', 'normal')
% set(gca, 'ColorScale', 'log')
% set(gca,'LooseInset',get(gca,'TightInset'))
% set(gcf, 'Units', 'centimeters', 'Position', [14 13 12 9]);

% fontSize=20;
% xlabel('Time', 'fontsize', fontSize);
% ylabel('Frequency', 'fontsize', fontSize);
% set(gca, 'xTick', [])
% set(gca, 'yTick', [])
% set(gca,'YDir', 'normal')
% set(gca, 'ColorScale', 'log')
% set(gca,'LooseInset',get(gca,'TightInset'))
% set(gcf, 'Units', 'centimeters', 'Position', [18 15 15 9]);