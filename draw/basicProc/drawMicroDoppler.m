function drawMicroDoppler(varargin)
% 绘制多普勒谱
% 输入:
% varargin
% - handleRslt: 微多普勒数据文件名
% - logEn: 是否将RDM颜色幅度设为dB. 0-否; 1-是
% 作者: 刘涵凯
% 更新: 2023-6-7

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('handleRslt', 'mdRslt.mat');
p.addOptional('logEn', 1);
p.parse(varargin{:});
handleRslt = p.Results.handleRslt;
logEn = p.Results.logEn;

%% 导入微多普勒数据
load(handleRslt)

%% 计算坐标刻度
t = tFrm * (iFrmLoad - 1);
vel = resV * (-nChirp1Frm / 2 : nChirp1Frm / 2 - 1); % nChirp为偶数时, 以第 nChirp/2+1个点为0

%% 图像参数
fontSize=12;

%% 绘图
figure
imagesc(t, vel, mdRslt, 'CDataMapping', 'scaled');

%% 图像设置
xlabel('Time (s)', 'fontsize', fontSize)
ylabel('Velocity (m/s)', 'fontsize', fontSize)
set(gca, 'Xlim', [t(1) - tFrm / 2, t(end) + tFrm / 2])
set(gca, 'Ylim', [vel(1) - resV / 2, vel(end) + resV / 2])
set(gca, 'YDir', 'normal')
if logEn; set(gca, 'ColorScale', 'log'); end % 将幅度转化为dB
set(gca, 'Fontsize', fontSize)
set(gcf, 'color', 'w')
set(gca, 'Box', 'off')
width = 15 * (t(end) - t(1) + tFrm) / 10; % 根据时间调整图像宽度
set(gcf, 'Units', 'centimeters', 'Position', [2 2 width 9])
set(gca, 'LooseInset', get(gca, 'TightInset'))
