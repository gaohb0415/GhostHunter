function drawPc2D(pc2D, varargin)
% 绘制2D点云, 含聚类结果
% 输入:
% 1. pc2D: 点云坐标
% 2. varargin:
%     - pcType: 点云类型. 'XY'; 'XV'
%     - pw: 点云反射强度, 用于调整点的大小
%     - limitXV: 图像x/v坐标范围
%     - limitY: 图像y坐标范围
%     - clusterIdx: 聚类簇序号
% 作者: 刘涵凯
% 更新: 2024-6-22

%% 默认参数
load('config.mat', 'maxR', 'maxV')
p = inputParser();
p.CaseSensitive = false;
p.addOptional('pcType', 'XY');
p.addOptional('pw', []);
p.addOptional('limitXV', []);
p.addOptional('limitY', []);
p.addOptional('clusterIdx', []);
p.parse(varargin{:});
pcType = p.Results.pcType;
pw = p.Results.pw;
limitXV = p.Results.limitXV;
limitY = p.Results.limitY;
clusterIdx = p.Results.clusterIdx;
if isempty(limitY);  limitY = [0, maxR]; end
switch pcType
    case 'XY'
        if isempty(limitXV); limitXV = sind(60) * [-maxR, maxR]; end
        labelX = 'X (m)';
        labelY = 'Y (m)';
    case 'VY'
        if isempty(limitXV); limitXV = [-maxV, maxV]; end
        labelX = 'Velocity (m/s)';
        labelY = 'Range (m)';
end

%% 图像参数
load colorLib.mat
lineWidth = 1.5;
markerSize1 = 7;
markerSize2 = 10;
markerSize3 = 20;
markerSize4 = 1;
fontSize = 12;
marker1 = '^';
marker2 = '.';

%% 点云参数
nPc = size(pc2D, 1);

%% 反射强度归一化
if ~isempty(pw)
    sizeAxis = linspace(markerSize4, markerSize3, 100);
    pw = pw / max(pw);
    pcSize = sizeAxis(ceil(pw * 100));
end

%% 绘图
figure; hold on
% 雷达自身
h1 = plot(0, 0);
set(h1, 'LineStyle', 'none', 'LineWidth', lineWidth, 'Marker', marker1, 'MarkerSize', markerSize1, 'Color', colorRed)
hold on
% 点云
% 首先把点云按序号画上
for iPc = 1 : nPc
    h2(iPc) = plot(pc2D(iPc, 1), pc2D(iPc, 2));
    set(h2(iPc), 'LineStyle', 'none', 'LineWidth', lineWidth, 'Marker', marker2, 'MarkerSize', markerSize2, ...
        'Color', 'k')
end
% for iCluster = 1 : max(clusterIdx)
if ~isempty(pw)
    for iPc = 1 : nPc
        set(h2(iPc), 'MarkerSize', pcSize(iPc))
    end
end
% 用不同颜色表示各簇
if max(clusterIdx) > 0
    for iCluster = 1 : max(clusterIdx)
        iPcTemp = clusterIdx == iCluster;
        set(h2(iPcTemp), 'Color', colorSet(iCluster, :))
    end
end
% 离群点颜色不变, 仍为黑色

%% 图像设置
xlabel(labelX, 'fontsize', fontSize)
ylabel(labelY, 'fontsize', fontSize)
set(gca, 'Xlim', limitXV)
set(gca, 'Ylim', limitY)
set(gca, 'XTick', 1 * (- 20 : 20)) % 坐标刻度不听话时调这个
set(gca, 'fontsize', fontSize)
% set(gca, 'Box', 'off')
set(gca,'LooseInset', get(gca, 'TightInset'))
set(gcf, 'color', 'w')
set(gcf, 'Units', 'centimeters', 'Position', [22 2 12 9])
grid on

set(gca, 'fontsize', 15)
% set(gca, 'Xlim', [-3.2, 3.2])
% set(gca, 'Ylim', [1.2, 6.8])
set(gca, 'XTick', 0.4 + 0.8 * (- 10 : 10))
set(gca, 'YTick', 0.8 * (- 10 : 10))
set(gca, 'Units', 'centimeters', 'Position', [1.7 1.7 12.1 6.0])
set(gcf, 'Units', 'centimeters', 'Position', [2 2 14 8])
