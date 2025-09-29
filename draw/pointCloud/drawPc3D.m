function drawPc3D(pc3D, varargin)
% 绘制3D位置点云
% 输入:
% 1. pc3D: 点云坐标
% 2. varargin:
%     - pcType: 点云类型. 'XYZ'; 'XYV'
%     - vel: 点云速度, 用于以颜色表示XYZ点云的速度, 在展示聚类结果时无作用
%     - pw: 点云反射强度, 用于调整点的大小
%     - limitX: 图像x坐标范围
%     - limitY: 图像y坐标范围
%     - limitZV: 图像z/v坐标范围
%     - clusterIdx: 聚类簇序号
% 作者: 刘涵凯
% 更新: 2024-6-22

%% 默认参数
load('config.mat', 'maxR', 'maxV', 'posRadar')
p = inputParser();
p.CaseSensitive = false;
p.addOptional('pcType', 'XYZ');
p.addOptional('vel', []);
p.addOptional('pw', []);
p.addOptional('limitX', []);
p.addOptional('limitY', []);
p.addOptional('limitZV', []);
p.addOptional('clusterIdx', []);
p.parse(varargin{:});
pcType = p.Results.pcType;
vel = p.Results.vel;
pw = p.Results.pw;
limitX = p.Results.limitX;
limitY = p.Results.limitY;
limitZV = p.Results.limitZV;
clusterIdx = p.Results.clusterIdx;
if ~isempty(clusterIdx); vel = []; end % 展示聚类结果时, 不根据速度设置颜色
if isempty(limitX); limitX = sind(60) * [-maxR, maxR]; end
if isempty(limitY); limitY = [0, maxR]; end
switch pcType
    case 'XYZ'
        if isempty(limitZV); limitZV = [0, 2]; end
        labelZ = 'Z (m)';
    case 'XYV'
        if isempty(limitZV); limitZV = [-maxV, maxV]; end
        labelZ = 'Velocity (m/s)';
end

%% 图像参数
load colorLib.mat
lineWidth = 1.5;
markerSize1 = 7;
markerSize2 = 25;
markerSize3 = 40;
markerSize4 = 10;
fontSize = 12;
marker1 = '^';
marker2 = '.';

%% 点云参数
nPc = size(pc3D, 1);

%% 速度、反射强度归一化
pcColor = zeros(nPc, 3);
pcSize = repelem(markerSize2, nPc, 1);
if ~isempty(vel)
    colorAxis = parula(100); % 使用parula渐变颜色图表示速度
    vel = (vel + maxV) / (2 * maxV);
    pcColor = colorAxis(max(1, ceil(vel * 100)), :);
end
if ~isempty(pw)
    sizeAxis = linspace(markerSize4, markerSize3, 100);
    pw = pw / max(pw);
    pcSize = sizeAxis(ceil(pw * 100));
end

%% 绘图
figure;
% 雷达自身
h1 = plot3(0, 0, posRadar(3));
set(h1, 'LineStyle', 'none', 'LineWidth', lineWidth, 'Marker', marker1, 'MarkerSize', markerSize1, 'Color', colorRed)
hold on
% 点云
% 首先把点云按序号画上
for iPc = 1 : nPc
    h2(iPc) = plot3(pc3D(iPc, 1), pc3D(iPc, 2), pc3D(iPc, 3));
    set(h2(iPc), 'LineStyle', 'none', 'LineWidth', lineWidth, 'Marker', marker2, 'MarkerSize', markerSize2, ...
        'Color', 'k')
end
% 设置点的大小和颜色
    for iPc = 1 : nPc
        set(h2(iPc), 'MarkerSize', pcSize(iPc), 'Color', pcColor(iPc, :))
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
xlabel('X (m)', 'fontsize', fontSize)
ylabel('Y (m)', 'fontsize', fontSize)
zlabel(labelZ, 'fontsize', fontSize)
set(gca, 'Xlim', limitX)
set(gca, 'Ylim', limitY)
set(gca, 'Zlim', limitZV)
set(gca, 'XTick', 1 * (- 20 : 20)) % 坐标刻度不听话时调这个
set(gca, 'fontsize', fontSize)
% set(gca, 'LooseInset', get(gca, 'TightInset'))
set(gcf, 'color', 'w')
% set(gcf, 'Units', 'centimeters', 'Position', [26 2 12 9])
set(get(gca, 'XLabel'), 'Rotation', 18);
set(get(gca, 'YLabel'), 'Rotation', -25);
% set(gca, 'Units', 'centimeters', 'Position', [1.2 1.2 12.6 6.6])
set(get(gca, 'XLabel'), 'Rotation', 17);
set(get(gca, 'YLabel'), 'Rotation', -22);
set(gca, 'Units', 'centimeters', 'Position', [1.4 1.2 12.4 6.6])
set(gcf, 'Units', 'centimeters', 'Position', [2 2 14 8])
grid on
