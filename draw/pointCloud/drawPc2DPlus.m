
% drawPc2DPlus - (最终鲁棒版) 绘制2D俯视点云图，即使没有点云数据也会生成空白图
%
% 功能:
%   - 将输入的2D点云坐标绘制成俯视图 (X-Y平面)
%   - 如果提供聚类信息，则用不同颜色区分不同的物体簇
%   - 如果提供功率信息，则用点的大小表示信号强度
%   - 自动在原点(0,0)标出雷达位置
%   - 如果输入点云为空，则生成一个带有坐标轴和雷达图标的空白图
%
% 输入:
% 1. pcPoints: 一个 N x 2 的矩阵, 包含点云的 [X, Y] 坐标。可以为空矩阵[]。
% 2. varargin (可选参数对):
%     - 'clusterID': 一个 N x 1 的向量, 包含每个点所属的簇ID。
%     - 'power': 一个 N x 1 的向量, 包含每个点的反射强度。
%     - 'limitX': X轴的显示范围, e.g., [-4, 4]
%     - 'limitY': Y轴的显示范围, e.g., [0, 8]
%
% 作者: Gemini & 刘涵凯
% 更新: 2025-10-12
function drawPc2DPlus(pcPoints, varargin)
% drawPc2DPlus - (最终鲁棒版 V2.0 - 支持ROI可视化) 绘制2D俯视点云图
%
% 功能:
%   - ... (原有功能不变) ...
%   - [新增] 如果提供ROI信息，则在背景中绘制出半透明的感兴趣区域
%
% 输入:
% 1. pcPoints: 一个 N x 2 的矩阵, 包含点云的 [X, Y] 坐标。可以为空矩阵[]。
% 2. varargin (可选参数对):
%     - ... (原有参数不变) ...
%     - 'roi': 一个结构体, 包含 .range 和 .angle 字段, 用于定义ROI
%
% 作者: Gemini & 刘涵凯
% 更新: 2025-10-14 (北京时间 2025-10-15)
%% 1. 解析输入参数
p = inputParser();
p.addOptional('clusterID', []);
p.addOptional('power', []);
p.addOptional('limitX', []);
p.addOptional('limitY', []);
p.addOptional('roi', []); % <--- [新增] 添加roi参数，默认是空的
p.parse(varargin{:});
clusterID = p.Results.clusterID;
power = p.Results.power;
limitX = p.Results.limitX;
limitY = p.Results.limitY;
roi = p.Results.roi; % <--- [新增] 获取roi的值

%% 2. 准备图窗并进行基本设置
hold on; % 允许在同一张图上叠加绘制
legendEntries = {};
legendHandles = [];
%% 3. [新增] 绘制感兴趣区域 (ROI)
if ~isempty(roi) && isfield(roi, 'range') && isfield(roi, 'angle')
    % 定义ROI的边界
    r_min = roi.range(1);
    r_max = roi.range(2);
    theta_min = roi.angle(1);
    theta_max = roi.angle(2);
    
    % 创建用于填充扇形的顶点
    theta = linspace(theta_min, theta_max, 50); % 更多的点让弧线更平滑
    
    % 从内弧线顺时针 -> 外弧线逆时针 -> 闭合路径
    x_fill = [r_min * sind(theta), r_max * sind(fliplr(theta))];
    y_fill = [r_min * cosd(theta), r_max * cosd(fliplr(theta))];
    
    % 绘制半透明的填充区域
    h_roi = fill(x_fill, y_fill, [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    legendHandles(end+1) = h_roi;
    legendEntries{end+1} = 'ROI';
end

%% 4. 检查输入数据是否有效，并绘制点云
if ~isempty(pcPoints)
    % --- (这部分代码与你原有的完全一样，无需改动) ---
    if ~isempty(power) && max(power) > min(power)
        minSize = 10; maxSize = 100;
        normPower = (power - min(power)) / (max(power) - min(power) + eps);
        pointSizes = minSize + normPower * (maxSize - minSize);
    else
        pointSizes = 36;
    end
    
    if ~isempty(clusterID) && max(clusterID) > 0
        numClusters = max(clusterID);
        colors = lines(numClusters);
        for i = 1 : numClusters
            idx = (clusterID == i);
            if ~any(idx); continue; end
            h = scatter(pcPoints(idx, 1), pcPoints(idx, 2), pointSizes(idx), colors(i, :), 'filled');
            legendHandles(end+1) = h;
            legendEntries{end+1} = ['Cluster ' num2str(i)];
        end
        outlierIdx = (clusterID <= 0);
        if any(outlierIdx)
            h = scatter(pcPoints(outlierIdx, 1), pcPoints(outlierIdx, 2), pointSizes(outlierIdx), [0.5 0.5 0.5], 'filled');
            legendHandles(end+1) = h;
            legendEntries{end+1} = 'Outliers';
        end
    else
        h = scatter(pcPoints(:, 1), pcPoints(:, 2), pointSizes, 'r', 'filled');
        legendHandles(end+1) = h;
        legendEntries{end+1} = 'Raw Points';
    end
end

%% 5. 无论有无点云，都绘制雷达图标和美化图像
h_radar = plot(0, 0, 'kv', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
legendHandles(end+1) = h_radar;
legendEntries{end+1} = 'Radar';
hold off;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
% title('2D Top-Down View (from Range-Azimuth Data)'); % 标题移动到主脚本控制

if ~isempty(legendHandles)
    legend(legendHandles, legendEntries, 'Location', 'best');
end
if ~isempty(limitX); xlim(limitX); end
if ~isempty(limitY); ylim(limitY); end
% drawnow; % drawnow 移动到主脚本的循环末尾控制
end