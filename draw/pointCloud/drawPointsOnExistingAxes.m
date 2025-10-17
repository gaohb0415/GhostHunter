function drawPointsOnExistingAxes(pcPoints, varargin)
% drawPointsOnExistingAxes - 在【已经存在的】图窗上绘制点云和ROI
% (这是drawPc2DPlus的精简版，专用于叠加绘制)

%% 1. 解析输入参数
p = inputParser();
p.addOptional('clusterID', []);
p.addOptional('power', []);
p.addOptional('roi', []);
p.parse(varargin{:});
clusterID = p.Results.clusterID;
power = p.Results.power;
roi = p.Results.roi;

%% 2. [新增] 绘制感兴趣区域 (ROI)
if ~isempty(roi) && isfield(roi, 'range') && isfield(roi, 'angle')
    theta = linspace(roi.angle(1), roi.angle(2), 50);
    x_fill = [roi.range(1) * sind(theta), roi.range(2) * sind(fliplr(theta))];
    y_fill = [roi.range(1) * cosd(theta), roi.range(2) * cosd(fliplr(theta))];
    fill(x_fill, y_fill, [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

%% 3. 绘制点云
if ~isempty(pcPoints)
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
            scatter(pcPoints(idx, 1), pcPoints(idx, 2), pointSizes(idx), colors(i, :), 'filled');
        end
        outlierIdx = (clusterID <= 0);
        if any(outlierIdx)
            scatter(pcPoints(outlierIdx, 1), pcPoints(outlierIdx, 2), pointSizes(outlierIdx), [0.5 0.5 0.5], 'filled');
        end
    else
        scatter(pcPoints(:, 1), pcPoints(:, 2), pointSizes, 'r', 'filled');
    end
end
end