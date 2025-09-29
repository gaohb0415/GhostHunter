function drawTracking(varargin)
% 绘制追踪结果
% 作者: 刘涵凯
% 更新: 2023-7-26

%% 默认参数
param = inputParser();
param.CaseSensitive = false;
param.addOptional('idxCal', 0); % 0-无; 1-4.24; 2-5.5; 3-5.9; 4-7.17空旷; 5-7.17复杂
param.addOptional('pcEn', 0);
param.addOptional('shape', 'square'); % 'square' 'rectangle'
param.addOptional('tickEn', 1);
param.addOptional('mkrStyle', 'ltr'); % 'num' 'ltr'
param.addOptional('gradEn', 1);
param.parse(varargin{:});
idxCal = param.Results.idxCal;
pcEn = param.Results.pcEn;
shape = param.Results.shape;
tickEn = param.Results.tickEn;
mkrStyle = param.Results.mkrStyle;
gradEn = param.Results.gradEn;

%% 载入数据
load trackingResults.mat
load colorLib.mat
p = drawTrackingParam('idxCal', idxCal, 'shape', shape, 'tickEn', tickEn);

%% 绘图初始化
close all; figure; set(gca, 'Box', 'on'); set(gcf, 'color', 'w'); hold on; grid on
plot(0, 0, 'LineStyle', 'none', 'Marker', p.mkrRadar, 'MarkerSize', p.mkrSizeRadar, 'Color', colorRed); % 雷达自身

%% 图框设置
if strcmp(shape, 'square'); axis equal; end
if ~tickEn; xticklabels([]); yticklabels([]); end
set(gca, 'fontsize', p.fontSize)
xlim(p.xLim); ylim(p.yLim)
xticks(p.xTick); yticks(p.yTick)
xlabel(p.xLabel); ylabel(p.yLabel)
xtickangle(0)
set(gca, 'LooseInset', get(gca, 'TightInset'))
set(gca, 'Units', 'centimeters', 'Position', p.gca)
set(gcf, 'Units', 'centimeters', 'Position', p.gcf)

%% 更新绘图
for iFrm = [1 : 2 : length(iFrmLoad) - 1, length(iFrmLoad)]
    % for iFrm = 120

    %% 坐标校准
    trajs = trajectory(iFrm).track;
    nTraj = structLength(trajs, 'iPeople');
    if ~nTraj; continue; end
    for iTraj = 1 : nTraj % trajectory和pcLast
        trajs(iTraj).trajectory = coordTransform2D(trajs(iTraj).trajectory, p.calRot, p.calDisp);
        trajs(iTraj).pcLast = coordTransform2D(trajs(iTraj).pcLast, p.calRot, p.calDisp);
    end
    clusters(iFrm).noise.pc = coordTransform2D(clusters(iFrm).noise.pc, p.calRot, p.calDisp);
    for iCluster = 1 : structLength(clusters(iFrm).cluster, 'centroid')
        clusters(iFrm).cluster(iCluster).pc = coordTransform2D(clusters(iFrm).cluster(iCluster).pc, p.calRot, p.calDisp);
    end

    if pcEn
        %% 全部点云
        % 离群点(黑色)
        if exist('hPcNoise', 'var'); delete(hPcNoise); end
        if ~isempty(clusters(iFrm).noise.pc)
            hPcNoise = plot(clusters(iFrm).noise.pc(:, 1), clusters(iFrm).noise.pc(:, 2), 'LineStyle', 'none', 'Marker', p.mkrPc, 'MarkerSize', p.mkrSizePc, 'Color', 'k');
        end
        % 簇(灰色)
        if exist('hPcCluster', 'var'); delete(hPcCluster); end
        if max(clusters(iFrm).clusterIdx) > 0
            pcCluster = vertcat(clusters(iFrm).cluster.pc);
            hPcCluster = plot(pcCluster(:, 1), pcCluster(:, 2), 'LineStyle', 'none', 'Marker', p.mkrPc, 'MarkerSize', p.mkrSizePc, 'Color', p.color.N);
        end

        %% 确立轨迹点云
        if exist('hPcConfirm', 'var'); delete(hPcConfirm); end
        for iTraj = 1 : nTraj
            if any(strcmp(trajs(iTraj).status, ["active", "overlap", "deviate"]))
                hPcConfirm(iTraj) = plot(trajs(iTraj).pcLast(:, 1), trajs(iTraj).pcLast(:, 2), 'LineStyle', 'none', 'Marker', p.mkrPc, 'MarkerSize', p.mkrSizePc);
                switch mkrStyle
                    case 'ltr'
                        set(hPcConfirm(iTraj), 'Color', p.color.(trajs(iTraj).name));
                    case 'num'
                        set(hPcConfirm(iTraj), 'Color', p.color.(strcat('num', num2str(trajs(iTraj).iPeople))));
                end
            end
        end
    end

    %% 轨迹
    if exist('hPath', 'var'); delete(hPath); end
    for iTraj = 1 : nTraj
        hPath(iTraj) = plot(trajs(iTraj).trajectory(:, 1), trajs(iTraj).trajectory(:, 2), 'LineStyle', '-', 'LineWidth', p.lineWidth);
        if strcmp(trajs(nTraj).status, "lost")
            c = p.color.lost;
        else
            switch mkrStyle
                case 'ltr'
                    c = p.color.(trajs(iTraj).name);
                case 'num'
                    c = p.color.(strcat('num', num2str(trajs(iTraj).iPeople)));
            end
        end
        set(hPath(iTraj), 'Color', c);
        if gradEn
            grad = ones(length(trajs(iTraj).frame), 1) ;
            if length(grad) > p.colorGradProtect
                grad(1 : end - p.colorGradProtect) = max(p.colorGradFac1, (trajs(iTraj).frame(1 : end - p.colorGradProtect) / (iFrm - p.colorGradProtect))  .^ p.colorGradFac2);
            end
            cGrad{iTraj} = 1 - (1 - c) .* grad;
            cGrad{iTraj} = uint8([cGrad{iTraj} * 255, 100 * ones(size(trajs(iTraj).trajectory, 1), 1)].');
        end
    end

    %% Marker
    if exist('hMkr', 'var'); delete(hMkr); end
    for iTraj = 1 : nTraj
        xLow = trajs(iTraj).trajectory(end, 1) - p.mkrSizeTgt(1) / 2; % //Left edge of marker
        xHigh = trajs(iTraj).trajectory(end, 1) + p.mkrSizeTgt(1) / 2; % //Right edge of marker
        yLow = trajs(iTraj).trajectory(end, 2) - p.mkrSizeTgt(2) / 2; % //Bottom edge of marker
        yHigh = trajs(iTraj).trajectory(end, 2) + p.mkrSizeTgt(2) / 2; % //Top edge of marker

        switch mkrStyle
            case 'ltr'
                if strcmp(trajs(nTraj).status, "lost")
                    hMkr(iTraj) =  imagesc([xLow xHigh], [yLow yHigh], flipud(p.mkrTgt.(trajs(iTraj).name).lost.marker), 'AlphaData', p.mkrTgt.(trajs(iTraj).name).lost.alpha);
                else
                    hMkr(iTraj) =  imagesc([xLow xHigh], [yLow yHigh], flipud(p.mkrTgt.(trajs(iTraj).name).confirm.marker), 'AlphaData', p.mkrTgt.(trajs(iTraj).name).confirm.alpha);
                end
            case 'num'
                numStr = strcat('num', num2str(trajs(iTraj).iPeople));
                if strcmp(trajs(nTraj).status, "lost")
                    hMkr(iTraj) =  imagesc([xLow xHigh], [yLow yHigh], flipud(p.mkrTgt.(numStr).lost.marker), 'AlphaData', p.mkrTgt.(numStr).lost.alpha);
                else
                    hMkr(iTraj) =  imagesc([xLow xHigh], [yLow yHigh], flipud(p.mkrTgt.(numStr).confirm.marker), 'AlphaData', p.mkrTgt.(numStr).confirm.alpha);
                end
        end
    end

    %% 渐变
    % 放在此处是因为通过设置Edge实现渐变必须放在drawnow之后才能看到, 而提前drawnow会影响图层优先级
    drawnow;
    if gradEn
        for iTraj = 1 : nTraj
            set(hPath(iTraj).Edge, 'ColorBinding', 'interpolated', 'ColorData', cGrad{iTraj})
        end
    end
    drawnow;
end

%% 最后展示以时间顺序堆叠的最终轨迹
grad = ones(length(iFrmLoad), 1) ;
if ~nTraj; return; end
if length(grad) > p.colorGradProtect
    grad(1 : end - p.colorGradProtect) = max(p.colorGradFac1, ((1 : length(iFrmLoad) - p.colorGradProtect)' / (length(iFrmLoad) - p.colorGradProtect))  .^ p.colorGradFac2);
end
traj = trajectory(iFrmLoad(end)).track;
% 坐标校准
nTraj = structLength(traj, 'iPeople');
for iTraj = 1 : nTraj % trajectory和pcLast
    traj(iTraj).trajectory = coordTransform2D(traj(iTraj).trajectory, p.calRot, p.calDisp);
    traj(iTraj).pcLast = coordTransform2D(traj(iTraj).pcLast, p.calRot, p.calDisp);
end
if exist('hPath', 'var'); delete(hPath); end
for iFrm = iFrmLoad
    [~, idx] = (arrayfun(@(x) ismember(iFrm, x.frame), traj));
    for iTraj = find(idx)
        if strcmp(trajs(nTraj).status, "lost")
            c = p.color.lost;
        else
            switch mkrStyle
                case 'ltr'
                    c = p.color.(trajs(iTraj).name);
                case 'num'
                    c = p.color.(strcat('num', num2str(trajs(iTraj).iPeople)));
            end
        end
        hLast = plot(traj(iTraj).trajectory(idx(iTraj), 1), traj(iTraj).trajectory(idx(iTraj), 2), 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', p.mkrSizeTraj, 'Color', c);
        if gradEn
            set(hLast, 'Color', 1 - (1 - c) * grad(iFrm - iFrmLoad(1) + 1));
        end
    end
end
uistack(hMkr, 'top') % 将Marker置于顶部
