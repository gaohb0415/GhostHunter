function generate2DTopDownView(figNum, pcRA, rotated_ground_truth, roi, iFrm)
% GENERATE2DTOPDOWNVIEW - 生成包含真值和ROI的2D俯视点云图 (无图例版)
%
% 输入:
%   figNum:               希望在哪个图窗窗口中绘制 (e.g., 5)
%   pcRA:                 由dbfProc1D生成的2D点云数据结构体
%   rotated_ground_truth: 经过旋转变换后的真值结构体
%   roi:                  感兴趣区域结构体
%   iFrm:                 当前的帧号 (用于标题)

%% 1. 准备舞台
figure(figNum); % 激活或创建指定的窗口
clf;          % 清空窗口内容
hold on;      % 准备叠加绘制

%% 2. 绘制静态真值背景
if isfield(rotated_ground_truth, 'car')
    % 直接绘图，不再需要接收句柄h_car
    plot(rotated_ground_truth.car.x, rotated_ground_truth.car.y, 'r-', 'LineWidth', 2);
end
if isfield(rotated_ground_truth, 'path')
    % 直接绘图，不再需要接收句柄h_path
    plot(rotated_ground_truth.path.x, rotated_ground_truth.path.y, 'r--', 'LineWidth', 2);
end

%% 3. 绘制雷达检测到的动态点云
if exist('pcRA', 'var') && ~isempty(pcRA.x)
    clusterRslt2D = pcCluster2D([pcRA.x, pcRA.y], 'pw', pcRA.power, 'drawEn', 0, 'minpts', 1);
    
    % 调用绘图函数，但不再需要接收它的返回值
    drawPointsOnExistingAxes(clusterRslt2D.pcInput, ...
                 'clusterID', clusterRslt2D.clusterIdx, ...
                 'power', clusterRslt2D.pw, ...
                 'roi', roi); 
else 
    % 即使没有点云，也画出ROI背景
    drawPointsOnExistingAxes([], 'roi', roi); 
    disp(['帧 ', num2str(iFrm), ': 未检测到2D点云。']);
end

%% 4. 美化图像 (已移除legend命令)
plot(0, 0, 'kv', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % 绘制雷达
hold off;
grid on;
axis equal; 
xlim([-4, 4]);
ylim([0, 8]);
xlabel('X (m)');
ylabel('Y (m)');
       
title(['二维俯视点云图 (帧: ', num2str(iFrm), ')']);

end