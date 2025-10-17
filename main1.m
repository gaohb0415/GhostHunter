% 功能: 自动播放，支持步进，每个图像在独立的窗口中稳定显示
% 作者: ghb
% 更新: 2025-9-27 

%% 清理环境与路径
close all; 
clear;     
addpath(genpath(pwd)); 

%% =================== 1. 控制与配置面板 ===================
% --- 播放控制 ---
cfg.startFrame = 1;      
cfg.endFrame   = 50;     
cfg.frameStep  = 5;      
cfg.pauseTime  = 0.25;   

% 这是我们为“鬼探头”场景设定的感兴趣区域 (Region of Interest)
disp('--- 启用基于ROI的增强处理 ---');
roi.range = [0, 8]; % 单位: 米
roi.angle = [-90, +90];    % 单位: 度


% =======================【场景化真值】=======================
disp('--- 加载场景化真值 (Ground Truth) ---');

% --- 遮挡车辆的边界框 (红框) ---
% 注意：为了画一个闭合的矩形，我们需要5个点（最后一个点和第一个点重合）
ground_truth.car.x = [ 1.0, 2.5, 2.5, 1.0, 1.0];
ground_truth.car.y = [ 2.0, 2.0, 4.5, 4.5, 2.0];

% --- 同学行走的路径 (红色虚线段) ---
ground_truth.path.x = [ 1.0, 2.5];
ground_truth.path.y = [ 4.75, 4.75];
% ====================================================================



% --- 图像生成开关 ---
% 你想看哪个图，就把它设为 true, 不想看就设为 false
cfg.showRangeFFT    = false;   
cfg.showRangeAngle  = true;   
cfg.show3DPointCloud= false;   
cfg.showClusteredPC = false;  
cfg.show2DTopDown   = true;    % 是否显示我们创建的2D俯视点云图

%% =================== 2. 载入雷达配置 ===================
config2243; 
fprintf('雷达配置加载完毕，开始处理数据...\n');

%% =================== 3. 循环处理与可视化 ===================
for iFrm = cfg.startFrame : cfg.frameStep : cfg.endFrame
    fprintf('正在处理第 %d 帧...\n', iFrm);
    
    try
        radarData = readBin(iFrm, 0);
    catch
        fprintf('读取第 %d 帧失败，可能已到达文件末尾。播放结束。\n', iFrm);
        break;
    end
    
    % --- 数据预处理 ---
    % 即使不画图，也要计算以供后续使用
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 1, 'drawEn', 0);
    if cfg.show3DPointCloud || cfg.showClusteredPC
        pc3D = pcFrom4DFFT(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakAz', 1, 'nPeakEl', 3, 'drawEn', 0);
    end

    %  如果需要2D俯视图，则计算pcRA
    if cfg.show2DTopDown
        % 'drawEn', 0: 只计算，不在函数内绘图
        % [~, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', [0, 8], 'resAng', 1, 'drawEn', 0);
        
        [~, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, ...
                         'limitR', roi.range, ...      % <-- 应用ROI
                         'limitAng', roi.angle, ...      % <-- 应用ROI
                         'resAng', 0.2, ...            % <-- 使用一个较好的分辨率
                         'cfarPfa', 0.25, ...          % <-- 使用为ROI调好的pfa
                         'drawEn', 0);                 % <-- 保持计算和绘图分离
    end
    
    if cfg.showRangeFFT
        figure(1); % 激活或创建1号窗口
        clf;       % 清空当前窗口
        fftRange(radarData, 'pcEn', 0, 'drawEn', 1); 
        title(['距离-FFT (帧: ', num2str(iFrm), ')']);
    end
    
    if cfg.showRangeAngle
        figure(2); % 激活或创建2号窗口
        clf;
        dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', roi.range,'limitAng',roi.angle, 'resAng', 0.2, 'drawEn', 1,'cfarPfa',0.25);
        title(['距离-角度 热力图 (帧: ', num2str(iFrm), ')']);
    end
    
    if cfg.show3DPointCloud
        figure(3); % 激活或创建3号窗口
        clf;
        pcFrom4DFFT(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakAz', 1, 'nPeakEl', 3, 'drawEn', 1);
        title(['三维点云 (帧: ', num2str(iFrm), ')']);
    end
    
    if cfg.showClusteredPC
        figure(4); % 激活或创建4号窗口
        clf;
        if exist('pc3D', 'var') && ~isempty(pc3D.x)
            pcCluster3D([pc3D.x, pc3D.y, pc3D.z], 'pw', [], 'vel', pc3D.vel, 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZV', [0, 2], 'drawEn', 1);
            title(['三维点云聚类 (帧: ', num2str(iFrm), ')']);
        else
            text(0.5, 0.5, '当前帧无有效点云', 'HorizontalAlignment', 'center');
            title(['三维点云聚类 (帧: ', num2str(iFrm), ')']);
            axis off;
        end
    end

    % =======================【升级后的2D俯视图模块】=======================
if cfg.show2DTopDown
    figure(5); % 激活或创建5号窗口
    clf;       % 清空上一帧的画面
    hold on;   % *** 关键：准备在同一张图上叠加绘制所有元素 ***

    % --- 步骤 1: 绘制真值作为背景 ---
    % 绘制红色的车辆边界框
    plot(ground_truth.car.x, ground_truth.car.y, 'r-', 'LineWidth', 2);
    % 绘制红色的虚线路径
    plot(ground_truth.path.x, ground_truth.path.y, 'r--', 'LineWidth', 2);
    
    % --- 步骤 2: 在真值背景上，绘制雷达检测到的点云 ---
    if exist('pcRA', 'var') && ~isempty(pcRA.x)
        clusterRslt2D = pcCluster2D([pcRA.x, pcRA.y], 'pw', pcRA.power, 'drawEn', 0);
        
        % [重要] 我们需要稍微修改一下drawPc2DPlus的调用方式
        % 让它不要自己创建新窗口(figure)和清空(clf)，而是直接画在我们当前的图上
        drawPointsOnExistingAxes(clusterRslt2D.pcInput, ... % 我们将创建一个新函数
                     'clusterID', clusterRslt2D.clusterIdx, ...
                     'power', clusterRslt2D.pw, ...
                     'roi', roi); 
    end
    
    % --- 步骤 3: 美化图像 ---
    plot(0, 0, 'kv', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % 绘制雷达
    hold off;
    grid on;
    axis equal; 
    xlim([-4, 4]);
    ylim([0, 8]);
    xlabel('X (m)');
    ylabel('Y (m)');
    legend('车辆边界', '行人路径', 'ROI', '检测点', '雷达'); % 你可以根据实际情况调整图例
    title(['二维俯视点云图 (帧: ', num2str(iFrm), ')']);
end
% =========================================================================
    

    % --- 循环末尾 ---
    drawnow; % 强制刷新所有打开的窗口
    if cfg.pauseTime > 0
        pause(cfg.pauseTime); 
    end
end

fprintf('播放完成。\n');