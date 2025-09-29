% 功能: 自动播放，支持步进，每个图像在独立的窗口中稳定显示
% 作者: 刘涵凯
% 更新: 2025-9-27 (由Gemini修正)

%% 清理环境与路径
close all; 
clear;     
addpath(genpath(pwd)); 

%% =================== 1. 控制与配置面板 ===================
% --- 播放控制 ---
cfg.startFrame = 1;      
cfg.endFrame   = 50;     
cfg.frameStep  = 3;      
cfg.pauseTime  = 0.5;   

% --- 图像生成开关 ---
% 你想看哪个图，就把它设为 true, 不想看就设为 false
cfg.showRangeFFT    = false;   
cfg.showRangeAngle  = true;   
cfg.show3DPointCloud= false;   
cfg.showClusteredPC = false;   

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

    % --- [核心修改] 每个图都在自己独立的窗口(Figure)中绘制 ---
    
    if cfg.showRangeFFT
        figure(1); % 激活或创建1号窗口
        clf;       % 清空当前窗口
        fftRange(radarData, 'pcEn', 0, 'drawEn', 1); 
        title(['距离-FFT (帧: ', num2str(iFrm), ')']);
    end
    
    if cfg.showRangeAngle
        figure(2); % 激活或创建2号窗口
        clf;
        dbfProc1D(fftRsltRg, 'pcEn', 0, 'limitR', [0, 8], 'resAng', 1, 'drawEn', 1);
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
    
    % --- 循环末尾 ---
    drawnow; % 强制刷新所有打开的窗口
    if cfg.pauseTime > 0
        pause(cfg.pauseTime); 
    end
end

fprintf('播放完成。\n');