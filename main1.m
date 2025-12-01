% 功能: 自动播放，支持步进，每个图像在独立的窗口中稳定显示
% 作者: ghb
% 更新: 2025-9-27

%% 清理环境与路径
close all;
clear;
addpath(genpath(pwd));

%% =================== 1. 控制与配置面板 ===================
% --- 播放控制 ---
cfg.startFrame = 148;
cfg.endFrame   = 148;
cfg.frameStep  = 1;
cfg.pauseTime  = 0.1;


% =======================【新增：模式控制开关】=======================
% 在这里选择你的实验模式:
% true  = 动态模式 (雷达在小推车上移动)
% false = 静态模式 (雷达固定在原点，模拟静止测量)
cfg.isDynamicMode = true; % <--- 这是你的总开关！
% ====================================================================

% =======================【新增：自车(小推车)模拟参数】=======================
% 只有在 isDynamicMode = true 时才会被使用
cfg.ego_start_x = 0;      % 假设小推车从世界坐标的X=0处开始
cfg.ego_start_y = 0;      % 假设小推车从世界坐标的Y=0处开始
% 【修改】假设小推车以 1.0 米/秒 的速度沿Y轴（雷达朝向）前进
cfg.ego_velocity_x = 0;   
cfg.ego_velocity_y = 0.579; 
% ====================================================================



% 这是我们为“鬼探头”场景设定的感兴趣区域 (Region of Interest)
disp('--- 启用基于ROI的增强处理 ---');
roi.range = [0, 8]; % 单位: 米
roi.angle = [-90, +90];    % 单位: 度

% 【修改】雷达(小推车)的“初始”朝向
% 无论动态还是静态，这都是雷达安装的初始角度
cfg.ego_start_yaw_deg = -15; %向左为正


% 真值车辆
ground_truth_world.car.x = [ 0.82, 2.85, 2.85, 0.82, 0.82];
ground_truth_world.car.y = [ 4.23, 4.23, 6.93, 6.93, 4.23];
% 行人路径
ground_truth_world.path.x = [ 2.85, 0];
ground_truth_world.path.y = [ 8.43, 8.43];



% --- 图像生成开关 ---
% 你想看哪个图，就把它设为 true, 不想看就设为 false
cfg.showRangeFFT    = true;
cfg.showRangeAngle  = true;
cfg.show3DPointCloud= false;
cfg.showClusteredPC = false;
cfg.show2DTopDown   = false; 
cfg.showRangeDoppler = false;


% =======================【场景化真值】=======================
disp('--- 加载场景化真值 (Ground Truth) ---');

% --- 遮挡车辆的边界框 (红框) ---
% 注意：为了画一个闭合的矩形，我们需要5个点（最后一个点和第一个点重合
% --- 同学行走的路径 (红色虚线段) ---
% ====================================================================

fprintf('雷达水平旋转角度: %d deg\n', cfg.ego_start_yaw_deg);
rotated_ground_truth = transform_ground_truth(ground_truth_world, cfg.ego_start_yaw_deg);   


% 缓存配置
cfg.cacheDir = 'pcRA_cache'; % 用于存储 pcRA 结果的文件夹


%% =================== 2. 载入雷达配置 ===================

config2243;
fprintf('雷达配置加载完毕，开始处理数据...\n');


% --- 【新增】 加载 tFrm (帧周期) 以便计算时间 ---
% tFrm 是由 config2243.m 保存到 config.mat 中的
try
    load('config.mat', 'tFrm');
    fprintf('已加载帧周期: tFrm = %.4f s\n', tFrm);
catch
    error('无法从 config.mat 加载 tFrm，请确保 config2243.m 运行正确。');
end



% --- 创建缓存目录 ---
if ~exist(cfg.cacheDir, 'dir')
    mkdir(cfg.cacheDir);
    fprintf('已创建缓存目录: %s\n', cfg.cacheDir);
end



%% =================== 3. 循环处理与可视化 ===================
for iFrm = cfg.startFrame : cfg.frameStep : cfg.endFrame
    fprintf('正在处理第 %d 帧...\n', iFrm);
    try
        radarData = readBin(iFrm, 0);
    catch
        fprintf('读取第 %d 帧失败，可能已到达文件末尾。播放结束。\n', iFrm);
        break;
    end


    % =======================【新增：动态/静态 真值处理模块】=======================
    if cfg.isDynamicMode
        % --- 动态模式 ---
        % 1. 计算当前时间
        % (iFrm-1) 是因为第1帧(startFrame)的时间戳是0
        currentTime = (iFrm - cfg.startFrame) * tFrm; 
        
        % 2. 计算小推车在世界坐标系下的【当前位置】
        % 假设匀速直线运动
        current_ego_x = cfg.ego_start_x + cfg.ego_velocity_x * currentTime;
        current_ego_y = cfg.ego_start_y + cfg.ego_velocity_y * currentTime;
        % 假设小推车朝向不变
        current_ego_yaw_deg = cfg.ego_start_yaw_deg;

        % 3. 将【世界坐标系】的真值，转换到【小推车当前的第一人称视角】
        % 我们需要一个临时的结构体来存储这一帧的相对真值
        gt_to_plot = struct();
        
        % 转换车辆
        [gt_car_x_radar, gt_car_y_radar] = world_to_radar_coords(...
            ground_truth_world.car.x, ground_truth_world.car.y, ...
            current_ego_x, current_ego_y, current_ego_yaw_deg);
        gt_to_plot.car.x = gt_car_x_radar;
        gt_to_plot.car.y = gt_car_y_radar;
        
        % 转换路径
        [gt_path_x_radar, gt_path_y_radar] = world_to_radar_coords(...
            ground_truth_world.path.x, ground_truth_world.path.y, ...
            current_ego_x, current_ego_y, current_ego_yaw_deg);
        gt_to_plot.path.x = gt_path_x_radar;
        gt_to_plot.path.y = gt_path_y_radar;
        
    else
        % --- 静态模式 ---
        % 直接使用在循环外计算好的、旋转一次的固定真值
        gt_to_plot = rotated_ground_truth;
    end
    % ====================================================================




    % --- 数据预处理 ---


    % 最初的数据处理（一切的起点）
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 1, 'drawEn', 0);


    % 提前处理数据进行4DFFT变换
    if cfg.show3DPointCloud || cfg.showClusteredPC
        pc3D = pcFrom4DFFT(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakAz', 1, 'nPeakEl', 3, 'drawEn', 0);
    end


    % --- 2D俯视点云图，则计算或加载 pcRA（缓存功能） ---
    if cfg.show2DTopDown
        % 1. 定义当前帧的缓存文件路径
        %    使用 sprintf('%04d', iFrm) 确保文件名对齐 (e.g., 0001, 0002, ..., 0210)
        cacheFile = fullfile(cfg.cacheDir, sprintf('pcRA_frame_%04d.mat', iFrm));

        % 2. 检查缓存是否存在
        if exist(cacheFile, 'file')
            % 2a. 缓存命中：直接加载
            fprintf('  -> 缓存命中，加载: %s\n', cacheFile);
            load(cacheFile, 'pcRA');
        else
            % 2b. 缓存未命中：计算，然后保存
            fprintf('  -> 缓存未命中，正在计算 pcRA (帧 %d)...\n', iFrm);
            
            % [这是原来的计算代码]
            [~, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, ...
                'limitR', roi.range, ...      % <-- 应用ROI
                'limitAng', roi.angle, ...      % <-- 应用ROI
                'resAng', 0.2, ...            % <-- 使用一个较好的分辨率
                'drawEn', 0);                 % <-- 保持计算和绘图分离

            % [新增] 保存到缓存
            save(cacheFile, 'pcRA');
        end
    end


    % =======================【2D点云俯视图 (代码复用)】=======================
    % 【修改】此模块现在完全复用，它只负责绘制 gt_to_plot
    % 而 gt_to_plot 变量的内容由上面的【动态/静态 真值处理模块】决定
    if cfg.show2DTopDown
        figure(5); % 激活或创建5号窗口
        clf;       % 清空上一帧的画面
        hold on;   % *** 关键：准备在同一张图上叠加绘制所有元素 ***

        % --- 步骤 1: 绘制【当前帧】的真值作为背景 ---
        % 【修改】
        % 不再绘制固定的 rotated_ground_truth
        % 而是绘制我们刚刚在上面 if/else 块中计算出的 gt_to_plot
        h_car = plot(gt_to_plot.car.x, gt_to_plot.car.y, 'r-', 'LineWidth', 2);
        h_path = plot(gt_to_plot.path.x, gt_to_plot.path.y, 'r--', 'LineWidth', 2);

        % --- 步骤 2: 在真值背景上，绘制雷达检测到的点云 ---
        % (这部分代码保持不变)
        if exist('pcRA', 'var') && ~isempty(pcRA.x)
            clusterRslt2D = pcCluster2D([pcRA.x, pcRA.y], 'pw', pcRA.power, 'drawEn', 0);

            drawPointsOnExistingAxes(clusterRslt2D.pcInput, ... 
                'clusterID', clusterRslt2D.clusterIdx, ...
                'power', clusterRslt2D.pw, ...
                'roi', roi);
        end

        % --- 步骤 3: 美化图像 ---
        % (这部分代码保持不变)
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
    % =========================================================================

    


    if cfg.showRangeFFT
        % figure(1); % 激活或创建1号窗口
        % clf;       % 清空当前窗口

        [fftRsltRg, pcRg] = fftRange(radarData, 'pcEn', 1, 'drawEn', 0);

        fftPower = matExtract(abs(fftRsltRg), 1, [0, 0, 0, 0]);
        load('config.mat', 'cfarParamRg');
        cfarParamRg.method = 'OS';
        cfarParamRg.rank = 12;
        [~, cfarTh] = cfar1D(fftPower, cfarParamRg);

        pcIdx = pcRg.iRange;

        figure(1); % 锚定1号窗口
        ax = gca;

        drawRangeFFTnew(fftPower, ...
                 'pcIdx', pcIdx, ...
                 'cfarTh', cfarTh, ...
                 'logEn', 1, ...
                 'ax', ax);
       
        title(['距离-FFT (帧: ', num2str(iFrm), ')']);
    end

    if cfg.showRangeAngle

        % hold on;

        [pwRA, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', roi.range,'limitAng',roi.angle, 'resAng', 0.2, 'drawEn', 0);
        
        load('config.mat', 'resR', 'spacingCal', 'resV');
        resAng_param = 0.2; % 确保这个值与上面 'resAng' 的输入值完全相同
        ang = (roi.angle(1) : resAng_param : roi.angle(2))';
        nRg_total = size(fftRsltRg, 1);       % 获取距离FFT后的总点数
        rg_total = resR * (0 : nRg_total - 1)'; % 计算完整的距离刻度
        valid_indices = rg_total >= roi.range(1) & rg_total <= roi.range(2);
        rg = rg_total(valid_indices);

        figure(2); % 激活或创建2号窗口
        
        ax = gca;

        drawRAMnew(pwRA, rg, ang, 'pcRA', pcRA, 'logEn', 0, 'ax', ax);
        
        title(['距离-角度 热力图 (帧: ', num2str(iFrm), ')'],'FontSize',10);

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



   % =======================【新增：Range-Doppler (RD) 图】=======================
    % 策略：调用 fftDoppler 只计算不画图 (drawEn=0)，避免弹出新窗口
    % 然后在 figure(6) 中手动复现 drawRDM 的绘图逻辑
    if cfg.showRangeDoppler
        figure(6); 
        clf;       % 清空 6 号窗口，确保画面不叠加
        ax = gca;  % 获取当前句柄

        % --- 1. 纯计算 (禁止内部绘图) ---
        [fftRsltRD, pcRD] = fftDoppler(fftRsltRg, 'pcEn', 1, 'drawEn', 0);

        % --- 2. 准备绘图数据 ---
        % 提取能量矩阵 (复现 drawRDM 内部逻辑)
        pwRD = matExtract(abs(fftRsltRD), [1, 2], [0, 0, 0]); 
        
        % 加载分辨率参数
        load('config.mat', 'resR', 'resV');
        
        % 计算坐标轴 (复现 drawRDM 内部逻辑)
        [nRg_rd, nVel_rd] = size(pwRD);
        rg_axis  = resR * (0 : nRg_rd - 1);               % 距离轴
        vel_axis = resV * (-nVel_rd / 2 : nVel_rd / 2 - 1); % 速度轴

        % --- 3. 手动绘图 (完全仿照 drawRDM，但在当前 figure(6) 中画) ---
        % 绘制热力图
        imagesc(vel_axis, rg_axis, pwRD, 'CDataMapping', 'scaled');
        
        % 绘制点云 (如果有)
        if ~isempty(pcRD.iRange)
            hold on;
            % 注意：pcRD.iVelocity 和 pcRD.iRange 是索引，需转为实际坐标
            plot(vel_axis(pcRD.iVelocity), rg_axis(pcRD.iRange), '.', 'Color', 'r');
            hold off;
        end

        % --- 4. 图像美化 (仿照 drawRDM 设置) ---
        xlabel('速度 (m/s)', 'fontsize', 12);
        ylabel('距离 (m)', 'fontsize', 12);
        title(['距离-多普勒图 (帧: ', num2str(iFrm), ')']);
        
        set(gca, 'YDir', 'normal'); % 确保Y轴方向正确（从小到大）
        set(gca, 'Xlim', [vel_axis(1) - resV/2, vel_axis(end) + resV/2]);
        set(gca, 'Ylim', [rg_axis(1) - resR/2, rg_axis(end) + resR/2]);
        set(gca, 'ColorScale', 'log'); % 开启对数显示，看清弱目标
        
    end
    % ============================================================================


    % --- 循环末尾 ---
    drawnow; % 强制刷新所有打开的窗口
    if cfg.pauseTime > 0
        pause(cfg.pauseTime);
    end
end

fprintf('播放完成。\n');