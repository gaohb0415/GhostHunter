% mmWaveMatlab主函数 - 动态鬼探头逻辑增强演示版
% 场景: 雷达向前运动 (0.579 m/s), 右偏 15度, 遮挡车静止
% 流程: 逐帧读取 -> 动态几何解算 -> 动态Mask生成 -> 局部Capon增强 -> 实时对比显示

%% 1. 初始化与全局配置
close all; clear; clc;
addpath(genpath(pwd));

% --- 实验动力学参数 (根据你的描述) ---
EGO_VELOCITY = 0.579;        % 雷达车速度 (m/s)
RADAR_YAW    = -15;          % 雷达安装偏角 (度, 负值代表右偏)

% --- 硬件采集参数 (根据配置文件) ---
TOTAL_FRAMES = 210;          % 总帧数
FRAME_PERIOD = 50e-3;        % 帧周期 50ms (0.05s)

% --- 算法扫描参数 (半圆全景) ---
CFG_LIMIT_ANG = [-90, 90];   % 扫描范围 -90 到 90 度
CFG_RES_ANG   = 0.5;         % 角度分辨率
CFG_LIMIT_R   = [];          % 距离范围设为空 (由Mask控制)

% --- 载入雷达基础配置 ---
config2243; 
try
    load('config.mat', 'resR', 'resV'); 
catch
    % 备用路径，防止路径报错
    load('.\config\config\config.mat', 'resR', 'resV'); 
end

% --- 构造全量雷达网格 (用于生成 Mask) ---
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);

% --- 车辆真值 (t=0时刻的世界坐标) ---
% 从代码中提取前4个点作为矩形 (去除闭合点)
% [1.65, 0.63; 3.44, 0.63; 3.44, 3.48; 1.65, 3.48]
ground_truth_world.car.x = [ 0.82, 2.85, 2.85, 0.82, 0.82];
ground_truth_world.car.y = [ 4.23, 4.23, 6.93, 6.93, 4.23];
car_init_mat = [ground_truth_world.car.x(1:4)', ground_truth_world.car.y(1:4)'];

%% 2. 准备可视化窗口
hFig = figure('Name', 'Dynamic Ghost Probe Detection', 'Position', [50, 50, 1500, 500]);

% 预计算绘图网格 (极坐标 -> 直角坐标 X/Y)
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);

%% 3. 逐帧处理循环 (Frame 1 -> 210)
for iFrm = 1 : 1 : (TOTAL_FRAMES / 2)
    
    % --- 打印进度 ---
    current_time = (iFrm - 1) * FRAME_PERIOD;
    fprintf('处理帧: %d / %d (时间: %.2fs)\n', iFrm, TOTAL_FRAMES, current_time);
    
    % =========================================================
    % Step A: 读取当前帧数据
    % =========================================================
    try
        radarData = readBin(iFrm, 0); 
    catch
        warning(['帧 ', num2str(iFrm), ' 读取失败或结束，停止循环。']);
        break;
    end
    
    % =========================================================
    % Step B: 基础 FFT (获取无损全量信号)
    % =========================================================
    % pcEn=0: 关闭CFAR，保留所有微弱信号
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0); 
    
    % =========================================================
    % Step C: 【动态核心】计算当前帧车辆坐标
    % =========================================================
    % 函数内部计算公式: y_new = y_old - velocity * time
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    
    % =========================================================
    % Step D: 【动态核心】生成当前帧阴影 Mask
    % =========================================================
    % Mask 形状会随着车辆位置变化而实时变形
    ShadowMask = generate_exact_shadow_mask(full_grid, car_pts);
    
    % =========================================================
    % Step E: 执行 Capon 增强 (对比实验)
    % =========================================================
    
    % 1. 全景模式 (Original): Mask为空，算全图
    [pwRA_Full, ~] = dbfProc1D(fftRsltRg, ...
        'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, 'limitR', CFG_LIMIT_R, ...
        'Mask', [], ... 
        'pcEn', 0, 'sigReconsEn', 0, 'velocityEn', 0, 'drawEn', 0);
    
    % 2. 增强模式 (Enhanced): 传入 ShadowMask，只算阴影区
    [pwRA_Roi, ~] = dbfProc1D(fftRsltRg, ...
        'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, 'limitR', CFG_LIMIT_R, ...
        'Mask', ShadowMask, ... 
        'pcEn', 0, 'sigReconsEn', 0, 'velocityEn', 0, 'drawEn', 0);
    
    % =========================================================
    % Step F: 动态可视化 (三图联动) - [已修改: 增加帧号显示]
    % =========================================================
    if ~ishandle(hFig), break; end 
    
    % 准备绘图网格
    [Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
    X_Plot = Rng_Grid .* sind(Ang_Grid); 
    Y_Plot = Rng_Grid .* cosd(Ang_Grid);
    
    % --- 子图1: 几何位置验证 ---
    subplot(1, 3, 1);
    cla; hold on; axis equal; grid on;
    plot(0, 0, 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % 雷达
    
    % 画车
    car_poly_x = [car_pts.all_x; car_pts.all_x(1)];
    car_poly_y = [car_pts.all_y; car_pts.all_y(1)];
    fill(car_poly_x, car_poly_y, [0.8 0.8 0.8], 'FaceAlpha', 0.5); 
    plot(car_poly_x, car_poly_y, 'k-', 'LineWidth', 1.5);
    
    % 标出关键点
    plot(car_pts.A.x, car_pts.A.y, 'ro', 'MarkerFaceColor', 'r'); 
    plot(car_pts.B.x, car_pts.B.y, 'bo', 'MarkerFaceColor', 'b'); 
    
    % Mask 轮廓
    contour(X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'm--', 'LineWidth', 1.5);
    
    xlim([-8, 8]); ylim([0, 12]); 
    xlabel('X (m)'); ylabel('Y (m)');
    % 【修改点】标题增加 Frame 显示
    title(sprintf('1. 几何模型 | Frame: %d | t=%.2fs', iFrm, current_time), 'FontSize', 12, 'FontWeight', 'bold');
    
    % --- 子图2: 原始全景 ---
    subplot(1, 3, 2);
    h1 = pcolor(X_Plot, Y_Plot, pwRA_Full);
    set(h1, 'EdgeColor', 'none'); shading interp;
    hold on;
    contour(X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    
    axis equal; grid on; colormap('jet');
    xlim([-11, 11]); ylim([0, 12]); 
    xlabel('X (m)'); ylabel('Y (m)');
    % 【修改点】标题增加 Frame 显示
    title(sprintf('2. 原始全景 | Frame: %d', iFrm), 'FontSize', 12);
    
    % --- 子图3: 逻辑增强 ---
    subplot(1, 3, 3);
    h2 = pcolor(X_Plot, Y_Plot, pwRA_Roi);
    set(h2, 'EdgeColor', 'none'); shading interp;
    hold on;
    contour(X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    plot(car_poly_x, car_poly_y, 'm-', 'LineWidth', 2);
    
    axis equal; grid on; colormap('jet');
    xlim([-11, 11]); ylim([0, 12]); 
    xlabel('X (m)'); ylabel('Y (m)');
    % 【修改点】标题增加 Frame 显示
    title(sprintf('3. 逻辑增强 | Frame: %d', iFrm), 'FontSize', 12, 'Color', 'r');
    
    drawnow;
end