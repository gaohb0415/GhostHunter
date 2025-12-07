% mmWaveMatlab主函数 - 最终全功能版 (含行人轨迹 GT)
% 功能: 10秒动态鬼探头 + 逻辑增强 + 行人真值轨迹叠加
% 场景: 雷达向前运动，右偏15度，车静止，人沿预定轨迹运动

%% 1. 初始化与配置
close all; clear; clc;
addpath(genpath(pwd));

% --- 动力学参数 ---
EGO_VELOCITY = 0.538;        % 雷达车速度 (m/s)
RADAR_YAW    = -30;          % 雷达安装偏角 (度)
TOTAL_FRAMES = 210; 
FRAME_PERIOD = 50e-3;

% --- 扫描参数 ---
CFG_LIMIT_ANG = [-90, 90]; 
CFG_RES_ANG   = 0.5; 
CFG_LIMIT_R   = [];

% --- 载入配置 ---
config2243; 
try load('config.mat', 'resR', 'resV'); 
catch, load('.\config\config\config.mat', 'resR', 'resV'); end

% --- 网格构建 ---
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);



% =====================【更新：最新实验真值数据】=====================

% 1. 车辆真值 (矩形4点, t=0世界坐标)
% [Updated based on user input]
ground_truth_world.car.x = [ 1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [ 5.38, 5.38, 10.11, 10.11];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];

% 2. 行人轨迹真值 (L形折线, t=0世界坐标)
% 路径点: (0, 11.87) -> (4.47, 11.87) -> (4.47, 9.3)
ped_path_x = [0,    4.47,  4.47]; 
ped_path_y = [11.87, 11.87, 9.3];

% 合并为矩阵 [N x 2]
ped_init_mat = [ped_path_x', ped_path_y'];

% ===============================================================



%% 2. 预创建绘图对象
hFig = figure('Name', 'Ghost Probe Final Demo', 'Position', [50, 50, 1400, 900]);

% === 子图1: 几何模型 ===
ax1 = subplot(2, 2, 1); hold on; axis equal; grid on;
plot(0, 0, 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
h_car_fill = fill(nan, nan, [0.8 0.8 0.8], 'FaceAlpha', 0.5);
h_car_edge = plot(nan, nan, 'k-', 'LineWidth', 1.5);
h_pt_A = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r');
h_pt_C = plot(nan, nan, 'go', 'MarkerFaceColor', 'g');
% 【新增】几何图里的行人轨迹
h_ped_geom = plot(nan, nan, 'g--', 'LineWidth', 2); 
title_str1 = title('1. 几何模型');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');

% === 子图2: Range-FFT ===
ax2 = subplot(2, 2, 2); hold on; grid on;
h_rfft_line = plot(nan, nan, 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
h_xline_A = xline(nan, 'r--', 'A', 'LineWidth', 1, 'LabelVerticalAlignment', 'bottom');
h_xline_K = xline(nan, 'm--', 'K', 'LineWidth', 1, 'LabelVerticalAlignment', 'bottom');
xlim([0, 10]); ylim([40, 120]); 
xlabel('距离 (m)'); ylabel('幅度 (dB)');
title('2. Range-FFT 能量分布');

% === 子图3: 原始热力图 ===
ax3 = subplot(2, 2, 3); hold on; axis equal; grid on;
h_pcolor1 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor1, 'EdgeColor', 'none'); shading interp; colormap(ax3, 'jet');
% 【新增】原始图叠加行人轨迹
h_ped_overlay1 = plot(nan, nan, 'w:', 'LineWidth', 2.5); % 白色点线
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title('3. 原始全景 (含车身)');

% === 子图4: 逻辑增强热力图 ===
ax4 = subplot(2, 2, 4); hold on; axis equal; grid on;
h_pcolor2 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor2, 'EdgeColor', 'none'); shading interp; colormap(ax4, 'jet');
h_car_overlay = plot(nan, nan, 'm-', 'LineWidth', 2);
% 【新增】增强图叠加行人轨迹
h_ped_overlay2 = plot(nan, nan, 'w:', 'LineWidth', 2.5); % 白色点线
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str4 = title('4. 逻辑增强');

%% 3. 极速循环
for iFrm = 75 : 2 : TOTAL_FRAMES
    
    current_time = (iFrm - 1) * FRAME_PERIOD;
    
    % --- 计算部分 ---
    try radarData = readBin(iFrm, 0); catch, break; end
    
    % 1. FFT
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0); 
    
    % 2. 坐标计算 (车)
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    
    % 3. 【核心新增】坐标计算 (人)
    % 直接调用通用变换函数，把行人轨迹也转到当前雷达坐标系
    ped_pts_radar = transform_world_to_radar(ped_init_mat, EGO_VELOCITY, RADAR_YAW, current_time);
    
    % 4. Mask 生成 (通用减法模型)
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
    PHASE_NAME = 'Universal Subtraction Model';
    
    % 5. Capon
    [pwRA_Full, ~] = dbfProc1D(fftRsltRg, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', [], 'pcEn', 0, 'drawEn', 0);
    [pwRA_Roi, ~] = dbfProc1D(fftRsltRg, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', ShadowMask, 'pcEn', 0, 'drawEn', 0);
        
    % --- 绘图部分 ---
    if ~ishandle(hFig), break; end 
    
    % 更新车身
    car_x = [car_pts.all_x; car_pts.all_x(1)];
    car_y = [car_pts.all_y; car_pts.all_y(1)];
    set(h_car_fill, 'XData', car_x, 'YData', car_y);
    set(h_car_edge, 'XData', car_x, 'YData', car_y);
    set(h_car_overlay, 'XData', car_x, 'YData', car_y);
    set(h_pt_A, 'XData', car_pts.A.x, 'YData', car_pts.A.y);
    set(h_pt_C, 'XData', car_pts.C.x, 'YData', car_pts.C.y);
    
    % 【新增】更新行人轨迹 (所有图都更新)
    % ped_pts_radar(:,1) 是X, (:,2) 是Y
    set(h_ped_geom, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    set(h_ped_overlay1, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    set(h_ped_overlay2, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    
    % 更新标题
    set(title_str1, 'String', {['1. 几何模型 (Frm ' num2str(iFrm) ')'], ['\color{red}' PHASE_NAME]}, 'Interpreter', 'tex');
    
    % 更新 Mask 轮廓
    delete(findobj(ax1, 'Type', 'contour')); 
    contour(ax1, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'm--', 'LineWidth', 1);
    
    delete(findobj(ax3, 'Type', 'contour'));
    contour(ax3, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    
    delete(findobj(ax4, 'Type', 'contour'));
    contour(ax4, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    
    % 更新 Range-FFT
    mag_data = abs(fftRsltRg);
    range_profile = 20 * log10(mean(mag_data, [2, 3, 4]) + 1e-6);
    set(h_rfft_line, 'XData', full_grid.range, 'YData', range_profile);
    set(h_xline_A, 'Value', car_pts.A.rho);
    set(h_xline_K, 'Value', car_pts.K.rho);
    
    % 更新热力图
    set(h_pcolor1, 'CData', pwRA_Full); 
    set(h_pcolor2, 'CData', pwRA_Roi); 
    
    drawnow limitrate; 
end

%% ================= 辅助函数: 通用坐标变换 =================
function pts_radar = transform_world_to_radar(pts_world, v, yaw, t)
    % 输入: pts_world [N x 2] (x, y)
    % 输出: pts_radar [N x 2] (x, y)
    
    % 1. 平移 (雷达向前走 = 物体向后退)
    dy = v * t;
    X_trans = pts_world(:, 1);
    Y_trans = pts_world(:, 2) - dy;
    
    % 2. 旋转 (逆时针旋转 -yaw)
    theta = -yaw;
    X_radar = X_trans * cosd(theta) - Y_trans * sind(theta);
    Y_radar = X_trans * sind(theta) + Y_trans * cosd(theta);
    
    pts_radar = [X_radar, Y_radar];
end