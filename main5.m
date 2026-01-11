% mmWaveMatlab主函数 - 导师建议升级版 (GhostBuster)
% 功能: 自车运动补偿 + 自适应模型阶数(Adaptive K) + 物理层干扰对消
% ---------------------------------------------------------
%% 1. 初始化与配置
close all; clear; clc; 
addpath(genpath(pwd));

% =====================【播放控制 (Play Control)】=====================
% 1. 单帧调试: 设置 START = END (例如 133, 133)
% 2. 连续播放: 设置 START < END, STEP = 1
% 3. 快速预览: 设置 START < END, STEP = 2 (跳帧)
FRM_START = 1; 
FRM_STEP  = 2;    % 步进控制
FRM_END   = 60;  
% =====================================================================

% =====================【功能开关】=====================
ENABLE_FULL_ANGLE_VIEW = 0; % 0=仅显示ROI, 1=全角度
ENABLE_EGO_COMPENSATE  = 1; % [新] 开启自车运动相位补偿
ENABLE_ADAPTIVE_K      = 1; % [新] 开启自适应K值选择
% ======================================================

% --- 动力学参数 ---
EGO_VELOCITY = 0.363;        % 自车速度 (m/s)
RADAR_YAW    = -30;          % 雷达安装偏角 (度)
FRAME_PERIOD = 50e-3;

% --- 扫描参数 ---
CFG_LIMIT_ANG = [-90, 90]; 
CFG_RES_ANG   = 0.5; 
CFG_LIMIT_R   = [];

% --- 载入配置 (获取波长lambda和时序参数) ---
config2243; 
try load('config.mat', 'resR', 'resV', 'lambda', 'tChirpIntvl'); 
catch, load('.\config\config\config.mat', 'resR', 'resV', 'lambda', 'tChirpIntvl'); end

% --- 网格构建 ---
% 将雷达采集到的极坐标系的坐标转换为直角坐标的映射网格
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
% 极坐标转直角坐标公式
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);




% =====================【实验真值数据 (模拟LiDAR先验)】=====================
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
ped_path_x = [3.44, 3.44, 1.11];
ped_path_y = [5.8, 8.62, 8.62];
ped_init_mat = [ped_path_x', ped_path_y'];




% ===============================================================
%% 2. 预创建绘图对象
% 作用： 预先占位，循环开始之前直接把整个空图画好，然后每次循环只负责画上数据即可，不需要将空图也连同重画
hFig = figure('Name', 'GhostBuster: Adaptive Cancellation', 'Position', [50, 50, 1400, 900]);
ax1 = subplot(2, 2, 1); hold on; axis equal; grid on;
plot(0, 0, 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
h_car_fill = fill(nan, nan, [0.8 0.8 0.8], 'FaceAlpha', 0.5);
h_car_edge = plot(nan, nan, 'k-', 'LineWidth', 1.5);
h_ped_geom = plot(nan, nan, 'g--', 'LineWidth', 2); 
title_str1 = title('1. 几何模型 & 自适应K');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');

ax2 = subplot(2, 2, 2); hold on; grid on;
h_rfft_line = plot(nan, nan, 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
xlim([0, 10]); ylim([40, 120]); xlabel('距离 (m)'); ylabel('幅度 (dB)');
title('2. Range-FFT');

ax3 = subplot(2, 2, 3); hold on; axis equal; grid on;
h_pcolor1 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor1, 'EdgeColor', 'none'); shading interp; colormap(ax3, 'jet');
h_ped_overlay1 = plot(nan, nan, 'w:', 'LineWidth', 2.5); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str3 = title('3. 原始数据');

ax4 = subplot(2, 2, 4); hold on; axis equal; grid on;
h_pcolor2 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor2, 'EdgeColor', 'none'); shading interp; colormap(ax4, 'jet');
h_car_overlay = plot(nan, nan, 'm-', 'LineWidth', 2);
h_ped_overlay2 = plot(nan, nan, 'w:', 'LineWidth', 2.5); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str4 = title('4. GhostBuster 对消结果');

%% 3. 主循环
% [修改] 使用配置好的 Start : Step : End 进行循环
for iFrm = FRM_START : FRM_STEP : FRM_END
    
    current_time = (iFrm - 1) * FRAME_PERIOD;
    
    % --- 3.1 数据读取 ---
    try radarData = readBin(iFrm, 0); catch, break; end
    [nAdc, nChirp, nRx, nTx, ~] = size(radarData);

    % =====================================================================
    % 【Module 1: 自车运动补偿 (Ego-Motion Compensation)】
    % 导师建议: 消除移动平台导致的导向矢量失配
    % =====================================================================
    if ENABLE_EGO_COMPENSATE
        % TDM-MIMO 模式下, Tx1, Tx2, Tx3... 依次发射
        % 每一根 Tx 发射时, 车都往前走了一点点
        % 相位校正因子: exp( j * 4pi/lambda * V_ego * t_delay )
        
        comp_phasor = zeros(1, 1, 1, nTx);   % 维度对齐


        for iTx = 1:nTx
            % 计算第 iTx 根天线发射时相对于第1根天线的时间延迟
            t_delay = (iTx - 1) * (tChirpIntvl / nTx); 
            % 计算因运动产生的双程距离变化 (接近正向为缩短距离, 故加正相位补偿)
            d_move = EGO_VELOCITY * t_delay;
            phase_shift = 4 * pi * d_move / lambda; 
            comp_phasor(1, 1, 1, iTx) = exp(1j * phase_shift); 
        end
        % 应用补偿到原始数据
        radarData = radarData .* comp_phasor;
    end
    
    % --- 3.2 Range FFT ---
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0); 
    
    % --- 3.3 几何计算 (LiDAR 先验) ---
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ped_pts_radar = transform_world_to_radar(ped_init_mat, EGO_VELOCITY, RADAR_YAW, current_time);
    
    % 生成 Mask (用于 Capon 加速和 RELAX 保护)
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
    
    % =====================================================================
    % 【Module 2: LiDAR 引导的自适应 K (Adaptive Model Order)】
    % =====================================================================
    if ENABLE_ADAPTIVE_K
        % 1. 计算车的中心距离
        car_center_rho = (car_pts.A.rho + car_pts.K.rho) / 2;
        
        % 2. 假设车宽
        CAR_WIDTH_GUESS = 1.8; 
        
        % 3. 计算张角
        angle_span_rad = 2 * atan( (CAR_WIDTH_GUESS / 2) / car_center_rho );
        angle_span_deg = rad2deg(angle_span_rad);
        
        % 4. [关键修正] 使用物理分辨率计算 K
        % TI 4片级联雷达的物理角分辨率约为 1.2 ~ 1.5 度
        RADAR_PHYS_RES = 1.4; 
        BETA = 1.5; % 稍微温和一点的过采样
        
        K_Adaptive = ceil( (angle_span_deg / RADAR_PHYS_RES) * BETA );
        
        % 5. [关键修正] 放宽上限
        % 现在 10m 处大约是 11，5m 处大约是 20
        % 建议上限设为 25 或 30，给近处留出空间
        MAX_K_LIMIT = 25; 
        K_Comps = max(min(K_Adaptive, MAX_K_LIMIT), 1);
        
        k_disp_str = sprintf('Adaptive K = %d (Dist: %.1fm)', K_Comps, car_center_rho);
    else
        K_Comps = 3; 
        k_disp_str = 'Fixed K = 3';
    end

    % =====================================================================
    % 【Module 3: SC-RELAX 干扰对消 (Spatially-Constrained)】
    % 导师建议: 不仅处理车身 Range，还要处理 Ghost Range (旁瓣泄露区)
    % =====================================================================
    
    fftRsltRg_Clean = fftRsltRg; 
    
    % 确定车的范围 (物理真值)
    car_range_min = min([car_pts.A.rho, car_pts.B.rho, car_pts.C.rho, car_pts.K.rho]);
    car_range_max = max([car_pts.A.rho, car_pts.B.rho, car_pts.C.rho, car_pts.K.rho]);
    
    % [升级]: 处理范围扩大到车后 4 米 (覆盖鬼探头区域)
    clean_mask = (full_grid.range >= car_range_min - 1.0) & ...
                 (full_grid.range <= car_range_max + 4.0); 
    range_indices_to_clean = find(clean_mask);
    
    % 确定车的角度范围
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);
    min_car_ang = min(all_angles) - 3; 
    max_car_ang = max(all_angles) + 3;
    
    % >>> 开始逐距离门清洗 <<<
    for r_idx = range_indices_to_clean'
        
        raw_slice = squeeze(fftRsltRg(r_idx, :, :, :));
        if ndims(raw_slice) == 3 
             [nCh, nRx_s, nTx_s] = size(raw_slice);
             raw_slice = reshape(raw_slice, nCh, nRx_s*nTx_s);
        end
        snapshot = raw_slice.'; 
        [M, N_Snaps] = size(snapshot);
        
        % RELAX 变量
        rel_angles = zeros(1, K_Comps);
        rel_alphas = zeros(1, K_Comps); 
        residual = snapshot; 
        
        % 空间约束: 仅在车所在的角域内搜索干扰
        search_ang = min_car_ang : 0.5 : max_car_ang; 
        A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang)); 
        
        % --- 步骤 A: 初始化 (CLEAN) ---
        for k = 1:K_Comps
            spec = sum(abs(A_scan' * residual), 2);
            [~, idx] = max(spec);
            rel_angles(k) = search_ang(idx); 
            a_k = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            rel_alphas(k) = (a_k' * residual(:,1)) / (a_k' * a_k); 
            residual = residual - rel_alphas(k) * a_k;
        end
        
        % --- 步骤 B: 迭代优化 (RELAX) ---
        MAX_RELAX_ITER = 5; 
        for iter = 1 : MAX_RELAX_ITER
            for k = 1 : K_Comps
                % Add Back
                data_k = snapshot; 
                for other = 1 : K_Comps
                    if other ~= k
                        a_other = exp(-1j * pi * (0:M-1)' * sind(rel_angles(other)));
                        data_k = data_k - rel_alphas(other) * a_other;
                    end
                end
                % Re-estimate
                spec = sum(abs(A_scan' * data_k), 2);
                [~, idx] = max(spec);
                rel_angles(k) = search_ang(idx); 
                a_new = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
                rel_alphas(k) = (a_new' * data_k(:,1)) / (a_new' * a_new);
            end
        end
        
        % --- 步骤 C: 最终对消 (Subtraction) ---
        total_interference = zeros(size(snapshot));
        for k = 1 : K_Comps
            a_final = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            total_interference = total_interference + rel_alphas(k) * a_final;
        end
        
        % 得到残差
        snapshot_clean = snapshot - total_interference;
        
        try fftRsltRg_Clean(r_idx, :, :) = snapshot_clean.'; catch, end
    end 
    
    % =====================================================================
    % 【Module 4: 残差域成像 (Residual Imaging)】
    % =====================================================================
    
    if ENABLE_FULL_ANGLE_VIEW
        procMask = ones(size(ShadowMask)); 
        view_str = ' (Full)';
    else
        procMask = ShadowMask;
        view_str = ' (ROI)';
    end
    
    % 图3: 原始数据 Capon
    [pwRA_Baseline, ~] = dbfProc1D(fftRsltRg, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', procMask, 'pcEn', 0, 'drawEn', 0);
        
    % 图4: 净化数据 Capon
    [pwRA_Roi, ~] = dbfProc1D(fftRsltRg_Clean, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', procMask, 'pcEn', 0, 'drawEn', 0);
        
    % --- 绘图刷新 ---
    if ~ishandle(hFig), break; end 
    
    % 更新几何
    car_x = [car_pts.all_x; car_pts.all_x(1)]; car_y = [car_pts.all_y; car_pts.all_y(1)];
    set(h_car_fill, 'XData', car_x, 'YData', car_y);
    set(h_car_edge, 'XData', car_x, 'YData', car_y);
    set(h_car_overlay, 'XData', car_x, 'YData', car_y);
    set(h_ped_geom, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    set(h_ped_overlay1, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    set(h_ped_overlay2, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    
    % 更新标题
    set(title_str1, 'String', {['Frame ' num2str(iFrm)], ['\color{red}' k_disp_str]});
    set(title_str4, 'String', ['4. 自适应对消' view_str]);
    
    % 更新 Range-FFT
    mag_data = abs(fftRsltRg);
    range_profile = 20 * log10(mean(mag_data, [2, 3, 4]) + 1e-6);
    set(h_rfft_line, 'XData', full_grid.range, 'YData', range_profile);
    
    % 更新热力图
    set(h_pcolor1, 'CData', pwRA_Baseline); 
    set(h_pcolor2, 'CData', pwRA_Roi); 
    
    drawnow limitrate; 
end

%% ================= 辅助函数 =================
function pts_radar = transform_world_to_radar(pts_world, v, yaw, t)
    dy = v * t;
    X_trans = pts_world(:, 1);
    Y_trans = pts_world(:, 2) - dy;
    theta = -yaw;
    X_radar = X_trans * cosd(theta) - Y_trans * sind(theta);
    Y_radar = X_trans * sind(theta) + Y_trans * cosd(theta);
    pts_radar = [X_radar, Y_radar];
end