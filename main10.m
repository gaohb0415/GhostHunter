% mmWaveMatlab主函数 - 分块处理最终版 (Batch Processing RELAX)
% 终极对比实验：全域 Capon (Raw) vs 全域 Capon (RELAX Clean)
% ---------------------------------------------------------
%% 1. 初始化与配置
close all; clear; clc; 
addpath(genpath(pwd));
% =====================【核心参数配置】=====================
PARAM_K_MAX = 6;       % 最大搜索点数
IMPROVE_TH  = 0.01;    % 改善率阈值 (恢复到正常值 1%)
BLOCK_SIZE  = 32;      % 【核心】：将128个chirp切分为32个一组，共4组
% ============================================================
% --- 动力学参数 ---
EGO_VELOCITY = 0.363;        
RADAR_YAW    = -30;          
TOTAL_FRAMES = 210; 
FRAME_PERIOD = 50e-3;
% --- 扫描参数 ---
CFG_LIMIT_ANG = [-90, 90];      % 雷达扫描的边界角度
CFG_RES_ANG   = 0.5;            % 扫描分辨率和步长
CFG_LIMIT_R   = [];
% --- 载入配置 ---
config2243; 
try load('config.mat', 'resR', 'resV', 'spacingCal'); 
catch, load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal'); end
if ~exist('spacingCal', 'var'); spacingCal = 1; end
% --- 网格构建 --- 
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)';                                   
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);        
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);          
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);
% =====================【实验真值数据】=====================
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
ped_path_x = [0, 0, 0];
ped_path_y = [0, 0, 0];
ped_init_mat = [ped_path_x', ped_path_y'];

%% 2. 预创建绘图对象 (1x3 布局)
hFig = figure('Name', 'Ablation Study: Full-Field Capon Comparison', 'Position', [50, 100, 1600, 500]);
% 子图1: Raw Data + 全域Capon
ax1 = subplot(1, 3, 1); hold on; axis equal; grid on;
h_pcolor_raw = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_raw, 'EdgeColor', 'none'); shading interp; colormap(ax1, 'jet');
h_ped_ov_raw = plot(nan, nan, 'w:', 'LineWidth', 2); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str1 = title('1. Raw Data + Full Capon');

% 子图2: RELAX Clean Data + 全域Capon
ax2 = subplot(1, 3, 2); hold on; axis equal; grid on;
h_pcolor_clean = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_clean, 'EdgeColor', 'none'); shading interp; colormap(ax2, 'jet');
h_ped_ov_clean = plot(nan, nan, 'w:', 'LineWidth', 2); 
h_kpts_global = plot(nan, nan, 'rs', 'MarkerSize', 6, 'LineWidth', 1.5, 'Visible', 'off'); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str2 = title('2. RELAX + Full Capon (Proposed)');

% 子图3: K值监控
ax3 = subplot(1, 3, 3); hold on; grid on;
h_bar_k = bar(full_grid.range, zeros(size(full_grid.range)), 'FaceColor', [0.2 0.6 0.8]);
xlim([0, 15]); ylim([0, PARAM_K_MAX + 1]); 
xlabel('Range (m)'); ylabel('Adaptive K');
title_str3 = title('3. K Profile');
yline(PARAM_K_MAX, 'r--', 'Limit');

%% 3. 核心循环
fprintf('================ STARTING ABLATION STUDY ================\n');
for iFrm = 58 : 1 : 58 
    current_time = (iFrm - 1) * FRAME_PERIOD;
    
    k_coll.R = []; k_coll.Ang = [];
    k_counts_per_bin = zeros(size(full_grid.range)); 
    
    try radarData = readBin(iFrm, 0); catch, break; end
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    
    antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');    
    sorted_data = antArray_Sorted.signal; 
    
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ped_pts_radar = transform_world_to_radar(ped_init_mat, EGO_VELOCITY, RADAR_YAW, current_time);
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);    
    
    all_corner_rhos = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);    
    car_rho_min = min(all_corner_rhos);
    car_rho_max = max(all_corner_rhos);
    dist_mask = (full_grid.range > (car_rho_min - 0.5)) & (full_grid.range < (car_rho_max + 0.5));
    car_range_indices = find(dist_mask);
    
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);          
    search_ang_global = (min(all_angles) - 5) : 0.5 : (max(all_angles) + 5); 
    
    sorted_data_clean = sorted_data; 
    
    % RELAX 处理过程 (消除干扰)
    for r_idx = car_range_indices'
        current_R = full_grid.range(r_idx);
        snapshot_full = squeeze(sorted_data(:, :, r_idx)); 
        [M, N_Total] = size(snapshot_full);
        interference_full = zeros(M, N_Total);
        
        num_blocks = ceil(N_Total / BLOCK_SIZE);
        max_k_in_blocks = 0; 
        for b = 1:num_blocks
            idx_start = (b-1) * BLOCK_SIZE + 1;
            idx_end   = min(b * BLOCK_SIZE, N_Total);
            current_indices = idx_start : idx_end;
            snapshot_batch = snapshot_full(:, current_indices);
            [~, N_Batch] = size(snapshot_batch);
            
            [~, alphas, angles] = run_relax_core_batch(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, ...
                search_ang_global, M, false, current_R, spacingCal);
            
            if length(alphas) > max_k_in_blocks
                max_k_in_blocks = length(alphas);
            end
            
            if ~isempty(alphas)
                interf_batch = reconstruct_signal_batch(alphas, angles, M, N_Batch, spacingCal);
                interference_full(:, current_indices) = interf_batch;
            end
        end
        k_counts_per_bin(r_idx) = max_k_in_blocks;
        % 执行减法获取 clean data
        sorted_data_clean(:, :, r_idx) = snapshot_full - interference_full;
    end
    
    % =========================================================================
    % 【控制变量计算】：对 Raw 和 Clean 数据执行完全相同的高分辨全域 Capon 计算
    % =========================================================================
    pwRA_Raw_Capon = zeros(nAdc, length(full_grid.angle));
    pwRA_Clean_Capon = zeros(nAdc, length(full_grid.angle));
    
    for iRg = 1 : nAdc
        % 1. Baseline: 原始数据算全图 Capon
        sig_raw = sorted_data(:, :, iRg);
        [pw_row_raw, ~] = dbf(full_grid.angle', [], sig_raw, antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
        pwRA_Raw_Capon(iRg, :) = pw_row_raw;
        
        % 2. Ours: 净化数据算全图 Capon
        sig_clean = sorted_data_clean(:, :, iRg);
        [pw_row_clean, ~] = dbf(full_grid.angle', [], sig_clean, antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
        pwRA_Clean_Capon(iRg, :) = pw_row_clean;
    end
    
    % =========================================================================
    % 【数值证明】：提取ROI内外的信干比 (SIR)，打印到控制台，绝不欺骗肉眼
    % =========================================================================
    % 找到 ROI 内部行人的最高能量
    roi_max_raw = max(pwRA_Raw_Capon(ShadowMask == 1));
    roi_max_clean = max(pwRA_Clean_Capon(ShadowMask == 1));
    
    % 找到 ROI 外部（旁瓣干扰）的平均能量
    bg_mean_raw = mean(pwRA_Raw_Capon(ShadowMask == 0));
    bg_mean_clean = mean(pwRA_Clean_Capon(ShadowMask == 0));
    
    fprintf('\n[FRAME %d NUMERICAL PROOF]\n', iFrm);
    fprintf('  Raw Data Capon   -> ROI Max: %.2e, Background Mean: %.2e, SIR: %.2f dB\n', roi_max_raw, bg_mean_raw, 10*log10(roi_max_raw/bg_mean_raw));
    fprintf('  RELAX Data Capon -> ROI Max: %.2e, Background Mean: %.2e, SIR: %.2f dB\n', roi_max_clean, bg_mean_clean, 10*log10(roi_max_clean/bg_mean_clean));
    
    %% --- 可视化渲染 (防欺骗锁定范围) ---
    if ~ishandle(hFig), break; end
    
    % 为了防止最大的车把全图压黑，我们将两幅图的显示上限，都强制设置为它们各自ROI内部的行人峰值！
    % 这样行人在两张图里一样亮。我们只需要观察哪张图的背景更干净。
    set(h_pcolor_raw, 'CData', pwRA_Raw_Capon);
    if roi_max_raw > 0
        caxis(ax1, [roi_max_raw * 10^(-20/10), roi_max_raw]); % 统一 20dB 动态范围
    end
    
    set(h_pcolor_clean, 'CData', pwRA_Clean_Capon);
    if roi_max_clean > 0
        caxis(ax2, [roi_max_clean * 10^(-20/10), roi_max_clean]); % 统一 20dB 动态范围
    end
    
    axes_list = [ax1, ax2];
    for ax = axes_list
        delete(findobj(ax, 'Type', 'contour'));
        contour(ax, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1);
    end
    
    set(h_bar_k, 'YData', k_counts_per_bin);
    title_suffix = [' (Frame ' num2str(iFrm) ')' ];
    set(title_str1, 'String', ['1. Raw Data + Full Capon' title_suffix]);
    drawnow;
end
fprintf('================ DONE ================\n');

% ------------------------- 附属函数保持不变 -------------------------
function [residual, rel_alphas, rel_angles] = run_relax_core_batch(input_signal, K_MAX, improve_th, search_ang, M, do_print, range_val, spacingCal)
    min_a = min(search_ang); max_a = max(search_ang);
    high_res_search_ang = min_a : 0.1 : max_a; 
    residual = input_signal;
    rel_angles_buf = zeros(1, K_MAX); rel_alphas_buf = zeros(1, K_MAX);
    total_raw_energy = sum(abs(input_signal(:)).^2);
    prev_energy = total_raw_energy;
    actual_k = 0; 
    A_scan = exp(1j * pi * (0:M-1)' * spacingCal * sind(high_res_search_ang)); 
    for k = 1:K_MAX
        spec = sum(abs(A_scan' * residual), 2); 
        [~, idx] = max(spec);       
        curr_ang = high_res_search_ang(idx);
        a_k = exp(1j * pi * (0:M-1)' * spacingCal * sind(curr_ang));
        curr_alpha = (a_k' * residual(:,1)) / (a_k' * a_k);
        temp_residual = residual - curr_alpha * a_k;
        current_energy = sum(abs(temp_residual(:)).^2);
        improvement = (prev_energy - current_energy) / prev_energy;
        if k > 1 && improvement < improve_th, break; end
        rel_angles_buf(k) = curr_ang; rel_alphas_buf(k) = curr_alpha;
        residual = temp_residual; prev_energy = current_energy; actual_k = k;
    end
    if actual_k == 0; rel_alphas = []; rel_angles = []; return; end
    rel_angles = rel_angles_buf(1:actual_k); rel_alphas = rel_alphas_buf(1:actual_k);
    MAX_ITER = 5;
    for iter = 1:MAX_ITER
        for k = 1:actual_k
            data_k = input_signal;          
            for other = 1:actual_k
                if other ~= k
                    a_other = exp(1j * pi * (0:M-1)' * spacingCal * sind(rel_angles(other))); 
                    data_k = data_k - rel_alphas(other) * a_other; 
                end
            end
            spec = sum(abs(A_scan' * data_k), 2); [~, idx] = max(spec); rel_angles(k) = high_res_search_ang(idx);
            a_new = exp(1j * pi * (0:M-1)' * spacingCal * sind(rel_angles(k))); rel_alphas(k) = (a_new' * data_k(:,1)) / (a_new' * a_new);
        end
    end
end

function total_sig = reconstruct_signal_batch(alphas, angles, M, N_Snaps, spacingCal)
    total_sig = zeros(M, N_Snaps);
    for k = 1:length(alphas)
        a_vec = exp(1j * pi * (0:M-1)' * spacingCal * sind(angles(k))); 
        total_sig = total_sig + alphas(k) * a_vec; 
    end
end

function pts_radar = transform_world_to_radar(pts_world, v, yaw, t)
    dy = v * t; X_trans = pts_world(:, 1); Y_trans = pts_world(:, 2) - dy; theta = -yaw;
    X_radar = X_trans * cosd(theta) - Y_trans * sind(theta); Y_radar = X_trans * sind(theta) + Y_trans * cosd(theta);
    pts_radar = [X_radar, Y_radar];
end