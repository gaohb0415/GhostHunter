% mmWaveMatlab主函数 - 分块处理最终版 (Robust Scalar RELAX Ablation)
% 终极对比实验：First-Chirp vs Median-Shared vs Energy-Weighted
% ---------------------------------------------------------
%% 1. 初始化与配置
close all; clear; clc;
addpath(genpath(pwd));
% =====================【核心参数配置】=====================
PARAM_K_MAX = 6;       % 最大搜索点数
IMPROVE_TH  = 0.01;    % 改善率阈值
BLOCK_SIZE  = 32;      % 将128个chirp切分为32个一组，共4组
% ============================================================
% --- 动力学参数 ---
EGO_VELOCITY = 0.363;
RADAR_YAW    = -30;
TOTAL_FRAMES = 210;
FRAME_PERIOD = 50e-3;
% --- 扫描参数 ---
CFG_LIMIT_ANG = [-90, 90];
CFG_RES_ANG   = 0.5;
% --- 载入配置 ---
config2243;
try load('config.mat', 'resR', 'resV', 'spacingCal');
catch, load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal'); end
if ~exist('spacingCal', 'var'), spacingCal = 1; end
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

%% 2. 预创建绘图对象 (1x3 全景 Capon 布局)
hFig = figure('Name', 'Robust Scalar RELAX Strategies', 'Position', [50, 100, 1800, 500]);

% 子图1: First-Chirp Alpha (Original)
ax1 = subplot(1, 3, 1); hold on; axis equal; grid on;
h_pcolor_raw = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_raw, 'EdgeColor', 'none'); shading interp; colormap(ax1, 'jet');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str1 = title('1. First-Chirp Alpha');

% 子图2: Median-Shared Alpha
ax2 = subplot(1, 3, 2); hold on; axis equal; grid on;
h_pcolor_v1 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_v1, 'EdgeColor', 'none'); shading interp; colormap(ax2, 'jet');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str2 = title('2. Median-Shared Alpha');

% 子图3: Energy-Weighted Alpha
ax3 = subplot(1, 3, 3); hold on; axis equal; grid on;
h_pcolor_v2 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_v2, 'EdgeColor', 'none'); shading interp; colormap(ax3, 'jet');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str3 = title('3. Energy-Weighted Alpha');

%% 3. 核心循环
fprintf('================ STARTING ABLATION STUDY ================\n');
for iFrm = 50 : 1 : 80
    current_time = (iFrm - 1) * FRAME_PERIOD;
    try radarData = readBin(iFrm, 0); catch, break; end
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');
    sorted_data = antArray_Sorted.signal;
    
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
    
    all_corner_rhos = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);
    dist_mask = (full_grid.range > (min(all_corner_rhos) - 0.5)) & (full_grid.range < (max(all_corner_rhos) + 0.5));
    car_range_indices = find(dist_mask);
    
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);
    search_ang_global = (min(all_angles) - 5) : 0.5 : (max(all_angles) + 5);
    
    % 三种策略的干净数据矩阵
    sorted_first  = sorted_data;
    sorted_median = sorted_data;
    sorted_weight = sorted_data;
    
    for r_idx = car_range_indices'
        snapshot_full = squeeze(sorted_data(:, :, r_idx));
        [M, N_Total] = size(snapshot_full);
        
        interf_first  = zeros(M, N_Total);
        interf_median = zeros(M, N_Total);
        interf_weight = zeros(M, N_Total);
        
        num_blocks = ceil(N_Total / BLOCK_SIZE);
        for b = 1:num_blocks
            idx_start = (b-1) * BLOCK_SIZE + 1;
            idx_end   = min(b * BLOCK_SIZE, N_Total);
            curr_indices = idx_start : idx_end;
            snapshot_batch = snapshot_full(:, curr_indices);
            [~, N_Batch] = size(snapshot_batch);
            
            % 1. 执行 First-Chirp
            [~, a_f, ang_f] = run_relax_core(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, spacingCal, 'first');
            if ~isempty(a_f), interf_first(:, curr_indices) = reconstruct_signal(a_f, ang_f, M, N_Batch, spacingCal); end
            
            % 2. 执行 Median-Shared
            [~, a_m, ang_m] = run_relax_core(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, spacingCal, 'median');
            if ~isempty(a_m), interf_median(:, curr_indices) = reconstruct_signal(a_m, ang_m, M, N_Batch, spacingCal); end
            
            % 3. 执行 Energy-Weighted
            [~, a_w, ang_w] = run_relax_core(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, spacingCal, 'weighted');
            if ~isempty(a_w), interf_weight(:, curr_indices) = reconstruct_signal(a_w, ang_w, M, N_Batch, spacingCal); end
        end
        
        sorted_first(:, :, r_idx)  = snapshot_full - interf_first;
        sorted_median(:, :, r_idx) = snapshot_full - interf_median;
        sorted_weight(:, :, r_idx) = snapshot_full - interf_weight;
    end
    
    % 执行 Capon
    pwRA_First  = zeros(nAdc, length(full_grid.angle));
    pwRA_Median = zeros(nAdc, length(full_grid.angle));
    pwRA_Weight = zeros(nAdc, length(full_grid.angle));
    
    for iRg = 1 : nAdc
        [pwRA_First(iRg, :), ~]  = dbf(full_grid.angle', [], sorted_first(:, :, iRg), antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
        [pwRA_Median(iRg, :), ~] = dbf(full_grid.angle', [], sorted_median(:, :, iRg), antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
        [pwRA_Weight(iRg, :), ~] = dbf(full_grid.angle', [], sorted_weight(:, :, iRg), antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
    end
    
    % SIR 统计
    roi_max_f = max(pwRA_First(ShadowMask == 1));  bg_mean_f = mean(pwRA_First(ShadowMask == 0));
    roi_max_m = max(pwRA_Median(ShadowMask == 1)); bg_mean_m = mean(pwRA_Median(ShadowMask == 0));
    roi_max_w = max(pwRA_Weight(ShadowMask == 1)); bg_mean_w = mean(pwRA_Weight(ShadowMask == 0));
    
    fprintf('\n[FRAME %d NUMERICAL PROOF]\n', iFrm);
    fprintf('  First-Chirp -> ROI Max: %.2e, SIR: %.2f dB\n', roi_max_f, 10*log10(roi_max_f/bg_mean_f));
    fprintf('  Median      -> ROI Max: %.2e, SIR: %.2f dB\n', roi_max_m, 10*log10(roi_max_m/bg_mean_m));
    fprintf('  Weighted    -> ROI Max: %.2e, SIR: %.2f dB\n', roi_max_w, 10*log10(roi_max_w/bg_mean_w));
    
    % 渲染
    if ~ishandle(hFig), break; end
    
    set(h_pcolor_raw, 'CData', pwRA_First);  if roi_max_f > 0, caxis(ax1, [roi_max_f * 10^(-20/10), roi_max_f]); end
    set(h_pcolor_v1, 'CData', pwRA_Median);  if roi_max_m > 0, caxis(ax2, [roi_max_m * 10^(-20/10), roi_max_m]); end
    set(h_pcolor_v2, 'CData', pwRA_Weight);  if roi_max_w > 0, caxis(ax3, [roi_max_w * 10^(-20/10), roi_max_w]); end
    
    axes_list = [ax1, ax2, ax3];
    for ax = axes_list
        delete(findobj(ax, 'Type', 'contour'));
        contour(ax, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1);
    end
    
    title_suffix = [' (Frame ' num2str(iFrm) ')' ];
    set(title_str1, 'String', ['1. First-Chirp Alpha' title_suffix]);
    set(title_str2, 'String', ['2. Median-Shared Alpha' title_suffix]);
    set(title_str3, 'String', ['3. Energy-Weighted Alpha' title_suffix]);
    drawnow;
end
fprintf('================ DONE ================\n');

%% ------------------------- 通用 RELAX 核心函数 -------------------------
function [residual, rel_alphas, rel_angles] = run_relax_core(input_signal, K_MAX, improve_th, search_ang, M, spacingCal, mode)
    min_a = min(search_ang); max_a = max(search_ang);
    high_res_search_ang = min_a : 0.1 : max_a;
    residual = input_signal;
    total_raw_energy = sum(abs(input_signal(:)).^2);
    prev_energy = total_raw_energy;
    
    A_scan = exp(1j * pi * (0:M-1)' * spacingCal * sind(high_res_search_ang));
    rel_angles_buf = zeros(1, K_MAX);
    rel_alphas_buf = zeros(1, K_MAX);
    actual_k = 0;
    
    % --- Stage 1: 粗搜 ---
    for k = 1:K_MAX
        spec = sum(abs(A_scan' * residual).^2, 2);
        [~, idx] = max(spec);
        curr_ang = high_res_search_ang(idx);
        a_k = exp(1j * pi * (0:M-1)' * spacingCal * sind(curr_ang));
        
        % 计算投影
        proj_all = (a_k' * residual) / (a_k' * a_k); % 1 x N
        
        % 根据不同模式计算共享 Alpha
        if strcmp(mode, 'first')
            curr_alpha = proj_all(1);
        elseif strcmp(mode, 'median')
            curr_alpha = median(real(proj_all)) + 1j * median(imag(proj_all));
        elseif strcmp(mode, 'weighted')
            w = sum(abs(residual).^2, 1);
            w = w / (sum(w) + eps);
            curr_alpha = sum(w .* proj_all);
        end
        
        temp_residual = residual - curr_alpha * a_k;
        current_energy = sum(abs(temp_residual(:)).^2);
        improvement = (prev_energy - current_energy) / max(prev_energy, eps);
        
        if k > 1 && improvement < improve_th, break; end
        rel_angles_buf(k) = curr_ang; rel_alphas_buf(k) = curr_alpha;
        residual = temp_residual; prev_energy = current_energy; actual_k = k;
    end
    
    if actual_k == 0; rel_alphas = []; rel_angles = []; residual = input_signal; return; end
    rel_angles = rel_angles_buf(1:actual_k); rel_alphas = rel_alphas_buf(1:actual_k);
    
    % --- Stage 2: 精炼 (Refinement) ---
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
            
            spec = sum(abs(A_scan' * data_k).^2, 2);
            [~, idx] = max(spec);
            rel_angles(k) = high_res_search_ang(idx);
            
            a_new = exp(1j * pi * (0:M-1)' * spacingCal * sind(rel_angles(k)));
            proj_all = (a_new' * data_k) / (a_new' * a_new);
            
            if strcmp(mode, 'first')
                rel_alphas(k) = proj_all(1);
            elseif strcmp(mode, 'median')
                rel_alphas(k) = median(real(proj_all)) + 1j * median(imag(proj_all));
            elseif strcmp(mode, 'weighted')
                w = sum(abs(data_k).^2, 1);
                w = w / (sum(w) + eps);
                rel_alphas(k) = sum(w .* proj_all);
            end
        end
    end
    
    residual = input_signal;
    for k = 1:actual_k
        a_final = exp(1j * pi * (0:M-1)' * spacingCal * sind(rel_angles(k)));
        residual = residual - rel_alphas(k) * a_final;
    end
end

function total_sig = reconstruct_signal(alphas, angles, M, N_Snaps, spacingCal)
    total_sig = zeros(M, N_Snaps);
    if isempty(alphas), return; end
    for k = 1:length(alphas)
        a_vec = exp(1j * pi * (0:M-1)' * spacingCal * sind(angles(k)));
        total_sig = total_sig + alphas(k) * a_vec;
    end
end

% ====== 保持不变的动力学转换函数 ======
function pts_radar = transform_world_to_radar(pts_world, v, yaw, t)
    dy = v * t; X_trans = pts_world(:, 1); Y_trans = pts_world(:, 2) - dy; theta = -yaw;
    X_radar = X_trans * cosd(theta) - Y_trans * sind(theta); Y_radar = X_trans * sind(theta) + Y_trans * cosd(theta);
    pts_radar = [X_radar, Y_radar];
end