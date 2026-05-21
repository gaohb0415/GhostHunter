% 动态对比主函数 - K>=1 与 K>=0 双路并行处理
% 功能: 逐帧对比两种算法在成像质量与算力开销(K值分布)上的差异
% ---------------------------------------------------------
%% 1. 初始化与配置
close all; clear; clc; 
addpath(genpath(pwd));

% =====================【核心参数配置】=====================
PARAM_K_MAX = 6;       
IMPROVE_TH  = 0.01;    
BLOCK_SIZE  = 32;      
NOISE_FLOOR_TH = 1e9;  % 【关键参数】底噪阈值，请根据实际打印调整！

EGO_VELOCITY = 0.363;        
RADAR_YAW    = -30;          
TOTAL_FRAMES = 210; 
FRAME_PERIOD = 50e-3;

CFG_LIMIT_ANG = [-90, 90];      
CFG_RES_ANG   = 0.5;            
CFG_LIMIT_R   = [];

% --- 录像开关 (强烈建议开启以保存动态对比结果) ---
RECORD_VIDEO = 0; % 设为1则录制mp4
if RECORD_VIDEO
    vWriter = VideoWriter('K_Strategy_Comparison.mp4', 'MPEG-4');
    vWriter.FrameRate = 10;
    open(vWriter);
end

% --- 载入配置与网格构建 ---
config2243; 
try load('config.mat', 'resR', 'resV', 'spacingCal'); 
catch, load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal'); end
if ~exist('spacingCal', 'var'); spacingCal = 1; end

nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)';                                   
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);        
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);          
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);

% --- 实验真值数据 ---
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];

%% 2. 预创建 2x2 绘图对象
hFig = figure('Name', 'Dynamic Comparison: K>=1 vs K>=0', 'Position', [50, 50, 1600, 900]);

% --- 左上 (1): K>=1 成像 ---
ax1 = subplot(2, 2, 1); hold on; axis equal; grid on;
h_pcolor_B = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_B, 'EdgeColor', 'none'); shading interp; colormap(ax1, 'jet');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str1 = title('TL: Strategy B (Adaptive K>=1) Image');

% --- 左下 (3): K>=0 成像 ---
ax3 = subplot(2, 2, 3); hold on; axis equal; grid on;
h_pcolor_C = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_C, 'EdgeColor', 'none'); shading interp; colormap(ax3, 'jet');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str3 = title('BL: Strategy C (Adaptive K>=0) Image');

% --- 右上 (2): K>=1 的 K值分布 ---
ax2 = subplot(2, 2, 2); hold on; grid on;
h_bar_B = bar(full_grid.range, zeros(size(full_grid.range)), 'FaceColor', [0.3 0.5 0.7]);
xlim([0, 15]); ylim([0, PARAM_K_MAX + 1]); 
yline(PARAM_K_MAX, 'r--', 'Limit');
xlabel('Range (m)'); ylabel('Adaptive K Value');
title_str2 = title('TR: K Profile (Strategy B)');

% --- 右下 (4): K>=0 的 K值分布 ---
ax4 = subplot(2, 2, 4); hold on; grid on;
h_bar_C = bar(full_grid.range, zeros(size(full_grid.range)), 'FaceColor', [0.2 0.7 0.3]);
xlim([0, 15]); ylim([0, PARAM_K_MAX + 1]); 
yline(PARAM_K_MAX, 'r--', 'Limit');
xlabel('Range (m)'); ylabel('Adaptive K Value');
title_str4 = title('BR: K Profile (Strategy C)');

%% 3. 核心大循环
fprintf('================ STARTING DYNAMIC COMPARISON ================\n');

for iFrm = 107 : 1 : 107
    current_time = (iFrm - 1) * FRAME_PERIOD;
    
    % --- 独立状态复位 ---
    k_counts_B = zeros(size(full_grid.range)); 
    k_counts_C = zeros(size(full_grid.range)); 
    
    % --- 数据与几何准备 ---
    try radarData = readBin(iFrm, 0); catch, break; end
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');     
    sorted_data = antArray_Sorted.signal; 
    
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);    
    
    all_corner_rhos = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);    
    car_rho_min = min(all_corner_rhos);
    car_rho_max = max(all_corner_rhos);
    dist_mask = (full_grid.range > (car_rho_min - 0.5)) & (full_grid.range < (car_rho_max + 0.5));
    car_range_indices = find(dist_mask);
    
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);          
    search_ang_global = (min(all_angles) - 5) : 0.5 : (max(all_angles) + 5); 
    
    % === 复制两条独立的数据流 ===
    sorted_data_clean_B = sorted_data; 
    sorted_data_clean_C = sorted_data; 
    
    % --- 算法计算核心 ---
    for r_idx = car_range_indices'
        current_R = full_grid.range(r_idx);
        snapshot_full = squeeze(sorted_data(:, :, r_idx)); 
        [M, N_Total] = size(snapshot_full);
        
        interference_full_B = zeros(M, N_Total);
        interference_full_C = zeros(M, N_Total);
        max_k_in_blocks_B = 0; 
        max_k_in_blocks_C = 0;
        
        num_blocks = ceil(N_Total / BLOCK_SIZE);
        for b = 1:num_blocks
            idx_start = (b-1) * BLOCK_SIZE + 1;
            idx_end   = min(b * BLOCK_SIZE, N_Total);
            current_indices = idx_start : idx_end;
            snapshot_batch = snapshot_full(:, current_indices);
            [~, N_Batch] = size(snapshot_batch);
            
            % [双线并行]：
            % 1. 策略 B (强制 K>=1, noise_th = 0)
            [~, alphas_B, angles_B] = run_relax_core_batch(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, current_R, spacingCal, 0);
            max_k_in_blocks_B = max(max_k_in_blocks_B, length(alphas_B));
            if ~isempty(alphas_B)
                interference_full_B(:, current_indices) = reconstruct_signal_batch(alphas_B, angles_B, M, N_Batch, spacingCal);
            end
            
            % 2. 策略 C (允许 K>=0, noise_th = NOISE_FLOOR_TH)
            [~, alphas_C, angles_C] = run_relax_core_batch(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, current_R, spacingCal, NOISE_FLOOR_TH);
            max_k_in_blocks_C = max(max_k_in_blocks_C, length(alphas_C));
            if ~isempty(alphas_C)
                interference_full_C(:, current_indices) = reconstruct_signal_batch(alphas_C, angles_C, M, N_Batch, spacingCal);
            end
        end
        
        k_counts_B(r_idx) = max_k_in_blocks_B;
        k_counts_C(r_idx) = max_k_in_blocks_C;
        sorted_data_clean_B(:, :, r_idx) = snapshot_full - interference_full_B;
        sorted_data_clean_C(:, :, r_idx) = snapshot_full - interference_full_C;
    end
    
    % --- DBF 成像 ---
    pwRA_B_Matrix = zeros(nAdc, length(full_grid.angle));
    pwRA_C_Matrix = zeros(nAdc, length(full_grid.angle));
    for iRg = 1 : nAdc
        sig_current_B = sorted_data_clean_B(:, :, iRg);
        sig_current_C = sorted_data_clean_C(:, :, iRg);
        valid_idx = find(ShadowMask(iRg, :) == 1);
        if ~isempty(valid_idx)
             ang_subset = full_grid.angle(valid_idx)';
             [pw_subset_B, ~] = dbf(ang_subset, [], sig_current_B, antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
             [pw_subset_C, ~] = dbf(ang_subset, [], sig_current_C, antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
             pwRA_B_Matrix(iRg, valid_idx) = pw_subset_B;
             pwRA_C_Matrix(iRg, valid_idx) = pw_subset_C;
        end
    end
    
    % --- 图形更新 ---
    if ~ishandle(hFig), break; end
    
    % 清理并重新绘制遮挡框 (为了图面干净)
    for ax = [ax1, ax3]
        delete(findobj(ax, 'Type', 'contour'));
        contour(ax, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1);
    end
    
    % 更新热力图数据
    set(h_pcolor_B, 'CData', pwRA_B_Matrix);
    set(h_pcolor_C, 'CData', pwRA_C_Matrix);
    
    % 更新柱状图数据
    set(h_bar_B, 'YData', k_counts_B);
    set(h_bar_C, 'YData', k_counts_C);
    
    % 标题带上当前帧号
    title_suffix = [' (Frame ' num2str(iFrm) ')'];
    set(title_str1, 'String', ['TL: Strategy B (K>=1)' title_suffix]);
    set(title_str3, 'String', ['BL: Strategy C (K>=0)' title_suffix]);
    drawnow;
    
    if RECORD_VIDEO
        frame = getframe(hFig);
        writeVideo(vWriter, frame);
    end
end

if RECORD_VIDEO; close(vWriter); fprintf('视频录制已完成。\n'); end
fprintf('================ BATCH FIX COMPLETE ================\n');

%% ================= 信号处理附属函数 =================

% 【核心修改】引入了 noise_th 作为一个控制参数
function [residual, rel_alphas, rel_angles] = run_relax_core_batch(input_signal, K_MAX, improve_th, search_ang, M, range_val, spacingCal, noise_th)
    
    total_raw_energy = sum(abs(input_signal(:)).^2);
    
    % 1. 底噪拦截机制
    if total_raw_energy < noise_th
        residual = input_signal; rel_alphas = []; rel_angles = []; return; 
    end
    
    min_a = min(search_ang); max_a = max(search_ang);
    high_res_search_ang = min_a : 0.1 : max_a; 
    
    residual = input_signal;
    rel_angles_buf = zeros(1, K_MAX); rel_alphas_buf = zeros(1, K_MAX);
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