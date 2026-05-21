function experiment2_quality_comparison()
% 实验2：成像质量等效对比 (纯净对比版)
% 目的: 证明 K=0 策略在大幅提速的同时，完美保留了真实的干扰抵消效果，没有引入像素级劣化。

    %% 1. 初始化与配置 (与原主函数完全对齐)
    close all; clear; clc; 
    addpath(genpath(pwd));
    
    % 核心参数
    PARAM_K_MAX = 6;       
    IMPROVE_TH  = 0.01;    
    BLOCK_SIZE  = 32;      
    NOISE_FLOOR_TH = 1e9; % 【关键参数】请确保此阈值与实验1一致

    % 动力学参数
    EGO_VELOCITY = 0.363;        
    RADAR_YAW    = -30;          
    FRAME_PERIOD = 50e-3;
    
    % --- 请在这里指定你想对比的特定帧 (建议选一个车辆和行人都比较明显的帧) ---
    TEST_FRAME = 115;     
    % -------------------------------------------------------------------------
    
    CFG_LIMIT_ANG = [-90, 90];      
    CFG_RES_ANG   = 0.5;            
    
    % 载入配置
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
    
    ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
    ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
    car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
    
    %% 2. 提取雷达原始数据与几何掩膜
    fprintf('正在加载 Frame %d 的雷达数据...\n', TEST_FRAME);
    current_time = (TEST_FRAME - 1) * FRAME_PERIOD;
    try radarData = readBin(TEST_FRAME, 0); 
    catch, error('读取第 %d 帧数据失败，请检查数据路径。', TEST_FRAME); end
    
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');     
    sorted_data = antArray_Sorted.signal; 
    
    car_pts = get_car_dynamic_coords(TEST_FRAME, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
    
    all_corner_rhos = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);    
    car_rho_min = min(all_corner_rhos);
    car_rho_max = max(all_corner_rhos);
    dist_mask = (full_grid.range > (car_rho_min - 0.5)) & (full_grid.range < (car_rho_max + 0.5));
    car_range_indices = find(dist_mask);
    
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);          
    search_ang_global = (min(all_angles) - 5) : 0.5 : (max(all_angles) + 5); 
    
    %% 3. 分别执行两种策略
    fprintf('执行策略 B: Adaptive (K>=1)...\n');
    sorted_data_B = process_frame(sorted_data, car_range_indices, full_grid.range, BLOCK_SIZE, PARAM_K_MAX, IMPROVE_TH, search_ang_global, spacingCal, 'B', 0);
    
    fprintf('执行策略 C: Adaptive (K>=0)...\n');
    sorted_data_C = process_frame(sorted_data, car_range_indices, full_grid.range, BLOCK_SIZE, PARAM_K_MAX, IMPROVE_TH, search_ang_global, spacingCal, 'C', NOISE_FLOOR_TH);

    %% 4. 生成 DBF 热力图数据
    fprintf('生成 DBF 热力图图像...\n');
    pwRA_B = generate_dbf_heatmap(sorted_data_B, nAdc, full_grid.angle, antArray_Sorted.arrayPos, spacingCal, ShadowMask);
    pwRA_C = generate_dbf_heatmap(sorted_data_C, nAdc, full_grid.angle, antArray_Sorted.arrayPos, spacingCal, ShadowMask);
    
    % 计算两者的绝对残差 (差异)
    pwRA_Diff = abs(pwRA_B - pwRA_C);
    
    %% 5. 绘图对比
    figure('Name', sprintf('Image Quality Comparison (Frame %d)', TEST_FRAME), 'Position', [50, 150, 1600, 500]);
    
    % 子图 1: 策略 B
    ax1 = subplot(1, 3, 1); hold on; axis equal; grid on;
    h1 = pcolor(X_Plot, Y_Plot, pwRA_B);
    set(h1, 'EdgeColor', 'none'); shading interp; colormap(ax1, 'jet');
    contour(ax1, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
    title('1. Strategy B (Adaptive K>=1)', 'FontSize', 12);
    
    % 子图 2: 策略 C
    ax2 = subplot(1, 3, 2); hold on; axis equal; grid on;
    h2 = pcolor(X_Plot, Y_Plot, pwRA_C);
    set(h2, 'EdgeColor', 'none'); shading interp; colormap(ax2, 'jet');
    contour(ax2, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
    title('2. Strategy C (Adaptive K>=0)', 'FontSize', 12);
    
    % 子图 3: 残差图 (差异分析)
    ax3 = subplot(1, 3, 3); hold on; axis equal; grid on;
    h3 = pcolor(X_Plot, Y_Plot, pwRA_Diff);
    set(h3, 'EdgeColor', 'none'); shading interp; colormap(ax3, 'hot'); % 使用 hot 颜色表凸显差异
    colorbar(ax3); % 显示差异量级
    contour(ax3, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
    title('3. Absolute Difference |B - C|', 'FontSize', 12);
    
    fprintf('对比完成！请查看图像。\n');
end

%% ================= 核心处理封装函数 =================
function sorted_data_clean = process_frame(sorted_data, car_range_indices, full_grid_range, BLOCK_SIZE, PARAM_K_MAX, IMPROVE_TH, search_ang_global, spacingCal, strategy_mode, noise_th)
    sorted_data_clean = sorted_data; 
    for r_idx = car_range_indices'
        current_R = full_grid_range(r_idx);
        snapshot_full = squeeze(sorted_data(:, :, r_idx)); 
        [M, N_Total] = size(snapshot_full);
        interference_full = zeros(M, N_Total);
        num_blocks = ceil(N_Total / BLOCK_SIZE);
        for b = 1:num_blocks
            idx_start = (b-1) * BLOCK_SIZE + 1;
            idx_end   = min(b * BLOCK_SIZE, N_Total);
            current_indices = idx_start : idx_end;
            snapshot_batch = snapshot_full(:, current_indices);
            [~, N_Batch] = size(snapshot_batch);
            
            if strategy_mode == 'B'
                [~, alphas, angles] = run_relax_core_B(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, spacingCal);
            else
                [~, alphas, angles] = run_relax_core_C(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, spacingCal, noise_th);
            end
            
            if ~isempty(alphas)
                interf_batch = reconstruct_signal_batch_local(alphas, angles, M, N_Batch, spacingCal);
                interference_full(:, current_indices) = interf_batch;
            end
        end
        sorted_data_clean(:, :, r_idx) = snapshot_full - interference_full;
    end
end

function pwRA_Matrix = generate_dbf_heatmap(sorted_data_clean, nAdc, full_grid_angle, arrayPos, spacingCal, ShadowMask)
    pwRA_Matrix = zeros(nAdc, length(full_grid_angle));
    for iRg = 1 : nAdc
        sig_current = sorted_data_clean(:, :, iRg);
        valid_idx = find(ShadowMask(iRg, :) == 1);
        if ~isempty(valid_idx)
             ang_subset = full_grid_angle(valid_idx)';
             [pw_subset, ~] = dbf(ang_subset, [], sig_current, arrayPos, [], 'spacingCal', spacingCal);
             pwRA_Matrix(iRg, valid_idx) = pw_subset;
        end
    end
end

% === 策略 B ===
function [residual, rel_alphas, rel_angles] = run_relax_core_B(input_signal, K_MAX, improve_th, search_ang, M, spacingCal)
    high_res_search_ang = min(search_ang) : 0.1 : max(search_ang);
    A_scan = exp(1j * pi * (0:M-1)' * spacingCal * sind(high_res_search_ang)); 
    residual = input_signal;
    prev_energy = sum(abs(input_signal(:)).^2);
    rel_angles_buf = zeros(1, K_MAX); rel_alphas_buf = zeros(1, K_MAX);
    actual_k = 0;
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
    rel_angles = rel_angles_buf(1:actual_k); rel_alphas = rel_alphas_buf(1:actual_k);
end

% === 策略 C ===
function [residual, rel_alphas, rel_angles] = run_relax_core_C(input_signal, K_MAX, improve_th, search_ang, M, spacingCal, noise_th)
    total_raw_energy = sum(abs(input_signal(:)).^2);
    if total_raw_energy < noise_th
        residual = input_signal; rel_alphas = []; rel_angles = []; return; 
    end
    high_res_search_ang = min(search_ang) : 0.1 : max(search_ang);
    A_scan = exp(1j * pi * (0:M-1)' * spacingCal * sind(high_res_search_ang)); 
    residual = input_signal;
    prev_energy = total_raw_energy;
    rel_angles_buf = zeros(1, K_MAX); rel_alphas_buf = zeros(1, K_MAX);
    actual_k = 0;
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
    rel_angles = rel_angles_buf(1:actual_k); rel_alphas = rel_alphas_buf(1:actual_k);
end

function total_sig = reconstruct_signal_batch_local(alphas, angles, M, N_Snaps, spacingCal)
    total_sig = zeros(M, N_Snaps);
    for k = 1:length(alphas)
        a_vec = exp(1j * pi * (0:M-1)' * spacingCal * sind(angles(k))); 
        total_sig = total_sig + alphas(k) * a_vec; 
    end
end