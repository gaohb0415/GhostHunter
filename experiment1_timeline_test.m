function experiment1_timeline_test()
% 实验1：多帧全流程耗时对比 (纯净计时版)
% 目的: 证明 K=0 策略在动态场景下能随目标面积变化而大幅节省算力
% 说明: 本脚本基于原主调函数精简修改，移除了所有绘图操作，专注于核心算法的耗时统计。

    %% 1. 初始化与配置 (与原主函数完全对齐)
    close all; clear; clc; 
    addpath(genpath(pwd));
    
    % 核心参数
    PARAM_K_MAX = 6;       
    IMPROVE_TH  = 0.01;    
    BLOCK_SIZE  = 32;      
    NOISE_FLOOR_TH = 1e9; % 【关键参数】底噪阈值，根据你原代码 slice_energy > 1e9 设定

    % 动力学参数
    EGO_VELOCITY = 0.363;        
    RADAR_YAW    = -30;          
    FRAME_PERIOD = 50e-3;
    START_FRAME  = 15;
    END_FRAME    = 210;
    
    % 扫描参数
    CFG_LIMIT_ANG = [-90, 90];      
    CFG_RES_ANG   = 0.5;            
    CFG_LIMIT_R   = [];
    
    % 载入配置与网格构建
    config2243; 
    try load('config.mat', 'resR', 'resV', 'spacingCal'); 
    catch, load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal'); end
    if ~exist('spacingCal', 'var'); spacingCal = 1; end
    
    nAdc = 256; 
    full_grid.range = resR * (0 : nAdc - 1)';                                   
    full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);        
    
    % 实验真值数据
    ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
    ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
    car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
    
    %% 2. 核心性能测试循环
    num_frames = END_FRAME - START_FRAME + 1;
    time_A_history = zeros(1, num_frames); % 策略A：固定 K=6
    time_B_history = zeros(1, num_frames); % 策略B：保底 K=1
    time_C_history = zeros(1, num_frames); % 策略C：引入 K=0
    
    fprintf('================ 开始多帧全流程耗时测试 ================\n');
    
    for idx_frm = 1 : num_frames
        iFrm = START_FRAME + idx_frm - 1;
        current_time = (iFrm - 1) * FRAME_PERIOD;
        
        if mod(iFrm, 10) == 0
            fprintf('正在处理 Frame %d / %d...\n', iFrm, END_FRAME);
        end
        
        % ---------------- 雷达数据读取与预处理 (公共部分，不计入核心耗时) ----------------
        try radarData = readBin(iFrm, 0); catch, break; end
        [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
        antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');     
        sorted_data = antArray_Sorted.signal; 
        
        car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
        
        all_corner_rhos = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);    
        car_rho_min = min(all_corner_rhos);
        car_rho_max = max(all_corner_rhos);
        dist_mask = (full_grid.range > (car_rho_min - 0.5)) & (full_grid.range < (car_rho_max + 0.5));
        car_range_indices = find(dist_mask);
        
        all_angles = atan2d(car_pts.all_x, car_pts.all_y);          
        search_ang_global = (min(all_angles) - 5) : 0.5 : (max(all_angles) + 5); 
        
        % ---------------- 分别测试三种策略的执行时间 ----------------
        
        % [策略 A: 一刀切，固定 K=6]
        tic;
        process_frame_with_strategy(sorted_data, car_range_indices, full_grid.range, BLOCK_SIZE, PARAM_K_MAX, IMPROVE_TH, search_ang_global, spacingCal, 'A', 0);
        time_A_history(idx_frm) = toc;
        
        % [策略 B: 原始自适应，保底 K=1]
        tic;
        process_frame_with_strategy(sorted_data, car_range_indices, full_grid.range, BLOCK_SIZE, PARAM_K_MAX, IMPROVE_TH, search_ang_global, spacingCal, 'B', 0);
        time_B_history(idx_frm) = toc;
        
        % [策略 C: 最终自适应，引入 K=0]
        tic;
        process_frame_with_strategy(sorted_data, car_range_indices, full_grid.range, BLOCK_SIZE, PARAM_K_MAX, IMPROVE_TH, search_ang_global, spacingCal, 'C', NOISE_FLOOR_TH);
        time_C_history(idx_frm) = toc;
    end
    
    %% 3. 结果统计与可视化
    % 过滤掉读取失败导致的 0 耗时帧
    valid_idx = time_A_history > 0;
    t_A_valid = time_A_history(valid_idx);
    t_B_valid = time_B_history(valid_idx);
    t_C_valid = time_C_history(valid_idx);
    frames_axis = START_FRAME : (START_FRAME + length(t_A_valid) - 1);
    
    avg_A = mean(t_A_valid);
    avg_B = mean(t_B_valid);
    avg_C = mean(t_C_valid);
    
    fprintf('\n================ 测试总结 ================\n');
    fprintf('测试总帧数: %d 帧\n', length(t_A_valid));
    fprintf('策略A (固定K=6) 平均单帧耗时: %.4f 秒\n', avg_A);
    fprintf('策略B (保底K=1) 平均单帧耗时: %.4f 秒\n', avg_B);
    fprintf('策略C (引入K=0) 平均单帧耗时: %.4f 秒\n', avg_C);
    fprintf('-> 相比策略A，策略B 总体提速 %.1f%%\n', (avg_A - avg_B)/avg_A * 100);
    fprintf('-> 相比策略B，策略C 总体再次提速 %.1f%%\n', (avg_B - avg_C)/avg_B * 100);

    % --- 图表绘制 ---
    figure('Name', 'Pipeline Timing Analysis', 'Position', [100, 100, 1200, 500]);
    
    % 子图 1: 平均耗时对比 (柱状图)
    subplot(1, 2, 1);
    bar_handle = bar([avg_A, avg_B, avg_C], 'FaceColor', 'flat');
    bar_handle.CData(1,:) = [0.8 0.3 0.3]; % 红
    bar_handle.CData(2,:) = [0.3 0.6 0.8]; % 蓝
    bar_handle.CData(3,:) = [0.2 0.7 0.3]; % 绿
    set(gca, 'XTickLabel', {'A: Fixed K=6', 'B: Adaptive (K>=1)', 'C: Adaptive (K>=0)'}, 'FontSize', 10);
    ylabel('Average Execution Time per Frame (s)', 'FontSize', 12);
    title('Average Computational Cost', 'FontSize', 14);
    grid on;
    text(1, avg_A, sprintf('%.3fs', avg_A), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(2, avg_B, sprintf('%.3fs', avg_B), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(3, avg_C, sprintf('%.3fs', avg_C), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    
    % 子图 2: 逐帧动态耗时 (折线图)
    subplot(1, 2, 2);
    plot(frames_axis, t_A_valid, 'r-', 'LineWidth', 1.5, 'DisplayName', 'A: Fixed K=6'); hold on;
    plot(frames_axis, t_B_valid, 'b-', 'LineWidth', 1.5, 'DisplayName', 'B: Adaptive (K>=1)');
    plot(frames_axis, t_C_valid, 'g-', 'LineWidth', 2, 'DisplayName', 'C: Adaptive (K>=0)');
    xlabel('Frame Number', 'FontSize', 12);
    ylabel('Execution Time (s)', 'FontSize', 12);
    title('Dynamic Execution Time vs Frame', 'FontSize', 14);
    legend('Location', 'best');
    grid on;
    % 

end

%% ================= 核心处理封装函数 =================
function process_frame_with_strategy(sorted_data, car_range_indices, full_grid_range, BLOCK_SIZE, PARAM_K_MAX, IMPROVE_TH, search_ang_global, spacingCal, strategy_mode, noise_th)
    % 封装单帧的处理逻辑，避免代码重复
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
            
            % 根据策略模式选择调用的 RELAX 核心逻辑
            switch strategy_mode
                case 'A'
                    [~, alphas, angles] = run_relax_core_A(snapshot_batch, PARAM_K_MAX, search_ang_global, M, spacingCal);
                case 'B'
                    [~, alphas, angles] = run_relax_core_B(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, spacingCal);
                case 'C'
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

%% ================= 三种策略的具体实现 =================

% 策略A：一刀切跑满 K_MAX 次
function [residual, rel_alphas, rel_angles] = run_relax_core_A(input_signal, K_MAX, search_ang, M, spacingCal)
    high_res_search_ang = min(search_ang) : 0.1 : max(search_ang);
    A_scan = exp(1j * pi * (0:M-1)' * spacingCal * sind(high_res_search_ang)); 
    residual = input_signal;
    rel_angles_buf = zeros(1, K_MAX); rel_alphas_buf = zeros(1, K_MAX);
    
    for k = 1:K_MAX
        spec = sum(abs(A_scan' * residual), 2); 
        [~, idx] = max(spec);
        curr_ang = high_res_search_ang(idx);
        a_k = exp(1j * pi * (0:M-1)' * spacingCal * sind(curr_ang));
        curr_alpha = (a_k' * residual(:,1)) / (a_k' * a_k);
        residual = residual - curr_alpha * a_k;
        rel_angles_buf(k) = curr_ang; rel_alphas_buf(k) = curr_alpha;
    end
    rel_angles = rel_angles_buf; rel_alphas = rel_alphas_buf;
end

% 策略B：至少跑 1 次，随后基于改善率跳出
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
        
        if k > 1 && improvement < improve_th
            break; 
        end
        rel_angles_buf(k) = curr_ang; rel_alphas_buf(k) = curr_alpha;
        residual = temp_residual; prev_energy = current_energy; actual_k = k;
    end
    rel_angles = rel_angles_buf(1:actual_k); rel_alphas = rel_alphas_buf(1:actual_k);
end

% 策略C：前置底噪判决 (K=0 逻辑)
function [residual, rel_alphas, rel_angles] = run_relax_core_C(input_signal, K_MAX, improve_th, search_ang, M, spacingCal, noise_th)
    total_raw_energy = sum(abs(input_signal(:)).^2);
    % --- K=0 核心跳出判断 ---
    if total_raw_energy < noise_th
        residual = input_signal;
        rel_alphas = []; rel_angles = [];
        return; 
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
        
        if k > 1 && improvement < improve_th
            break; 
        end
        rel_angles_buf(k) = curr_ang; rel_alphas_buf(k) = curr_alpha;
        residual = temp_residual; prev_energy = current_energy; actual_k = k;
    end
    rel_angles = rel_angles_buf(1:actual_k); rel_alphas = rel_alphas_buf(1:actual_k);
end

% 辅助重构函数
function total_sig = reconstruct_signal_batch_local(alphas, angles, M, N_Snaps, spacingCal)
    total_sig = zeros(M, N_Snaps);
    for k = 1:length(alphas)
        a_vec = exp(1j * pi * (0:M-1)' * spacingCal * sind(angles(k))); 
        total_sig = total_sig + alphas(k) * a_vec; 
    end
end