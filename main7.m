% =========================================================================
% SOTA 雷达流水线: RELAX 旁瓣抑制 + NLOS 异构多维粒子滤波 (PF-TBD)
% =========================================================================
close all; clearvars; clear global; clc; 
addpath(genpath(pwd));
addpath(genpath('my_tracking'));

%% ================== 全局变量与 SOTA PF-TBD 初始化 ==================
START_FRAME = 30;
TOTAL_FRAMES = 210;
FRAME_PERIOD = 50e-3;
H_RADAR = 0.375; 

% --- PF-TBD 核心参数 ---
pf_params.N_particles = 1000;      
pf_params.state_dim = 4;           
pf_params.T = FRAME_PERIOD;
sigma_a = 2.0; 
pf_params.Q = (sigma_a^2) * [ (pf_params.T^4)/4, 0, (pf_params.T^3)/2, 0;
                              0, (pf_params.T^4)/4, 0, (pf_params.T^3)/2;
                              (pf_params.T^3)/2, 0, pf_params.T^2, 0;
                              0, (pf_params.T^3)/2, 0, pf_params.T^2 ];
                          
particles = zeros(pf_params.state_dim, pf_params.N_particles);
particles(1, :) = -4 + 8 * rand(1, pf_params.N_particles);
particles(2, :) = 2 + 8 * rand(1, pf_params.N_particles);
particles(3, :) = -3 + 6 * rand(1, pf_params.N_particles);
particles(4, :) = -3 + 6 * rand(1, pf_params.N_particles);
weights = ones(1, pf_params.N_particles) / pf_params.N_particles;
est_trajectory = [];

%% ================== 雷达参数与环境初始化 ==================
PARAM_K_MAX = 6;                
IMPROVE_TH = 0.01;              
BLOCK_SIZE = 32;                
EGO_VELOCITY = 0.363; RADAR_YAW = -30; 
CFG_LIMIT_ANG = [-90, 90]; CFG_RES_ANG = 0.5; CFG_LIMIT_R = [];

config2243; 
try load('config.mat', 'resR', 'resV', 'maxV', 'spacingCal'); 
catch, load('.\config\config\config.mat', 'resR', 'resV', 'maxV', 'spacingCal'); end

nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)';                                   
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);        
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);          
X_Plot = Rng_Grid .* sind(Ang_Grid); Y_Plot = Rng_Grid .* cosd(Ang_Grid);

nChirps = 128; 
vel_axis = linspace(-maxV, maxV, nChirps);
el_axis = -45 : 1 : 45; 

ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11]; ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];

hFig = figure('Name', 'SOTA Pipeline: RELAX + PF-TBD', 'Position', [100, 100, 1200, 600]);
ax1 = subplot(1, 2, 1); hold on; axis equal; grid on; title('Phase 1: RELAX Clean Heatmap');
h_pcolor = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot))); set(h_pcolor, 'EdgeColor', 'none'); shading interp; colormap(ax1, 'jet');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
ax2 = subplot(1, 2, 2); hold on; axis equal; grid on; title('Phase 2: Multi-dimensional PF-TBD');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');

%% ================== 核心大循环 ==================
fprintf('================ 启动 SOTA PF-TBD (带计时器) ================\n');

for idx_frm = START_FRAME : TOTAL_FRAMES
    iFrm = idx_frm;
    fprintf('--- 处理第 %d 帧 ---\n', iFrm);
    
    %% 【阶段 0】：数据读取与基本 FFT
    tic_total = tic; % 记录整帧开始时间
    tic_stage = tic;
    try radarData = readBin(iFrm, 0); catch, break; end
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');     
    sorted_data = antArray_Sorted.signal; 
    
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);    
    
    all_corner_rhos = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);    
    car_range_indices = find((full_grid.range > min(all_corner_rhos) - 0.5) & (full_grid.range < max(all_corner_rhos) + 0.5));
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);          
    search_ang_global = (min(all_angles) - 5) : 0.5 : (max(all_angles) + 5); 
    t_stage0 = toc(tic_stage);
    fprintf('  [阶段0] 读取数据与前置准备耗时: %.4f 秒\n', t_stage0);
    
    %% 【阶段 1A】：RELAX 核心解算
    tic_stage = tic;
    sorted_data_clean = sorted_data; 
    for r_idx = car_range_indices'
        current_R = full_grid.range(r_idx);
        snapshot_full = squeeze(sorted_data(:, :, r_idx)); 
        [M, N_Total] = size(snapshot_full);
        interference_full = zeros(M, N_Total);
        
        num_blocks = ceil(N_Total / BLOCK_SIZE);
        for b = 1:num_blocks
            idx_start = (b-1) * BLOCK_SIZE + 1; idx_end = min(b * BLOCK_SIZE, N_Total);
            current_indices = idx_start : idx_end;
            snapshot_batch = snapshot_full(:, current_indices);
            [~, alphas, angles] = run_relax_core_batch_v2(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, search_ang_global, M, spacingCal, 0);
            if ~isempty(alphas)
                interference_full(:, current_indices) = reconstruct_signal_batch_v2(alphas, angles, M, length(current_indices), spacingCal);
            end
        end
        sorted_data_clean(:, :, r_idx) = snapshot_full - interference_full;
    end
    t_stage1a = toc(tic_stage);
    fprintf('  [阶段1A] RELAX 交替投影算法耗时: %.4f 秒\n', t_stage1a);
    
    %% 【阶段 1B】：Capon (DBF) 波束形成
    tic_stage = tic;
    pwRA_Clean = zeros(nAdc, length(full_grid.angle));
    for iRg = 1 : nAdc
        if any(ShadowMask(iRg, :) == 1)
             valid_idx = find(ShadowMask(iRg, :) == 1);
             [pw_subset, ~] = dbf(full_grid.angle(valid_idx)', [], sorted_data_clean(:, :, iRg), antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
             pwRA_Clean(iRg, valid_idx) = pw_subset;
        end
    end
    t_stage1b = toc(tic_stage);
    fprintf('  [阶段1B] Capon (DBF) 波束形成耗时: %.4f 秒\n', t_stage1b);
    
   %% 【阶段 1.5】：生成辅助张量 (Doppler & Elevation)
    tic_stage = tic;
    
    % 仅保留极速的 1D Doppler FFT
    fftRsltRD_raw = fftshift(fft(sorted_data, [], 2), 2);
    RD_Heatmap_2D = squeeze(mean(abs(fftRsltRD_raw), 1));
    RD_Heatmap = RD_Heatmap_2D.';
    
    % ！！！强行关闭下面这行极其耗时的 2D 角度解算 ！！！
    % [~, heatmapAE] = fftAngle2D(fftRsltRg, 'limitR', [0, 10], 'drawEn', 0); 
    
    t_stage15 = toc(tic_stage);
    fprintf('  [阶段1.5] 多普勒张量生成耗时: %.4f 秒\n', t_stage15);
    
    %% 【阶段 2】：SOTA PF-TBD (空间 + 多普勒双维异构查表)
    tic_stage = tic;
    
    % 1. 状态转移预测
    F_mat = [1, 0, pf_params.T, 0; 0, 1, 0, pf_params.T; 0, 0, 1, 0; 0, 0, 0, 1];
    process_noise = mvnrnd(zeros(1, 4), pf_params.Q, pf_params.N_particles)'; 
    particles = F_mat * particles + process_noise;
    
    px = particles(1, :); py = particles(2, :);
    pvx = particles(3, :); pvy = particles(4, :); % 提取粒子速度
    
    R_bounce = sqrt(px.^2 + py.^2 + H_RADAR^2);
    az_angle = atan2d(px, py);
    v_radial = (px.*pvx + py.*pvy) ./ R_bounce; % 计算径向速度
    
    idx_R = round(R_bounce / resR) + 1;
    idx_Az = round((az_angle - full_grid.angle(1)) / CFG_RES_ANG) + 1;
    idx_V = round((v_radial - vel_axis(1)) / (vel_axis(2) - vel_axis(1))) + 1;
    
    valid_mask = (idx_R > 0 & idx_R <= size(pwRA_Clean, 1)) & ...
                 (idx_Az > 0 & idx_Az <= size(pwRA_Clean, 2)); 
             
    L_space = ones(1, pf_params.N_particles) * 1e-10; 
    L_doppler = ones(1, pf_params.N_particles) * 1e-10; % 唤醒多普勒底分
    
    if any(valid_mask)
        % === A. 空间 Capon 查表 (动态峰值锐化) ===
        lin_idx_RA = sub2ind(size(pwRA_Clean), idx_R(valid_mask), idx_Az(valid_mask));
        val_capon = pwRA_Clean(lin_idx_RA);
        
        % 铁腕手段：提取当前帧的绝对最高峰值！
        max_capon = max(pwRA_Clean(:));
        if max_capon < 1e-6, max_capon = 1e-6; end
        
        % 将能量强制归一化到 [0, 1]，消灭底噪干扰
        val_capon_norm = val_capon / max_capon;
        
        % 只有能量达到全局峰值 10% 以上的像素才配得分，过滤一切假弱峰
        active_mask = val_capon_norm > 0.1;
        
        temp_scores = ones(1, sum(valid_mask)) * 1e-10; 
        % 指数极度锐化：满分拉到 exp(20)，真实目标与底噪的权重差距被拉大到上亿倍！
        temp_scores(active_mask) = exp(val_capon_norm(active_mask) * 20); 
        L_space(valid_mask) = temp_scores;
        
        % === B. 多普勒速度查表 (解除横穿误杀) ===
        valid_V_mask = valid_mask & (idx_V > 0 & idx_V <= size(RD_Heatmap, 2));
        if any(valid_V_mask)
            lin_idx_RD = sub2ind(size(RD_Heatmap), idx_R(valid_V_mask), idx_V(valid_V_mask));
            val_rd = RD_Heatmap(lin_idx_RD);
            
            % 同样执行动态峰值归一化
            max_rd = max(RD_Heatmap(:));
            if max_rd < 1e-6, max_rd = 1e-6; end
            val_rd_norm = val_rd / max_rd;
            
            active_rd_mask = val_rd_norm > 0.05;
            temp_dop_scores = ones(1, sum(valid_V_mask)) * 1e-10;
            temp_dop_scores(active_rd_mask) = exp(val_rd_norm(active_rd_mask) * 10);
            
            % --- 修正防卫过当：惩罚系数从 0.001 宽容到 0.2 ---
            % 允许行人在横穿马路或短暂犹豫时存在瞬时的极低径向速度！
            static_mask = abs(v_radial(valid_V_mask)) < 0.3;
            temp_dop_scores(static_mask) = temp_dop_scores(static_mask) * 0.2;
            
            L_doppler(valid_V_mask) = temp_dop_scores;
        end
    end
    
    % 3. 权重双维融合与归一化
    weights = weights .* L_space .* L_doppler;
    sum_w = sum(weights);
    
    if isnan(sum_w) || sum_w < 1e-30
        weights = ones(1, pf_params.N_particles) / pf_params.N_particles;
    else
        weights = weights / sum_w;
    end
    
    % 4. 核心修复：带侦察兵注入的高效重采样
    N_eff = 1 / sum(weights.^2);
    
    if N_eff < pf_params.N_particles * 0.75 || max(weights) < (1.5 / pf_params.N_particles)
        cumsum_w = cumsum(weights);
        cumsum_w = cumsum_w / cumsum_w(end); 
        edges = [0, cumsum_w];
        edges(end) = 1.0001; 
        
        for k_edge = 2:length(edges)
            if edges(k_edge) < edges(k_edge-1)
                edges(k_edge) = edges(k_edge-1) + eps;
            end
        end
        
        [~, idx] = histc(rand(1, pf_params.N_particles), edges);
        particles = particles(:, idx);
        weights = ones(1, pf_params.N_particles) / pf_params.N_particles;
        
        % 10% 伞兵注入 (防迷失)
        N_scouts = round(pf_params.N_particles * 0.1); 
        particles(1, 1:N_scouts) = -4 + 8 * rand(1, N_scouts); 
        particles(2, 1:N_scouts) = 2 + 8 * rand(1, N_scouts);  
        particles(3, 1:N_scouts) = -3 + 6 * rand(1, N_scouts); 
        particles(4, 1:N_scouts) = -3 + 6 * rand(1, N_scouts); 
    end
    
    % 5. 状态提取
    if var(particles(1,:)) < 4.0 && var(particles(2,:)) < 4.0
        est_state = sum(particles .* weights, 2);
        est_trajectory = [est_trajectory; est_state(1), est_state(2)];
    end
    
    t_stage2 = toc(tic_stage);
    
    %% 【阶段 3】：绘图与渲染更新
    tic_stage = tic;
    if ~ishandle(hFig), break; end
    title(ax1, sprintf('Phase 1: RELAX Heatmap (Frm: %d)', iFrm));
    set(h_pcolor, 'CData', pwRA_Clean);
    delete(findobj(ax1, 'Type', 'contour')); contour(ax1, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1);
    
    cla(ax2); 
    contour(ax2, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'k--', 'LineWidth', 1); 
    
    scatter(ax2, particles(1, :), particles(2, :), 5, [0.5 0.9 0.5], 'filled', 'MarkerFaceAlpha', 0.5);
    
    if ~isempty(est_trajectory)
        plot(ax2, est_trajectory(:, 1), est_trajectory(:, 2), 'r-', 'LineWidth', 3);
        scatter(ax2, est_trajectory(end, 1), est_trajectory(end, 2), 150, 'r', 'p', 'filled');
        text(ax2, est_trajectory(end, 1)+0.3, est_trajectory(end, 2), 'Target (NLOS)', 'Color', 'r', 'FontWeight', 'bold');
    end
    drawnow;
    t_stage3 = toc(tic_stage);
    fprintf('  [阶段 3] 图像渲染耗时: %.4f 秒\n', t_stage3);
    
    t_total = toc(tic_total);
    fprintf('=> 第 %d 帧总耗时: %.4f 秒\n', iFrm, t_total);
end
fprintf('================ 处理完成 ================\n');

%% ================= 附属函数 (真·RELAX 核心) =================
function [residual, rel_alphas, rel_angles] = run_relax_core_batch_v2(input_signal, K_MAX, improve_th, search_ang, M, spacingCal, noise_th)
    total_raw_energy = sum(abs(input_signal(:)).^2);
    if total_raw_energy < noise_th
        residual = input_signal; rel_alphas = []; rel_angles = []; return; 
    end
    
    high_res_search_ang = min(search_ang) : 0.1 : max(search_ang);
    A_scan = exp(1j * pi * (0:M-1)' * spacingCal * sind(high_res_search_ang)); 
    
    residual = input_signal; prev_energy = total_raw_energy;
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
        
        rel_angles_buf(k) = curr_ang; 
        rel_alphas_buf(k) = curr_alpha;
        residual = temp_residual; 
        prev_energy = current_energy; 
        actual_k = k;
    end
    
    if actual_k == 0
        rel_alphas = []; rel_angles = []; return; 
    end
    
    rel_angles = rel_angles_buf(1:actual_k); 
    rel_alphas = rel_alphas_buf(1:actual_k);
    
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
            
            spec = sum(abs(A_scan' * data_k), 2); 
            [~, idx] = max(spec); 
            rel_angles(k) = high_res_search_ang(idx);
            
            a_new = exp(1j * pi * (0:M-1)' * spacingCal * sind(rel_angles(k))); 
            rel_alphas(k) = (a_new' * data_k(:,1)) / (a_new' * a_new);
        end
    end
    
    residual = input_signal;
    for k = 1:actual_k
        a_final = exp(1j * pi * (0:M-1)' * spacingCal * sind(rel_angles(k)));
        residual = residual - rel_alphas(k) * a_final;
    end
end

function total_sig = reconstruct_signal_batch_v2(alphas, angles, M, N_Snaps, spacingCal)
    total_sig = zeros(M, N_Snaps);
    for k = 1:length(alphas)
        a_vec = exp(1j * pi * (0:M-1)' * spacingCal * sind(angles(k))); 
        total_sig = total_sig + alphas(k) * a_vec; 
    end
end