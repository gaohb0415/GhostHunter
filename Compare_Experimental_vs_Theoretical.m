% =========================================================================
% 脚本名称: Compare_Experimental_vs_Theoretical_FullRELAX.m
% 功能: 对比 "全血版RELAX拐点(红点)" 与 "理论公式值(蓝点)"
% 更新: 红点计算已替换为完整的 RELAX 迭代算法
% =========================================================================

%% 1. 初始化
close all; clear; clc; 
addpath(genpath(pwd)); 

% --- 动力学与几何参数 ---
EGO_VELOCITY = 0.363;        
RADAR_YAW    = -30;          
TOTAL_FRAMES = 210;          
FRAME_PERIOD = 50e-3;        

% --- 理论公式参数 (根据你的调整保持一致) ---
CAR_WIDTH_GT  = 1.76; 
RADAR_RES_ANG = 1.5;  
BETA_FACTOR   = 0.66;  % 稀疏系数

% --- 载入配置 ---
try config2243; catch, warning('config2243 not found'); end
try load('config.mat', 'resR', 'resV'); catch, load('.\config\config\config.mat', 'resR', 'resV'); end

% --- 实验真值数据 ---
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];

% --- 结果容器 ---
data_R     = []; 
data_K_exp = []; 
data_K_theo= []; 

fprintf('开始对比实验 (Full RELAX Mode)：共 %d 帧...\n', TOTAL_FRAMES);
fprintf('注意：由于开启了 RELAX 迭代，计算速度会稍慢，请耐心等待。\n');
h_wait = waitbar(0, '正在进行硬核计算...');

%% 2. 循环处理每一帧
for iFrm = 1 : TOTAL_FRAMES
    
    % --- 2.1 几何距离 R 计算 ---
    current_time = (iFrm - 1) * FRAME_PERIOD;
    dy = EGO_VELOCITY * current_time;
    
    pts_world = car_init_mat; 
    pts_world(:, 2) = pts_world(:, 2) - dy; 
    theta = -RADAR_YAW;
    X_radar = pts_world(:, 1) * cosd(theta) - pts_world(:, 2) * sind(theta);
    Y_radar = pts_world(:, 1) * sind(theta) + pts_world(:, 2) * cosd(theta);
    
    dists = sqrt(X_radar.^2 + Y_radar.^2);
    R_current = min(dists); 
    
    if R_current > 15 || R_current < 0.5, continue; end
    
    % --- 2.2 理论值计算 (Blue Dots) ---
    angle_span_deg = atan2d(CAR_WIDTH_GT, R_current);
    k_theory_val = ceil( (angle_span_deg / RADAR_RES_ANG) * BETA_FACTOR );
    if k_theory_val > 20, k_theory_val = 20; end
    
    % --- 2.3 实验值挖掘 (Red Dots - Full RELAX) ---
    try radarData = readBin(iFrm, 0); catch, continue; end
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    
    % ROI 峰值搜索
    nAdc = 256; full_grid_range = resR * (0 : nAdc - 1)';
    dist_mask = abs(full_grid_range - R_current) < 3.0;
    roi_indices = find(dist_mask);
    if isempty(roi_indices), continue; end
    
    range_energy = sum(abs(fftRsltRg(roi_indices, :, :, :)).^2, [2,3,4]);
    [~, max_e_idx] = max(range_energy);
    best_r_idx = roi_indices(max_e_idx);
    
    raw_slice = squeeze(fftRsltRg(best_r_idx, :, :, :));
    if ndims(raw_slice) == 3, [nCh, nRx, nTx] = size(raw_slice); raw_slice = reshape(raw_slice, nCh, nRx*nTx); end
    snapshot = raw_slice.'; [M, ~] = size(snapshot);
    
    % 字典构建
    all_angles = atan2d(X_radar, Y_radar);
    search_ang = (min(all_angles)-5) : 0.5 : (max(all_angles)+5);
    A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang));
    
    % --- 暴力扫描 K (3~50) ---
    K_SCAN_RANGE = 3:50;
    energy_curve = zeros(size(K_SCAN_RANGE));
    
    for k_idx = 1 : length(K_SCAN_RANGE)
        test_K = K_SCAN_RANGE(k_idx);
        
        % ==================================================
        % 完整版 RELAX 算法实现
        % ==================================================
        % 初始化存储
        rel_angles = zeros(1, test_K);
        rel_alphas = zeros(1, test_K);
        residual = snapshot;
        
        % Step 1: Initialization (CLEAN-like)
        for k = 1 : test_K
             spec = sum(abs(A_scan' * residual), 2);
             [~, idx] = max(spec);
             rel_angles(k) = search_ang(idx);
             
             a_k = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
             rel_alphas(k) = (a_k' * residual(:,1)) / (a_k' * a_k);
             
             residual = residual - rel_alphas(k) * a_k;
        end
        
        % Step 2: Iterative Optimization (The "RELAX" part)
        MAX_RELAX_ITER = 5; % 迭代次数，与主函数保持一致
        for iter = 1 : MAX_RELAX_ITER
            for k = 1 : test_K
                % Add back others
                data_k = snapshot;
                for other = 1 : test_K
                    if other ~= k
                        a_other = exp(-1j * pi * (0:M-1)' * sind(rel_angles(other)));
                        data_k = data_k - rel_alphas(other) * a_other;
                    end
                end
                
                % Re-estimate k
                spec = sum(abs(A_scan' * data_k), 2);
                [~, idx] = max(spec);
                rel_angles(k) = search_ang(idx); % 更新角度
                
                a_new = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
                rel_alphas(k) = (a_new' * data_k(:,1)) / (a_new' * a_new); % 更新幅度
            end
        end
        
        % Step 3: Compute Final Residual Energy
        recon_signal = zeros(size(snapshot));
        for k = 1 : test_K
            a_final = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            recon_signal = recon_signal + rel_alphas(k) * a_final;
        end
        final_resid = snapshot - recon_signal;
        energy_curve(k_idx) = sum(abs(final_resid(:)).^2);
        % ==================================================
    end
    
    % 拐点计算 (Elbow Detection)
    x_vec = 1 : length(K_SCAN_RANGE); y_vec = energy_curve;

    
    p1 = [x_vec(1), y_vec(1)]; p2 = [x_vec(end), y_vec(end)];
    vec_line = p2 - p1;


    dists_to_line = zeros(size(x_vec));
    for i = 1 : length(x_vec)
        p0 = [x_vec(i), y_vec(i)]; vec_p1p0 = p0 - p1;
        dists_to_line(i) = abs(vec_line(1)*vec_p1p0(2) - vec_line(2)*vec_p1p0(1)) / norm(vec_line);
    end
    [~, best_idx] = max(dists_to_line);
    k_exp_val = K_SCAN_RANGE(best_idx);
    
    % --- 存储 ---
    data_R      = [data_R, R_current];
    data_K_exp  = [data_K_exp, k_exp_val];
    data_K_theo = [data_K_theo, k_theory_val];
    
    waitbar(iFrm/TOTAL_FRAMES, h_wait, sprintf('R=%.2fm | FullRELAX=%d vs Theo=%d', R_current, k_exp_val, k_theory_val));
end
close(h_wait);

%% 3. 画图对比 (R-K Plot)
figure('Name', 'Theoretical vs Full RELAX K', 'Position', [100, 100, 1000, 600]);
hold on; grid on; box on;

% 1. 绘制理论值 (蓝点)
scatter(data_R, data_K_theo, 50, 'b', 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Theoretical Model (Fixed)');

% 2. 绘制实验值 (红点)
scatter(data_R, data_K_exp, 50, 'r', 'filled', 'MarkerFaceAlpha', 0.7, 'DisplayName', 'Experimental Truth (Full RELAX)');

% 3. 装饰
xlabel('Distance to Car (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Optimal K Value', 'FontSize', 12, 'FontWeight', 'bold');
title(sprintf('Comparison: Formula (Beta=%.2f) vs. Actual RELAX', BETA_FACTOR), 'FontSize', 14);
legend('Location', 'best');

set(gca, 'XDir', 'reverse'); 

% 趋势线
[p_theo, ~] = polyfit(1./data_R, data_K_theo, 1);
x_fit = linspace(min(data_R), max(data_R), 100);
plot(x_fit, p_theo(1)./x_fit + p_theo(2), 'b--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

fprintf('绘制完成。这才是真正的 RELAX 性能基准。\n');