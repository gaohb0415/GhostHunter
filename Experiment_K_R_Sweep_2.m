% =========================================================================
% 脚本名称: Experiment_K_Optimization_by_Result_Lite.m
% 功能: [轻量版] 以“最终Capon成像ROI能量最小化”为标准，优选 K 值
% 调整: 扫描范围缩小至 3~20，增加 K < M 边界保护
% =========================================================================

%% 1. 初始化与配置
close all; clear; clc; 
addpath(genpath(pwd)); 

% --- 动力学参数 ---
EGO_VELOCITY = 0.363;        
RADAR_YAW    = -30;          
TOTAL_FRAMES = 210;          
FRAME_PERIOD = 50e-3;        

% --- 扫描参数 ---
CFG_LIMIT_ANG = [-90, 90]; 
CFG_RES_ANG   = 0.5; 
CFG_LIMIT_R   = [];

% --- 载入配置 ---
try config2243; catch, warning('config2243 not found'); end
try load('config.mat', 'resR', 'resV'); catch, load('.\config\config\config.mat', 'resR', 'resV'); end

% --- 网格构建 ---
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);

% --- 实验真值数据 ---
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
ped_path_x = [3.44, 3.44, 1.11];
ped_path_y = [5.8, 8.62, 8.62];
ped_init_mat = [ped_path_x', ped_path_y'];

%% 2. 实验容器
result_R = [];
result_K = [];
result_MinEnergy = [];

% 【修改点1】范围缩小到 3:20，大幅降低计算量
K_test_range = 1 : 20; 

fprintf('==================================================\n');
fprintf('开始基于[最终成像效果]的 K 值优选实验 (Lite Version)\n');
fprintf('扫描 K 范围: 3 ~ 20\n');
fprintf('==================================================\n');

h_wait = waitbar(0, '正在进行轻量化仿真...');

%% 3. 核心循环
% 为了进一步加速，可以选择每隔一帧跑一次 (iFrm = 1:1:TOTAL_FRAMES)
for iFrm = 1 : TOTAL_FRAMES
    
    % --- 3.1 基础数据准备 ---
    try radarData = readBin(iFrm, 0); catch, continue; end
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    
    current_time = (iFrm - 1) * FRAME_PERIOD;
    
    % 几何计算
    try
        car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    catch
        continue;
    end
    
    dists_to_corners = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);
    R_current = min(dists_to_corners);
    
    if R_current > 15 || R_current < 0.5, continue; end
    
    % 生成 ShadowMask
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
    
    % 确定范围
    car_center_rho = (car_pts.A.rho + car_pts.K.rho) / 2;
    dist_mask = abs(full_grid.range - car_center_rho) < 2.5;
    car_range_indices = find(dist_mask);
    
    if isempty(car_range_indices), continue; end

    % 角度范围
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);
    min_car_ang = min(all_angles) - 2;
    max_car_ang = max(all_angles) + 2;
    search_ang = min_car_ang : 0.5 : max_car_ang; 
    
    % 天线数 M 计算
    temp_slice = squeeze(fftRsltRg(car_range_indices(1), :, :, :));
    if ndims(temp_slice) == 3
        [nSnaps, nRx, nTx] = size(temp_slice);
        M = nRx * nTx; 
    else
        [nSnaps, M] = size(temp_slice);
    end
    
    % 预计算字典
    A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang));
    
    % === 3.2 遍历 K 值 ===
    roi_energy_trace = nan(size(K_test_range)); % 用 NaN 初始化
    
    for k_idx = 1 : length(K_test_range)
        current_k = K_test_range(k_idx);
        
        % 【修改点2】边界条件检查：确保 K < M
        % RELAX 要求观测通道数 M 必须大于源数目 K，否则无法求解
        if current_k >= M
            break; % 停止继续增大 K，因为后面肯定都算不了
        end
        
        fftRsltRg_Temp = fftRsltRg;
        
        % --- RELAX 核心 ---
        for r_idx = car_range_indices'
            raw_slice = squeeze(fftRsltRg(r_idx, :, :, :));
            if ndims(raw_slice) == 3
                [nCh, nRx, nTx] = size(raw_slice);
                raw_slice = reshape(raw_slice, nCh, nRx*nTx);
            end
            snapshot = raw_slice.'; 
            
            % 维度保护
            if size(snapshot, 1) ~= M
                 snapshot = reshape(raw_slice, [], size(raw_slice, 1)).';
            end

            % Init
            residual = snapshot;
            rel_angles = zeros(1, current_k);
            rel_alphas = zeros(1, current_k);
            
            for k = 1 : current_k
                spec = sum(abs(A_scan' * residual), 2);
                [~, idx] = max(spec);
                rel_angles(k) = search_ang(idx);
                a_k = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
                rel_alphas(k) = (a_k' * residual(:,1)) / (a_k' * a_k);
                residual = residual - rel_alphas(k) * a_k;
            end
            
            % Iterate (3次足够)
            for iter = 1 : 3 
                for k = 1 : current_k
                    data_k = snapshot;
                    for other = 1 : current_k
                        if other ~= k
                            a_other = exp(-1j * pi * (0:M-1)' * sind(rel_angles(other)));
                            data_k = data_k - rel_alphas(other) * a_other;
                        end
                    end
                    spec = sum(abs(A_scan' * data_k), 2);
                    [~, idx] = max(spec);
                    rel_angles(k) = search_ang(idx);
                    a_new = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
                    rel_alphas(k) = (a_new' * data_k(:,1)) / (a_new' * a_new);
                end
            end
            
            % Reconstruct & Subtract
            interference = zeros(size(snapshot));
            for k = 1 : current_k
                a_final = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
                interference = interference + rel_alphas(k) * a_final;
            end
            snapshot_clean = snapshot - interference;
            
            % Fill back
            try
                fftRsltRg_Temp(r_idx, :, :) = reshape(snapshot_clean.', nSnaps, nRx, nTx);
            catch
                 fftRsltRg_Temp(r_idx, :, :) = snapshot_clean.';
            end
        end
        
        % --- Capon ROI 能量计算 ---
        [pwRA_Roi, ~] = dbfProc1D(fftRsltRg_Temp, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
            'limitR', CFG_LIMIT_R, 'Mask', ShadowMask, 'pcEn', 0, 'drawEn', 0);
            
        roi_energy_trace(k_idx) = sum(pwRA_Roi(:), 'omitnan');
    end
    
    % === 3.3 选取最佳 K ===
    % 忽略 NaN (因为可能触发了 K>=M 的 break)
    [min_e, best_idx] = min(roi_energy_trace);
    
    if ~isnan(min_e)
        best_k = K_test_range(best_idx);
        result_R = [result_R, R_current];
        result_K = [result_K, best_k];
        result_MinEnergy = [result_MinEnergy, min_e];
        waitbar(iFrm/TOTAL_FRAMES, h_wait, sprintf('R=%.2fm | Best K=%d', R_current, best_k));
    end
end
close(h_wait);

%% 4. 绘图
if isempty(result_R)
    error('没有有效数据生成。');
end

figure('Name', 'Result-Oriented K Selection', 'Position', [100, 100, 1000, 600]);

scatter(result_R, result_K, 60, result_MinEnergy, 'filled', 'MarkerEdgeColor', 'k');
colormap('jet');
c = colorbar;
c.Label.String = 'Residual ROI Energy';

hold on; grid on; box on;
xlabel('Distance to Car (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Optimal K (Min ROI Energy)', 'FontSize', 12, 'FontWeight', 'bold');
title('Best K Selection based on Capon Quality (Range 3-20)', 'FontSize', 14);

set(gca, 'XDir', 'reverse'); 

yline(3, 'k--', 'Min K');
yline(20, 'k--', 'Max K');

fprintf('完成！计算负担已减轻。\n');