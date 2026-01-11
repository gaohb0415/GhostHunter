% =========================================================================
% 脚本名称: Experiment_K_R_Sweep.m
% 功能: 暴力遍历 K (3~20)，寻找每帧的最佳 K 值，并生成 R-K 散点图
% 设定: RELAX迭代固定5次，距离基于车身左下角
% =========================================================================

%% 1. 初始化与配置
close all; clear; clc; 
addpath(genpath(pwd)); 

% --- 动力学参数 ---
EGO_VELOCITY = 0.363;        % 雷达车速度 (m/s)
RADAR_YAW    = -30;          % 雷达安装偏角 (度)
TOTAL_FRAMES = 210;          % 总帧数
FRAME_PERIOD = 50e-3;        % 帧周期

% --- 载入配置 ---
try config2243; catch, warning('config2243 not found'); end
try load('config.mat', 'resR', 'resV'); catch, load('.\config\config\config.mat', 'resR', 'resV'); end

% --- 网格构建 ---
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 

% --- 实验真值数据 ---
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];

%% 2. 实验容器初始化
% 存储最终结果的容器
result_R = []; % 距离
result_K = []; % 最佳K
result_E = []; % 对应的残差能量

% K值扫描范围
K_SCAN_RANGE = 3 : 20; 

fprintf('==================================================\n');
fprintf('开始全量 K-R 遍历实验\n');
fprintf('扫描 K 范围: %d ~ %d | 固定 Iter: 5次\n', min(K_SCAN_RANGE), max(K_SCAN_RANGE));
fprintf('==================================================\n');

h_wait = waitbar(0, '正在暴力扫描 K 值 (计算量较大，请耐心等待)...');

%% 3. 帧循环
for iFrm = 1 : TOTAL_FRAMES
    
    % --- 3.1 数据读取 ---
    try 
        radarData = readBin(iFrm, 0); 
    catch 
        continue; 
    end
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    
    % --- 3.2 严格几何计算 (计算 R) ---
    current_time = (iFrm - 1) * FRAME_PERIOD;
    
    % 1. 计算世界坐标系下车的当前位置
    dy = EGO_VELOCITY * current_time;
    pts_world = car_init_mat; % 初始位置
    pts_world(:, 2) = pts_world(:, 2) - dy; % 模拟靠近
    
    % 2. 转换到雷达坐标系
    theta = -RADAR_YAW;
    X_radar = pts_world(:, 1) * cosd(theta) - pts_world(:, 2) * sind(theta);
    Y_radar = pts_world(:, 1) * sind(theta) + pts_world(:, 2) * cosd(theta);
    
    % 3. 寻找“雷达视角左下角”
    % 定义：在雷达视角中，X 最小（最左）且 Y 较小的点
    % 这里我们简单粗暴：计算4个角点到原点的距离，取最小的那个作为 R
    % (通常左下角或右下角是最近点，符合物理反射规律)
    dists = sqrt(X_radar.^2 + Y_radar.^2);
    R_current = min(dists);
    
    % 过滤：如果距离超出实验有效范围（比如跑到雷达背后或太远），跳过
    if R_current > 15 || R_current < 0.5
        continue;
    end
    
    % --- 3.3 锁定目标 ROI ---
    % 找到能量最强的距离门作为代表
    dist_mask = abs(full_grid.range - R_current) < 3.0; % 放宽一点范围搜峰值
    roi_indices = find(dist_mask);
    if isempty(roi_indices), continue; end
    
    range_energy = sum(abs(fftRsltRg(roi_indices, :, :, :)).^2, [2,3,4]);
    [~, max_e_idx] = max(range_energy);
    best_r_idx = roi_indices(max_e_idx);
    
    % 准备 RELAX 输入数据 snapshot
    raw_slice = squeeze(fftRsltRg(best_r_idx, :, :, :));
    if ndims(raw_slice) == 3
        [nCh, nRx, nTx] = size(raw_slice);
        raw_slice = reshape(raw_slice, nCh, nRx*nTx);
    end
    snapshot = raw_slice.'; % [M x N]
    [M, N_Snaps] = size(snapshot);
    
    % --- 3.4 核心：K 值暴力扫描 ---
    % 确定角度搜索范围
    all_angles = atan2d(X_radar, Y_radar);
    search_ang = (min(all_angles)-2) : 0.5 : (max(all_angles)+2);
    A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang));
    
    energy_curve = zeros(size(K_SCAN_RANGE));
    
    % === 内层循环：测试每一个 K ===
    for k_idx = 1 : length(K_SCAN_RANGE)
        test_K = K_SCAN_RANGE(k_idx);
        
        % -- 运行 RELAX (固定5次迭代) --
        % 初始化
        residual = snapshot;
        rel_angles = zeros(1, test_K);
        rel_alphas = zeros(1, test_K);
        
        % 1. 初始化估计 (无迭代)
        for k = 1 : test_K
            spec = sum(abs(A_scan' * residual), 2);
            [~, idx] = max(spec);
            rel_angles(k) = search_ang(idx);
            a_k = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            rel_alphas(k) = (a_k' * residual(:,1)) / (a_k' * a_k);
            residual = residual - rel_alphas(k) * a_k;
        end
        
        % 2. 迭代优化 (固定5次)
        for iter = 1 : 5
            for k = 1 : test_K
                % Add back
                data_k = snapshot;
                for other = 1 : test_K
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
        
        % 3. 计算最终残差能量并记录
        % 重构信号
        recon = zeros(size(snapshot));
        for k = 1 : test_K
            a_final = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            recon = recon + rel_alphas(k) * a_final;
        end
        resid_final = snapshot - recon;
        energy_curve(k_idx) = sum(abs(resid_final(:)).^2);
    end
    
    % --- 3.5 拐点选择 (Elbow Point Detection) ---
    % 使用“最大距离法”：连接曲线首尾的直线，寻找离直线最远的点
    
    % 归一化 (防止能量绝对值过大影响计算)
    x_vec = 1 : length(K_SCAN_RANGE);
    y_vec = energy_curve;
    % 简单的几何投影法找拐点
    p1 = [x_vec(1), y_vec(1)];
    p2 = [x_vec(end), y_vec(end)];
    % 直线方程 Ax + By + C = 0
    % 向量 P1P2
    vec_line = p2 - p1;
    % 对于每个点 P0，计算到直线的距离
    dists_to_line = zeros(size(x_vec));
    for i = 1 : length(x_vec)
        p0 = [x_vec(i), y_vec(i)];
        vec_p1p0 = p0 - p1;
        % 叉乘求面积 / 底边长 = 高 (距离)
        % abs(x1*y2 - x2*y1)
        dists_to_line(i) = abs(vec_line(1)*vec_p1p0(2) - vec_line(2)*vec_p1p0(1)) / norm(vec_line);
    end
    
    % 找到距离直线最远的点，即为拐点
    [~, best_idx] = max(dists_to_line);
    k_opt = K_SCAN_RANGE(best_idx);
    
    % 存储结果
    result_R = [result_R, R_current];
    result_K = [result_K, k_opt];
    result_E = [result_E, energy_curve(best_idx)];
    
    waitbar(iFrm/TOTAL_FRAMES, h_wait, sprintf('Frm %d: R=%.2fm -> Opt_K=%d', iFrm, R_current, k_opt));
end
close(h_wait);

%% 4. 画图 (R-K 散点图)
% 按照要求，生成 R-K 图，不拟合，只画点
figure('Name', 'R-K Distribution', 'Position', [100, 100, 1000, 600]);

% 颜色映射：根据距离显示颜色
scatter(result_R, result_K, 60, result_R, 'filled', 'MarkerEdgeColor', 'k');
colormap('jet');
colorbar;
ylabel(colorbar, 'Distance R (m)');

grid on;
box on;
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% 坐标轴设置
xlabel('Distance to Car (m) [Radar to Bottom-Left Corner]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Optimal K Value (Elbow Point)', 'FontSize', 14, 'FontWeight', 'bold');
title(['Optimal K vs Distance (Total Frames: ' num2str(length(result_R)) ')'], 'FontSize', 16);

% 设置 X 轴方向：如果是要表现“靠近过程”，可以将X轴反向
set(gca, 'XDir', 'reverse'); % 这样左边是远距离，右边是近距离(靠近过程)
% 或者保持默认从小到大，看你的习惯。这里默认反向，符合“从远到近”的视觉流。

% 添加辅助线
yline(3, 'k--', 'Min K (3)');
yline(20, 'k--', 'Max K (20)');

fprintf('图表绘制完成。X轴已设置为反向（从远到近）。\n');