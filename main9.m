% =========================================================================
% 终极雷达流水线: RELAX 旁瓣抑制 + 全图Capon连续检测 + 几何后置截断 + 时空防鬼影跟踪
% =========================================================================
close all; clearvars; clear global; clc;
addpath(genpath(pwd));
addpath(genpath('my_tracking')); % 引入刚刚剥离纯净的 tracking 沙盒
%% ================== 全局变量与 Tracking 初始化 ==================
global p clusters iFrm trackCand trackConfirm trackLost trackWait trajectory ovlpRec sepRec multivRec assocRec
% 1. 载入并初始化 Tracking 全局参数
p = trackParamConfig(0);    % tracking的初始化配置
START_FRAME = 35;
TOTAL_FRAMES = 210;
p.nFrmLoad = TOTAL_FRAMES;
p.iFrmLoad = 1:TOTAL_FRAMES;
% 2. 强行覆盖一些可能导致崩溃的开关 (确保处于纯净模式)
p.multiverseEn = 0; p.ovlpProcEn = 0; p.identifyEn = 0; p.staticEnhEn = 0;


% tracking参数修改测试
p.trackAlgo = 'KF';
p.candWin = 6;
p.presRatioNew = 0.5;
p.nFrmNotGhost = 2;
p.costCand = 0.45;
p.costConfirm = 0.35;
p.waitZoneEn = 0;
p.backtrackEn = 0;
p.smthWin = 5;
p.nSmthNewConfirm = 1;
p.nFrmLost = 3;



% 3. 初始化 Tracking 内存结构
trackCand = struct('centroid', [], 'kalmanFilter', [], 'presence', [], 'ghostLabel', [], 'age', [], 'trajectory', [], 'frame', []);
trackConfirm = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], 'name', [], 'pc', [], 'status', [], 'statusAge', [], 'trajectory', [], 'frame', []);
trackLost = struct('centroid', [], 'iPeople', [], 'name', [], 'trajectory', [], 'frame', []);
trackWait = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], 'name', [], 'age', []);
assocRec = struct('frame', 1, 'association', 1 : 10);
% 防止内存崩溃
for f = 1 : TOTAL_FRAMES
    trajectory(f).track = struct('iPeople', [], 'name', [], 'trajectory', [], 'frame', [], 'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []);
    ovlpRec(f).ovlp = struct('idxSet', []);
    sepRec(f).sep = struct('idxSet', [], 'nSeperate', []);
    multivRec(f).multiv = struct('iMultiverse', 1, 'track', struct('iPeople', [], 'name', [], 'trajectory', [], 'frame', [], 'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []), 'brother', [], 'parent', [], 'association', []);
end
p.nPpl = 0; p.backtrackFlag = 0; p.sepDelFlag = 0;
%% ================== 雷达参数与环境初始化 ==================
PARAM_K_MAX = 6;                % K的最大值
IMPROVE_TH = 0.01;              % 能量改善阈值
BLOCK_SIZE = 32;                % 快拍分块大小
% 自车运动配置
EGO_VELOCITY = 0.363; RADAR_YAW = -30; FRAME_PERIOD = 50e-3;
% 雷达物理网格与硬件配置
CFG_LIMIT_ANG = [-90, 90]; CFG_RES_ANG = 0.5; CFG_LIMIT_R = [];
config2243;
try load('config.mat', 'resR', 'resV', 'spacingCal', 'cfarParamRA');
catch, load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal', 'cfarParamRA'); end
% 坐标系映射
nAdc = 256;
full_grid.range = resR * (0 : nAdc - 1)';
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
X_Plot = Rng_Grid .* sind(Ang_Grid); Y_Plot = Rng_Grid .* cosd(Ang_Grid);
% ground_truth构建
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11]; ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
% 预创绘图对象
hFig = figure('Name', 'Final Pipeline: RELAX + Tracking', 'Position', [100, 100, 1200, 600]);
ax1 = subplot(1, 2, 1); hold on; axis equal; grid on; title('Phase 1: RELAX Clean Heatmap');
h_pcolor = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot))); set(h_pcolor, 'EdgeColor', 'none'); shading interp; colormap(ax1, 'jet');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
ax2 = subplot(1, 2, 2); hold on; axis equal; grid on; title('Phase 2-4: CFAR & Target Tracking');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
%% ================== 核心大循环 ==================
fprintf('================ 启动 ================\n');
% 提前将 global clusters 强制定义为结构体数组，包含 velocity 字段
empty_cluster = struct('pc', [], 'centroid', [], 'velocity', [], 'ghostLabel', []);
empty_cluster(1) = [];
empty_rslt = struct('cluster', empty_cluster, ...
    'noise', struct('pc', []), ...
    'pcInput', [], 'pw', [], 'clusterIdx', []);
clusters = repmat(empty_rslt, TOTAL_FRAMES, 1);
for idx_frm = START_FRAME : TOTAL_FRAMES
    iFrm = idx_frm;
    current_time = (iFrm - 1) * FRAME_PERIOD;

    %% 【Phase 1】: 信号处理层 (提取干净的 pwRA_Clean)
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
        % 核心动作：在 Raw Data 域减去强干扰 (此时尚未丢失相位)
        sorted_data_clean(:, :, r_idx) = snapshot_full - interference_full;
    end

    % =====================================================================
    % 【修改点 1: 全域 Capon 空间谱估计】
    % 废弃之前的“只算 Mask 内部”，直接全图计算，保留自然、连续的背景噪声！
    % =====================================================================
    pwRA_Clean = zeros(nAdc, length(full_grid.angle));
    for iRg = 1 : nAdc
        % 我们对全视野执行 dbf(Capon)，杜绝 0 边界的产生
        [pw_subset, ~] = dbf(full_grid.angle', [], sorted_data_clean(:, :, iRg), antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
        pwRA_Clean(iRg, :) = pw_subset;
    end

    %% 【Phase 2】: 桥接层 (CFAR 目标检测 + 几何截断 + 多普勒测后提取)

    % =====================================================================
    % 【修改点 2: 保持物理连续性的 2D-CFAR】
    % 直接将连续热力图送入 CFAR，任何"强制置0"的门限过滤全部移到 CFAR 之后！
    % =====================================================================
    [cfar_iRange, cfar_iAngle, ~] = cfar2D(pwRA_Clean, cfarParamRA);

    if ~isempty(cfar_iRange)
        % === [后置过滤网] ROI内软门限 ===
        SOFT_THRESHOLD_DB = -18;   % 当前先固定这个值

        roi_vals = pwRA_Clean(ShadowMask == 1);
        if ~isempty(roi_vals)
            roi_peak = max(roi_vals(:));
            roi_threshold = roi_peak * 10^(SOFT_THRESHOLD_DB / 10);
        else
            roi_threshold = inf;
        end

        % =================================================================
        % 【修改点 3: 几何与能量后置截断 (Post-Detection Truncation)】
        % 逐点检查 CFAR 吐出的目标，只保留 ROI 内且高于 ROI 软门限的点
        % =================================================================
        valid_mask_idx = false(length(cfar_iRange), 1);
        for k = 1:length(cfar_iRange)
            r_idx = cfar_iRange(k);
            a_idx = cfar_iAngle(k);

            if ShadowMask(r_idx, a_idx) == 1 && pwRA_Clean(r_idx, a_idx) >= roi_threshold
                valid_mask_idx(k) = true;
            end
        end
        % 刷新 CFAR 检测点索引
        cfar_iRange = cfar_iRange(valid_mask_idx);
        cfar_iAngle = cfar_iAngle(valid_mask_idx);

        % 如果过滤完还有幸存点，再进行坐标映射和手术刀过滤
        if ~isempty(cfar_iRange)
            pc_R = full_grid.range(cfar_iRange);
            pc_A = full_grid.angle(cfar_iAngle)';
            pc_X = pc_R .* sind(pc_A);
            pc_Y = pc_R .* cosd(pc_A);
            pc_coords = [pc_X, pc_Y];
            pc_pw = pwRA_Clean(sub2ind(size(pwRA_Clean), cfar_iRange, cfar_iAngle));

            % === [手术刀 2] 空间先验边界抗拒 (剔除贴着车身的假目标) ===
            EXCLUDE_MARGIN = 0.35;
            car_poly = [car_pts.all_x(:), car_pts.all_y(:)];
            valid_idx = true(size(pc_coords, 1), 1);
            num_poly_pts = size(car_poly, 1);

            for k = 1:size(pc_coords, 1)
                d_min = inf;
                for v = 1:num_poly_pts
                    p1 = car_poly(v, :);
                    p2 = car_poly(mod(v, num_poly_pts)+1, :);
                    d = point_to_line_segment_dist(pc_coords(k,:), p1, p2);
                    if d < d_min, d_min = d; end
                end
                if d_min < EXCLUDE_MARGIN
                    valid_idx(k) = false;
                end
            end

            pc_coords = pc_coords(valid_idx, :);
            pc_pw = pc_pw(valid_idx);
            pc_R = pc_R(valid_idx);
            pc_A = pc_A(valid_idx);
            % ===================================

            % === [手术刀 3] 测后多普勒速度回溯提取 ===
            if exist('pc_coords', 'var') && ~isempty(pc_coords)
                pc_Vr = zeros(size(pc_coords, 1), 1);
                [M_ant, N_chirp] = size(squeeze(sorted_data_clean(:, :, 1)));
                vel_axis = resV * (-N_chirp / 2 : N_chirp / 2 - 1)';

                for k = 1 : size(pc_coords, 1)
                    curr_R_idx = find(abs(full_grid.range - pc_R(k)) < 1e-4, 1);
                    curr_Ang = pc_A(k);

                    clean_snap = squeeze(sorted_data_clean(:, :, curr_R_idx));
                    a_vec = exp(1j * pi * (0:M_ant-1)' * spacingCal * sind(curr_Ang));
                    signal_slow_time = (a_vec' * clean_snap);

                    signal_slow_time = signal_slow_time .* hanning(N_chirp)';
                    dop_spec = abs(fftshift(fft(signal_slow_time, N_chirp)));

                    [~, max_v_idx] = max(dop_spec);
                    pc_Vr(k) = vel_axis(max_v_idx);
                end
            end
            % =========================================================
            if ~isempty(pc_coords)
                clusterRslt = pcCluster2D(pc_coords, 'pw', pc_pw, 'epsilon', p.epsilon, 'minpts', 2, 'limitX', p.limitX, 'limitY', p.limitY);

                for c_idx = 1:length(clusterRslt.cluster)
                    curr_cluster_pts_idx = clusterRslt.clusterIdx == c_idx;
                    if any(curr_cluster_pts_idx)
                        clusterRslt.cluster(c_idx).velocity = median(pc_Vr(curr_cluster_pts_idx));
                    else
                        clusterRslt.cluster(c_idx).velocity = 0;
                    end
                end
            else
                clusterRslt = struct('cluster', struct('pc', [], 'centroid', [], 'velocity', []), 'noise', struct('pc', []), 'pcInput', [], 'pw', [], 'clusterIdx', []);
            end

        else
            clusterRslt = struct('cluster', struct('pc', [], 'centroid', [], 'velocity', []), 'noise', struct('pc', []), 'pcInput', [], 'pw', [], 'clusterIdx', []);
        end

    else
        clusterRslt = struct('cluster', struct('pc', [], 'centroid', [], 'velocity', []), 'noise', struct('pc', []), 'pcInput', [], 'pw', [], 'clusterIdx', []);
    end

    %% 【Phase 3】: 空间防鬼影层
    clusterRslt = ghostInit(clusterRslt);
    clusterRslt = ghostLabeling(clusterRslt);
    clusters(iFrm) = clusterRslt;

    %% 【Phase 4】: 时序 Tracking 层
    if iFrm == START_FRAME
        for iCluster = 1 : structLength(clusters(iFrm).cluster, 'centroid')
            trackCand(iCluster) = struct('centroid', clusters(iFrm).cluster(iCluster).centroid, ...
                'kalmanFilter', createNewKF(clusters(iFrm).cluster(iCluster).centroid, 'motionType', p.motionType), ...
                'presence', 1, 'ghostLabel', clusters(iFrm).cluster(iCluster).ghostLabel, ...
                'age', 1, 'trajectory', clusters(iFrm).cluster(iCluster).centroid, 'frame', iFrm);
        end
    else
        kfPredict();

        if isfield(clusters(iFrm).cluster, 'velocity') && ~isempty(clusters(iFrm).cluster)
            vel_data = vertcat(clusters(iFrm).cluster.velocity);
        else
            vel_data = zeros(structLength(clusters(iFrm).cluster, 'centroid'), 1);
        end

        clusterNew1 = struct('centroid', vertcat(clusters(iFrm).cluster.centroid), ...
            'ghostLabel', vertcat(clusters(iFrm).cluster.ghostLabel), ...
            'velocity', vel_data, ...
            'pc', {{clusters(iFrm).cluster.pc}'});

        if p.waitZoneEn; waitZoneProcess(clusterNew1); end

        assocRslt1 = trackAssociation(trackConfirm, vertcat(clusterNew1.centroid), clusterNew1.ghostLabel, p.costConfirm);
        [clusterNew1, assocRslt1] = confirmZoneProcess(clusterNew1, assocRslt1);

        if ~isempty(assocRslt1.unassignedDetections)
            iUnassign = assocRslt1.unassignedDetections;
            clusterNew2 = struct('centroid', clusterNew1.centroid(iUnassign, :), ...
                'ghostLabel', clusterNew1.ghostLabel(iUnassign), ...
                'velocity', clusterNew1.velocity(iUnassign), ...
                'pc', {clusterNew1.pc(iUnassign)});
            assocRslt2 = trackAssociation(trackCand, vertcat(clusterNew2.centroid), clusterNew2.ghostLabel, p.costCand);
            candidateZoneProcess(clusterNew2, assocRslt2);
        else
            renewCandidate(0, []);
        end
        renewTrajectory();
    end

%% ================== 动态可视化 ==================
    if ~ishandle(hFig), break; end
    
    title(ax1, sprintf('Phase 1: Full-Field Capon Heatmap (Frame: %d)', iFrm), 'FontSize', 12, 'FontWeight', 'bold');
    title(ax2, sprintf('Phase 2-4: Mask Filtered CFAR & Tracking (Frame: %d)', iFrm), 'FontSize', 12, 'FontWeight', 'bold');
    
    % =====================================================================
    % 【视觉解欺骗】：基于 ROI 的局部动态范围颜色映射
    % 核心逻辑：底层送给 CFAR 的仍是保留真实底噪的 pwRA_Clean 全图矩阵，
    % 但画图时，强行将颜色映射的上限锁定为“ROI 掩膜内部”的最大值。
    % =====================================================================
    disp_pw = pwRA_Clean;
    
    % 1. 提取 ROI（白色虚线框）内部的真实能量最大值（通常就是行人的峰值）
    roi_max = max(disp_pw(ShadowMask == 1));
    
    % 2. 更新热力图的数据（原封不动地传进去，不破坏物理底噪）
    set(h_pcolor, 'CData', disp_pw);
    
    % 3. 强行重置绘图坐标轴的颜色映射范围 (Color Axis)
    if ~isempty(roi_max) && roi_max > 0
        % 动态范围设为 20dB (数值越小，对比度越强，你可以根据效果微调成 15 或 25)
        DYNAMIC_RANGE_DB = 20; 
        caxis(ax1, [roi_max * 10^(-DYNAMIC_RANGE_DB/10), roi_max]);
    end
    % =====================================================================
    
    % 刷新掩膜轮廓线
    delete(findobj(ax1, 'Type', 'contour')); 
    contour(ax1, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1);
    
    % 刷新右侧追踪图
    cla(ax2); 
    contour(ax2, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'k--', 'LineWidth', 1); 
    
    % 绘制合法的原始 CFAR 点云 (灰色小点)
    if ~isempty(cfar_iRange)
        if exist('pc_coords', 'var') && ~isempty(pc_coords)
            scatter(ax2, pc_coords(:,1), pc_coords(:,2), 10, [0.7 0.7 0.7], 'filled');
        end
    end
    
    % 绘制被判定为鬼影的聚类簇 (青色大叉)
    if structLength(clusters(iFrm).cluster, 'centroid') > 0
        all_centroids = vertcat(clusters(iFrm).cluster.centroid);
        ghost_flags = vertcat(clusters(iFrm).cluster.ghostLabel);
        if any(ghost_flags)
            ghost_pts = all_centroids(ghost_flags == 1, :);
            scatter(ax2, ghost_pts(:,1), ghost_pts(:,2), 80, 'c', 'x', 'LineWidth', 2);
        end
    end
    
    % 绘制 Confirm 状态的确认追踪目标 (红色实线轨迹 + 红色五角星 + 速度标签)
    for iT = 1 : structLength(trackConfirm, 'centroid')
        traj_pts = trackConfirm(iT).trajectory;
        plot(ax2, traj_pts(:,1), traj_pts(:,2), 'r-', 'LineWidth', 2);
        scatter(ax2, trackConfirm(iT).centroid(1), trackConfirm(iT).centroid(2), 100, 'r', 'p', 'filled');
        
        % 动态查找当前确认目标在这一帧对应的测得速度
        disp_vel = 0;
        if structLength(clusters(iFrm).cluster, 'centroid') > 0
            dists = sum((vertcat(clusters(iFrm).cluster.centroid) - trackConfirm(iT).centroid).^2, 2);
            [min_dist, min_idx] = min(dists);
            if min_dist < 0.5 
                disp_vel = clusters(iFrm).cluster(min_idx).velocity;
            end
        end
        
        text(ax2, trackConfirm(iT).centroid(1)+0.2, trackConfirm(iT).centroid(2), ...
            sprintf('ID:%d, V:%.2f', trackConfirm(iT).iPeople, disp_vel), ...
            'Color', 'r', 'FontWeight', 'bold');
    end
    
    % 绘制 Candidate 状态的候选追踪目标 (黄色虚线轨迹 + 黄色圆点)
    for iC = 1 : structLength(trackCand, 'centroid')
        traj_pts = trackCand(iC).trajectory;
        plot(ax2, traj_pts(:,1), traj_pts(:,2), 'y--', 'LineWidth', 1);
        scatter(ax2, trackCand(iC).centroid(1), trackCand(iC).centroid(2), 50, 'y', 'o', 'filled');
    end
    
    drawnow;
end
fprintf('================ 处理完成 ================\n');



%% ================= 附属函数 (RELAX 及几何计算) =================
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

    if k > 1 && improvement < improve_th, break; end

    rel_angles_buf(k) = curr_ang;
    rel_alphas_buf(k) = curr_alpha;
    residual = temp_residual;
    prev_energy = current_energy;
    actual_k = k;
end

if actual_k == 0, rel_alphas = []; rel_angles = []; return; end

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
% === 新增空间几何核心函数：计算点到线段的最短距离 ===
function d = point_to_line_segment_dist(pt, v1, v2)
l2 = sum((v1 - v2).^2);
if l2 == 0
    d = norm(pt - v1);
    return;
end
t = max(0, min(1, dot(pt - v1, v2 - v1) / l2));
proj = v1 + t * (v2 - v1);
d = norm(pt - proj);
end