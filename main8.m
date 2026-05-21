% =========================================================================
% 峰值引导单目标雷达流水线:
% RELAX车体抑制 + 局部Capon(扩张ROI计算区) + ROI内2D CFAR + 时序跟踪
% + 新增：前端鲁棒遮挡车矩形修正模块
% 显示要求版：
% 1) 不画ROI_calc梯形框
% 2) 两图只画ROI虚线边界
% 3) 左图只显示ROI内热力图，半圆蓝色背景保留
% 4) 右图不显示背景热力图，只保留ROI虚线 + 检测点 + tracking
% 5) ROI顶端两点用平滑弧线连接
% 6) 不再把前后脚二选一，而是优先由双脚点估计人体中心点
% =========================================================================
close all; clearvars; clear global; clc;
addpath(genpath(pwd));
addpath(genpath('my_tracking'));

%% ================== 全局变量与 Tracking 初始化 ==================
global p clusters iFrm trackCand trackConfirm trackLost trackWait trajectory ovlpRec sepRec multivRec assocRec

p = trackParamConfig(0);
START_FRAME = 170;
TOTAL_FRAMES = 207;
p.nFrmLoad = TOTAL_FRAMES;
p.iFrmLoad = 1:TOTAL_FRAMES;

% 纯净 tracking 模式
p.multiverseEn = 0;
p.ovlpProcEn = 0;
p.identifyEn = 0;
p.staticEnhEn = 0;
p.waitZoneEn = 0;
p.backtrackEn = 0;

% tracking 参数
p.trackAlgo = 'KF';
p.candWin = 6;
p.presRatioNew = 0.5;
p.nFrmNotGhost = 2;
p.costCand = 0.45;
p.costConfirm = 0.45;
p.smthWin = 5;
p.nSmthNewConfirm = 1;
p.nFrmLost = 12;

trackCand = struct('centroid', [], 'kalmanFilter', [], 'presence', [], 'ghostLabel', [], 'age', [], 'trajectory', [], 'frame', []);
trackConfirm = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], 'name', [], 'pc', [], 'status', [], 'statusAge', [], 'trajectory', [], 'frame', []);
trackLost = struct('centroid', [], 'iPeople', [], 'name', [], 'trajectory', [], 'frame', []);
trackWait = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], 'name', [], 'age', []);
assocRec = struct('frame', 1, 'association', 1:10);

for f = 1:TOTAL_FRAMES
    trajectory(f).track = struct('iPeople', [], 'name', [], 'trajectory', [], 'frame', [], ...
        'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []);
    ovlpRec(f).ovlp = struct('idxSet', []);
    sepRec(f).sep = struct('idxSet', [], 'nSeperate', []);
    multivRec(f).multiv = struct('iMultiverse', 1, ...
        'track', struct('iPeople', [], 'name', [], 'trajectory', [], 'frame', [], ...
        'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []), ...
        'brother', [], 'parent', [], 'association', []);
end
p.nPpl = 0;
p.backtrackFlag = 0;
p.sepDelFlag = 0;

%% ================== 雷达参数与环境初始化 ==================
PARAM_K_MAX = 6;
IMPROVE_TH = 0.01;
BLOCK_SIZE = 32;
EGO_VELOCITY = 0.363;
RADAR_YAW = -30;
FRAME_PERIOD = 50e-3;
CFG_LIMIT_ANG = [-90, 90];
CFG_RES_ANG = 0.5;
CFG_LIMIT_R = [];

config2243;
try
    load('config.mat', 'resR', 'resV', 'spacingCal', 'cfarParamRA');
catch
    load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal', 'cfarParamRA');
end

nAdc = 256;
full_grid.range = resR * (0:nAdc-1)';
full_grid.angle = CFG_LIMIT_ANG(1):CFG_RES_ANG:CFG_LIMIT_ANG(2);
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
X_Plot = Rng_Grid .* sind(Ang_Grid);
Y_Plot = Rng_Grid .* cosd(Ang_Grid);

ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];

% ===== 局部Capon / 局部CFAR 参数 =====
ROI_MARGIN_R = 1;
ROI_MARGIN_A = 1;
PAD_R = cfarParamRA.train(1) + cfarParamRA.guard(1) + ROI_MARGIN_R;
PAD_A = cfarParamRA.train(2) + cfarParamRA.guard(2) + ROI_MARGIN_A;

% 主峰门控参数
SOFT_THRESHOLD_DB = -15;
PEAK_GATE_R = 0.8;   % m
PEAK_GATE_A = 6.0;   % deg

% 贴车体剔除参数
EXCLUDE_MARGIN = 0.25;

% 说明：这里关闭近邻峰值去重，因为两个近邻点可能对应前后脚
USE_NMS = 0;
NMS_MERGE_R = 0.40;    % m
NMS_MERGE_A = 2.5;     % deg

% 双脚 -> 人体中心 参数
FOOT_PAIR_DIST_MIN = 0.15;   % m
FOOT_PAIR_DIST_MAX = 0.90;   % m
FOOT_PAIR_DIST_PREF = 0.45;  % m
FOOT_PAIR_DIST_SIGMA = 0.18; % m
BODY_REF_SIGMA = 0.60;       % m
BODY_SINGLE_BLEND = 0.65;    % 单脚时当前点与历史中心融合系数

% 显示层：ROI顶部弧线半径（只影响画图）
ROI_ARC_RADIUS = 10.8;

%% ================== 鲁棒矩形修正模块参数 ==================
robustParam = struct();
robustParam.enable = true;

% 观测/时序融合
robustParam.minRawWeight = 0.35;
robustParam.maxRawWeight = 0.80;

% 候选搜索范围（以 seed rectangle 为中心的小范围刚性修正）
robustParam.dxGrid   = -0.30:0.15:0.30;   % m
robustParam.dyGrid   = -0.10:0.10:0.10;   % m
robustParam.dyawGrid = -4:2:4;            % deg

% 各项距离尺度
robustParam.sigmaObs  = struct('x', 0.30, 'y', 0.20, 'yaw', 4.5, 'l', 0.25, 'w', 0.18);
robustParam.sigmaLidar = struct('x', 0.25, 'y', 0.20, 'yaw', 4.0, 'l', 0.25, 'w', 0.18);
robustParam.sigmaTemp  = struct('x', 0.25, 'y', 0.18, 'yaw', 3.0, 'l', 0.20, 'w', 0.15);

% Radar consistency 局部搜索图参数
robustParam.radarRangeMargin = 0.80;   % m
robustParam.radarAngleMargin = 8.0;    % deg
robustParam.radarAngleStep = 0.5;      % deg
robustParam.radarOuterRangePad = 0.35; % m
robustParam.radarOuterAnglePad = 3.0;  % deg
robustParam.radarContrastBeta = 0.35;

% 各项打分权重（会在模块内部按观测可靠性自适应微调）
robustParam.wRadarBase = 0.50;
robustParam.wLidarBase = 0.30;
robustParam.wTempBase  = 0.20;

% 最终状态平滑
robustParam.finalBlendMin = 0.65;
robustParam.finalBlendMax = 0.90;

% 尺度更新平滑
robustParam.sizeBlendAlpha = 0.25;
robustParam.minLength = 2.5;
robustParam.maxLength = 6.5;
robustParam.minWidth  = 1.2;
robustParam.maxWidth  = 2.8;

% 调试记录
robustState = [];
robustMetaRec = repmat(struct('rawState', [], 'predState', [], ...
    'seedState', [], 'refinedState', [], 'scoreInfo', []), TOTAL_FRAMES, 1);

%% ================== 图形初始化 ==================
hFig = figure('Name', 'Local Capon + ROI-CFAR + Tracking', 'Position', [100, 100, 1200, 600]);

ax1 = subplot(1, 2, 1);
hold(ax1, 'on');
axis(ax1, 'equal');
grid(ax1, 'on');
title(ax1, 'Phase 1: Local Capon Heatmap');
xlabel(ax1, 'X (m)');
ylabel(ax1, 'Y (m)');
xlim(ax1, [-11, 11]);
ylim(ax1, [0, 12]);
h_pcolor = pcolor(ax1, X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor, 'EdgeColor', 'none');
shading(ax1, 'interp');
colormap(ax1, 'jet');

ax2 = subplot(1, 2, 2);
hold(ax2, 'on');
axis(ax2, 'equal');
grid(ax2, 'on');
title(ax2, 'Phase 2-4: ROI-CFAR Detection & Tracking');
xlabel(ax2, 'X (m)');
ylabel(ax2, 'Y (m)');
xlim(ax2, [-11, 11]);
ylim(ax2, [0, 12]);

%% ================== 核心大循环 ==================
fprintf('================ 启动 ================\n');
clusters = repmat(makeEmptyClusterResult(), TOTAL_FRAMES, 1);

for idx_frm = START_FRAME:TOTAL_FRAMES
    iFrm = idx_frm;

    %% 【Phase 1】: 信号处理层
    try
        radarData = readBin(iFrm, 0);
    catch
        break;
    end

    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');
    sorted_data = antArray_Sorted.signal;

    % ===== 新增：前端鲁棒矩形修正模块 =====
    car_pts_raw = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    [car_pts, robustState, robustMeta] = robust_blocker_rectangle_refine( ...
        car_pts_raw, sorted_data, full_grid, spacingCal, robustState, robustParam);
    robustMetaRec(iFrm) = robustMeta;

    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
    ROI_target_mask = (ShadowMask == 1);

    if ~any(ROI_target_mask(:))
        clusterRslt = makeEmptyClusterResult();
        clusters(iFrm) = clusterRslt;
        continue;
    end

    % ===== 1) 构造 ROI_target 与外扩计算区 ROI_calc =====
    roiInfo = build_local_roi_info(ROI_target_mask, size(ShadowMask), PAD_R, PAD_A);

    % ===== 2) 仅在车辆相关 range 上做 RELAX 清理 =====
    all_corner_rhos = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);
    car_range_indices = find((full_grid.range > min(all_corner_rhos) - 0.5) & ...
                             (full_grid.range < max(all_corner_rhos) + 0.5));

    all_angles = atan2d(car_pts.all_x, car_pts.all_y);
    search_ang_global = (min(all_angles) - 5):0.5:(max(all_angles) + 5);

    sorted_data_clean = sorted_data;
    for r_idx = car_range_indices'
        snapshot_full = squeeze(sorted_data(:, :, r_idx));
        [M, N_Total] = size(snapshot_full);
        interference_full = zeros(M, N_Total);

        num_blocks = ceil(N_Total / BLOCK_SIZE);
        for b = 1:num_blocks
            idx_start = (b-1) * BLOCK_SIZE + 1;
            idx_end = min(b * BLOCK_SIZE, N_Total);
            current_indices = idx_start:idx_end;
            snapshot_batch = snapshot_full(:, current_indices);

            [~, alphas, angles] = run_relax_core_batch_v2(snapshot_batch, PARAM_K_MAX, ...
                IMPROVE_TH, search_ang_global, M, spacingCal, 0);

            if ~isempty(alphas)
                interference_full(:, current_indices) = reconstruct_signal_batch_v2( ...
                    alphas, angles, M, length(current_indices), spacingCal);
            end
        end
        sorted_data_clean(:, :, r_idx) = snapshot_full - interference_full;
    end

    % ===== 3) 局部 Capon：只在 ROI_calc 上计算 =====
    pwRA_Clean = zeros(nAdc, length(full_grid.angle));

    local_range_idx = roiInfo.calc_rows(1):roiInfo.calc_rows(2);
    local_angle_idx = roiInfo.calc_cols(1):roiInfo.calc_cols(2);
    local_angles = full_grid.angle(local_angle_idx);

    for iRg = local_range_idx
        [pw_subset, ~] = dbf(local_angles', [], sorted_data_clean(:, :, iRg), ...
            antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal, 'pwAlgo', 'Capon');
        pwRA_Clean(iRg, local_angle_idx) = pw_subset(:).';
    end

    %% 【Phase 2】: ROI增强 + 局部CFAR + 主峰约束
    pwRA_Proc = pwRA_Clean;

    % 只在 ROI_target 内做增强
    for iRg = roiInfo.target_rows(1):roiInfo.target_rows(2)
        roi_idx_row = find(ROI_target_mask(iRg, :) == 1);
        if isempty(roi_idx_row)
            continue;
        end
        roi_row = pwRA_Clean(iRg, roi_idx_row);
        bg_val = median(roi_row);
        roi_row_enh = roi_row - bg_val;
        roi_row_enh(roi_row_enh < 0) = 0;
        roi_row_enh = roi_row_enh .^ 1.2;
        pwRA_Proc(iRg, roi_idx_row) = roi_row_enh;
    end

    % ===== 4) 只对 ROI_calc 对应局部 patch 做 CFAR =====
    local_patch_proc = pwRA_Proc(local_range_idx, local_angle_idx);
    [cfar_r_local, cfar_a_local, ~] = cfar2D(local_patch_proc, cfarParamRA);

    % 映射回全局索引
    cfar_iRange = cfar_r_local + roiInfo.calc_rows(1) - 1;
    cfar_iAngle = cfar_a_local + roiInfo.calc_cols(1) - 1;

    pc_coords = [];
    pc_pw = [];
    pc_R = [];
    pc_A = [];
    pc_Vr = [];

    if ~isempty(cfar_iRange)
        keep_target = ROI_target_mask(sub2ind(size(ROI_target_mask), cfar_iRange, cfar_iAngle));
        cfar_iRange = cfar_iRange(keep_target);
        cfar_iAngle = cfar_iAngle(keep_target);
    end

    % ===== 5) ROI内主峰约束 =====
    if ~isempty(cfar_iRange)
        roi_map = pwRA_Proc;
        roi_map(~ROI_target_mask) = 0;
        roi_vals = roi_map(ROI_target_mask);

        if isempty(roi_vals) || max(roi_vals(:)) <= 0
            clusterRslt = makeEmptyClusterResult();
        else
            [roi_peak, peak_lin_idx] = max(roi_map(:));
            [peak_r_idx, peak_a_idx] = ind2sub(size(roi_map), peak_lin_idx);
            peak_R = full_grid.range(peak_r_idx);
            peak_A = full_grid.angle(peak_a_idx);

            roi_threshold = roi_peak * 10^(SOFT_THRESHOLD_DB / 10);

            valid_mask_idx = false(length(cfar_iRange), 1);
            for k = 1:length(cfar_iRange)
                r_idx = cfar_iRange(k);
                a_idx = cfar_iAngle(k);
                curr_R = full_grid.range(r_idx);
                curr_A = full_grid.angle(a_idx);

                inROI = ROI_target_mask(r_idx, a_idx);
                inPeakGate = abs(curr_R - peak_R) <= PEAK_GATE_R && ...
                             abs(curr_A - peak_A) <= PEAK_GATE_A;
                highEnough = pwRA_Proc(r_idx, a_idx) >= roi_threshold;

                if inROI && inPeakGate && highEnough
                    valid_mask_idx(k) = true;
                end
            end

            cfar_iRange = cfar_iRange(valid_mask_idx);
            cfar_iAngle = cfar_iAngle(valid_mask_idx);

            % 可选 NMS：双脚假设下默认关闭
            if USE_NMS
                [cfar_iRange, cfar_iAngle] = suppressNearbyDetectionsRA( ...
                    cfar_iRange, cfar_iAngle, pwRA_Proc, full_grid, NMS_MERGE_R, NMS_MERGE_A);
            end

            % ===== 6) 极坐标转平面点 =====
            if ~isempty(cfar_iRange)
                pc_R = full_grid.range(cfar_iRange);
                pc_A = full_grid.angle(cfar_iAngle)';
                pc_X = pc_R .* sind(pc_A);
                pc_Y = pc_R .* cosd(pc_A);
                pc_coords = [pc_X, pc_Y];
                pc_pw = pwRA_Proc(sub2ind(size(pwRA_Proc), cfar_iRange, cfar_iAngle));

                % ===== 7) 贴车体剔除 =====
                car_poly = [car_pts.all_x(:), car_pts.all_y(:)];
                valid_idx = true(size(pc_coords, 1), 1);
                num_poly_pts = size(car_poly, 1);

                for k = 1:size(pc_coords, 1)
                    d_min = inf;
                    for v = 1:num_poly_pts
                        p1 = car_poly(v, :);
                        p2 = car_poly(mod(v, num_poly_pts) + 1, :);
                        d = point_to_line_segment_dist(pc_coords(k, :), p1, p2);
                        if d < d_min
                            d_min = d;
                        end
                    end
                    if d_min < EXCLUDE_MARGIN
                        valid_idx(k) = false;
                    end
                end

                pc_coords = pc_coords(valid_idx, :);
                pc_pw = pc_pw(valid_idx);
                pc_R = pc_R(valid_idx);
                pc_A = pc_A(valid_idx);

                % ===== 8) 逐点估计径向速度 =====
                if ~isempty(pc_coords)
                    pc_Vr = zeros(size(pc_coords, 1), 1);
                    [M_ant, N_chirp] = size(squeeze(sorted_data_clean(:, :, 1)));
                    vel_axis = resV * (-N_chirp / 2:N_chirp / 2 - 1)';

                    for k = 1:size(pc_coords, 1)
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

                    % ===== 9) 双脚点 -> 人体中心点 =====
                    clusterRslt = buildHumanCenterCluster(pc_coords, pc_pw, pc_Vr, ...
                        trackConfirm, trackCand, FOOT_PAIR_DIST_MIN, FOOT_PAIR_DIST_MAX, ...
                        FOOT_PAIR_DIST_PREF, FOOT_PAIR_DIST_SIGMA, BODY_REF_SIGMA, BODY_SINGLE_BLEND);
                else
                    clusterRslt = makeEmptyClusterResult();
                end
            else
                clusterRslt = makeEmptyClusterResult();
            end
        end
    else
        clusterRslt = makeEmptyClusterResult();
    end

    %% 【Phase 3】: 空间防鬼影层
    clusterRslt = ghostInit(clusterRslt);
    clusterRslt = ghostLabeling(clusterRslt);
    clusters(iFrm) = clusterRslt;

    %% 【Phase 4】: Tracking 层
    if iFrm == START_FRAME
        for iCluster = 1:structLength(clusters(iFrm).cluster, 'centroid')
            trackCand(iCluster) = struct( ...
                'centroid', clusters(iFrm).cluster(iCluster).centroid, ...
                'kalmanFilter', createNewKF(clusters(iFrm).cluster(iCluster).centroid, 'motionType', p.motionType), ...
                'presence', 1, ...
                'ghostLabel', clusters(iFrm).cluster(iCluster).ghostLabel, ...
                'age', 1, ...
                'trajectory', clusters(iFrm).cluster(iCluster).centroid, ...
                'frame', iFrm);
        end
    else
        kfPredict();

        if isfield(clusters(iFrm).cluster, 'velocity') && ~isempty(clusters(iFrm).cluster)
            vel_data = vertcat(clusters(iFrm).cluster.velocity);
        else
            vel_data = zeros(structLength(clusters(iFrm).cluster, 'centroid'), 1);
        end

        if structLength(clusters(iFrm).cluster, 'centroid') > 0
            clusterNew1 = struct( ...
                'centroid', vertcat(clusters(iFrm).cluster.centroid), ...
                'ghostLabel', vertcat(clusters(iFrm).cluster.ghostLabel), ...
                'velocity', vel_data, ...
                'pc', {{clusters(iFrm).cluster.pc}'});
        else
            clusterNew1 = struct('centroid', zeros(0,2), 'ghostLabel', zeros(0,1), ...
                'velocity', zeros(0,1), 'pc', {{}});
        end

        assocRslt1 = trackAssociation(trackConfirm, vertcat(clusterNew1.centroid), ...
            clusterNew1.ghostLabel, p.costConfirm);
        [clusterNew1, assocRslt1] = confirmZoneProcess(clusterNew1, assocRslt1);

        if ~isempty(assocRslt1.unassignedDetections)
            [clusterNew1, assocRslt1] = reactivateLostTracks(clusterNew1, assocRslt1, FRAME_PERIOD);
        end

        if ~isempty(assocRslt1.unassignedDetections)
            iUnassign = assocRslt1.unassignedDetections;
            clusterNew2 = struct( ...
                'centroid', clusterNew1.centroid(iUnassign, :), ...
                'ghostLabel', clusterNew1.ghostLabel(iUnassign), ...
                'velocity', clusterNew1.velocity(iUnassign), ...
                'pc', {clusterNew1.pc(iUnassign)});
            assocRslt2 = trackAssociation(trackCand, vertcat(clusterNew2.centroid), ...
                clusterNew2.ghostLabel, p.costCand);
            candidateZoneProcess(clusterNew2, assocRslt2);
        else
            renewCandidate(0, []);
        end

        renewTrajectory();
    end

    %% ================== 动态可视化（只改显示，不改链路） ==================
    if ~ishandle(hFig)
        break;
    end

    title(ax1, sprintf('Phase 1: Local Capon Heatmap (Frame: %d)', iFrm), ...
        'FontSize', 12, 'FontWeight', 'bold');

    disp_pw = zeros(size(pwRA_Clean));
    disp_pw(ROI_target_mask) = pwRA_Clean(ROI_target_mask);
    set(h_pcolor, 'CData', disp_pw);

    roi_max = max(pwRA_Clean(ROI_target_mask));
    if ~isempty(roi_max) && roi_max > 0
        DYNAMIC_RANGE_DB = 20;
        caxis(ax1, [roi_max * 10^(-DYNAMIC_RANGE_DB / 10), roi_max]);
    end

    xlim(ax1, [-11, 11]);
    ylim(ax1, [0, 12]);

    delete(findobj(ax1, 'Tag', 'roiArcBoundary'));
    draw_roi_boundary_with_arc(ax1, car_pts, ROI_ARC_RADIUS, 'w--', 1.4);

    cla(ax2);
    hold(ax2, 'on');
    axis(ax2, 'equal');
    grid(ax2, 'on');
    xlim(ax2, [-11, 11]);
    ylim(ax2, [0, 12]);
    xlabel(ax2, 'X (m)');
    ylabel(ax2, 'Y (m)');
    title(ax2, sprintf('Phase 2-4: ROI-CFAR Detection & Tracking (Frame: %d)', iFrm), ...
        'FontSize', 12, 'FontWeight', 'bold');

    draw_roi_boundary_with_arc(ax2, car_pts, ROI_ARC_RADIUS, 'k--', 1.4);

    if ~isempty(pc_coords)
        scatter(ax2, pc_coords(:, 1), pc_coords(:, 2), 16, [0.70 0.70 0.70], 'filled');
    end

    if structLength(clusters(iFrm).cluster, 'centroid') > 0
        all_centroids = vertcat(clusters(iFrm).cluster.centroid);
        ghost_flags = vertcat(clusters(iFrm).cluster.ghostLabel);
        if any(ghost_flags)
            ghost_pts = all_centroids(ghost_flags == 1, :);
            scatter(ax2, ghost_pts(:, 1), ghost_pts(:, 2), 80, 'c', 'x', 'LineWidth', 2);
        end
    end

    for iT = 1:structLength(trackConfirm, 'centroid')
        traj_pts = trackConfirm(iT).trajectory;
        plot(ax2, traj_pts(:, 1), traj_pts(:, 2), 'r-', 'LineWidth', 2);
        scatter(ax2, trackConfirm(iT).centroid(1), trackConfirm(iT).centroid(2), ...
            100, 'r', 'p', 'filled');

        disp_vel = 0;
        if structLength(clusters(iFrm).cluster, 'centroid') > 0
            dists = sum((vertcat(clusters(iFrm).cluster.centroid) - ...
                trackConfirm(iT).centroid).^2, 2);
            [min_dist, min_idx] = min(dists);
            if min_dist < 0.5
                disp_vel = clusters(iFrm).cluster(min_idx).velocity;
            end
        end

        text(ax2, trackConfirm(iT).centroid(1) + 0.2, trackConfirm(iT).centroid(2), ...
            sprintf('ID:%d, V:%.2f', trackConfirm(iT).iPeople, disp_vel), ...
            'Color', 'r', 'FontWeight', 'bold');
    end

    for iC = 1:structLength(trackCand, 'centroid')
        traj_pts = trackCand(iC).trajectory;
        plot(ax2, traj_pts(:, 1), traj_pts(:, 2), 'y--', 'LineWidth', 1);
        scatter(ax2, trackCand(iC).centroid(1), trackCand(iC).centroid(2), ...
            50, 'y', 'o', 'filled');
    end

    drawnow;
end

fprintf('================ 处理完成 ================\n');

%% ================= 附属函数 =================
function roiInfo = build_local_roi_info(ROI_target_mask, mapSize, padR, padA)
    [rows, cols] = find(ROI_target_mask);
    if isempty(rows) || isempty(cols)
        error('ROI_target_mask为空，无法构造局部ROI。');
    end

    rMin = min(rows); rMax = max(rows);
    cMin = min(cols); cMax = max(cols);

    nRow = mapSize(1);
    nCol = mapSize(2);

    calc_r1 = max(1, rMin - padR);
    calc_r2 = min(nRow, rMax + padR);
    calc_c1 = max(1, cMin - padA);
    calc_c2 = min(nCol, cMax + padA);

    roiInfo.target_rows = [rMin, rMax];
    roiInfo.target_cols = [cMin, cMax];
    roiInfo.calc_rows = [calc_r1, calc_r2];
    roiInfo.calc_cols = [calc_c1, calc_c2];
end

function draw_roi_boundary_with_arc(ax, car_pts, arc_radius, lineSpec, lineWidth)
    delete(findobj(ax, 'Tag', 'roiArcBoundary'));

    pts = [car_pts.A.x, car_pts.A.y;
           car_pts.B.x, car_pts.B.y;
           car_pts.K.x, car_pts.K.y;
           car_pts.C.x, car_pts.C.y];
    thetas = [car_pts.A.theta, car_pts.B.theta, car_pts.K.theta, car_pts.C.theta];

    [theta_min, idx_left] = min(thetas);
    [theta_max, idx_right] = max(thetas);

    leftPt = pts(idx_left, :);
    rightPt = pts(idx_right, :);

    innerPath = get_blocker_hull_path(car_pts, idx_left, idx_right);

    theta_arc = linspace(theta_min, theta_max, 160);
    x_arc = arc_radius * sind(theta_arc);
    y_arc = arc_radius * cosd(theta_arc);

    plot(ax, [leftPt(1), x_arc(1)], [leftPt(2), y_arc(1)], ...
        lineSpec, 'LineWidth', lineWidth, 'Tag', 'roiArcBoundary');
    plot(ax, x_arc, y_arc, lineSpec, 'LineWidth', lineWidth, 'Tag', 'roiArcBoundary');
    plot(ax, [rightPt(1), x_arc(end)], [rightPt(2), y_arc(end)], ...
        lineSpec, 'LineWidth', lineWidth, 'Tag', 'roiArcBoundary');

    if ~isempty(innerPath)
        plot(ax, innerPath(:,1), innerPath(:,2), ...
            lineSpec, 'LineWidth', lineWidth, 'Tag', 'roiArcBoundary');
    end
end

function innerPath = get_blocker_hull_path(car_pts, idx_left, idx_right)
    poly_x = [0, car_pts.A.x, car_pts.B.x, car_pts.K.x, car_pts.C.x];
    poly_y = [0, car_pts.A.y, car_pts.B.y, car_pts.K.y, car_pts.C.y];

    hull_idx = convhull(poly_x, poly_y);
    hull_idx = hull_idx(:).';
    if hull_idx(end) == hull_idx(1)
        hull_idx(end) = [];
    end

    left_poly_idx = idx_left + 1;
    right_poly_idx = idx_right + 1;

    posL = find(hull_idx == left_poly_idx, 1);
    posR = find(hull_idx == right_poly_idx, 1);

    if isempty(posL) || isempty(posR)
        innerPath = [poly_x([left_poly_idx, right_poly_idx])', ...
                     poly_y([left_poly_idx, right_poly_idx])'];
        return;
    end

    path1 = cyclic_path(hull_idx, posL, posR, 1);
    path2 = cyclic_path(hull_idx, posL, posR, -1);

    hasOrigin1 = any(path1 == 1);
    hasOrigin2 = any(path2 == 1);

    if ~hasOrigin1 && hasOrigin2
        path_use = path1;
    elseif hasOrigin1 && ~hasOrigin2
        path_use = path2;
    elseif ~hasOrigin1 && ~hasOrigin2
        if numel(path1) <= numel(path2)
            path_use = path1;
        else
            path_use = path2;
        end
    else
        path_use = [left_poly_idx, right_poly_idx];
    end

    innerPath = [poly_x(path_use(:))', poly_y(path_use(:))'];
end

function path = cyclic_path(arr, startPos, endPos, direction)
    n = numel(arr);
    path = arr(startPos);

    curr = startPos;
    while curr ~= endPos
        curr = curr + direction;
        if curr > n
            curr = 1;
        elseif curr < 1
            curr = n;
        end
        path(end + 1) = arr(curr); %#ok<AGROW>
    end
end

function [residual, rel_alphas, rel_angles] = run_relax_core_batch_v2(input_signal, K_MAX, improve_th, search_ang, M, spacingCal, noise_th)
    total_raw_energy = sum(abs(input_signal(:)).^2);
    if total_raw_energy < noise_th
        residual = input_signal;
        rel_alphas = [];
        rel_angles = [];
        return;
    end

    high_res_search_ang = min(search_ang):0.1:max(search_ang);
    A_scan = exp(1j * pi * (0:M-1)' * spacingCal * sind(high_res_search_ang));

    residual = input_signal;
    prev_energy = total_raw_energy;
    rel_angles_buf = zeros(1, K_MAX);
    rel_alphas_buf = zeros(1, K_MAX);
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
        rel_alphas = [];
        rel_angles = [];
        return;
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

function rslt = makeEmptyClusterResult()
    empty_cluster = struct('pc', [], 'centroid', [], 'velocity', [], 'ghostLabel', []);
    empty_cluster(1) = [];
    rslt = struct('cluster', empty_cluster, 'noise', struct('pc', []), ...
        'pcInput', [], 'pw', [], 'clusterIdx', []);
end

function [cfar_iRange_keep, cfar_iAngle_keep] = suppressNearbyDetectionsRA(cfar_iRange, cfar_iAngle, pwRA_Proc, full_grid, mergeR, mergeA)
    if isempty(cfar_iRange)
        cfar_iRange_keep = cfar_iRange;
        cfar_iAngle_keep = cfar_iAngle;
        return;
    end

    pw = pwRA_Proc(sub2ind(size(pwRA_Proc), cfar_iRange, cfar_iAngle));
    [~, ord] = sort(pw, 'descend');

    cfar_iRange = cfar_iRange(ord);
    cfar_iAngle = cfar_iAngle(ord);

    nDet = numel(cfar_iRange);
    suppressed = false(nDet, 1);
    keep = false(nDet, 1);

    for i = 1:nDet
        if suppressed(i)
            continue;
        end

        keep(i) = true;
        Ri = full_grid.range(cfar_iRange(i));
        Ai = full_grid.angle(cfar_iAngle(i));

        for j = i+1:nDet
            if suppressed(j)
                continue;
            end

            Rj = full_grid.range(cfar_iRange(j));
            Aj = full_grid.angle(cfar_iAngle(j));

            if abs(Rj - Ri) <= mergeR && abs(angleDiffDeg(Aj, Ai)) <= mergeA
                suppressed(j) = true;
            end
        end
    end

    cfar_iRange_keep = cfar_iRange(keep);
    cfar_iAngle_keep = cfar_iAngle(keep);
end

function d = angleDiffDeg(a, b)
    d = mod((a - b) + 180, 360) - 180;
end

function clusterRslt = buildHumanCenterCluster(pc_coords, pc_pw, pc_Vr, trackConfirm, trackCand, ...
    footDistMin, footDistMax, footDistPref, footDistSigma, refSigma, singleBlend)

    clusterRslt = makeEmptyClusterResult();

    nPts = size(pc_coords, 1);
    if nPts == 0
        return;
    end

    refCenter = getNearestReferenceCenter(trackConfirm, trackCand, mean(pc_coords, 1));
    hasRef = ~isempty(refCenter);

    bestScore = -inf;
    bestPair = [];

    if nPts >= 2
        for i = 1:nPts-1
            for j = i+1:nPts
                dij = norm(pc_coords(i,:) - pc_coords(j,:));
                if dij < footDistMin || dij > footDistMax
                    continue;
                end

                pairCenter = weightedMidpoint(pc_coords([i j], :), pc_pw([i j]));
                powerTerm = sum(pc_pw([i j]));
                distTerm = exp(-((dij - footDistPref)^2) / (2 * footDistSigma^2));

                if hasRef
                    refTerm = exp(-(norm(pairCenter - refCenter)^2) / (2 * refSigma^2));
                else
                    refTerm = 1;
                end

                score = powerTerm * distTerm * refTerm;

                if score > bestScore
                    bestScore = score;
                    bestPair = [i, j];
                end
            end
        end
    end

    if ~isempty(bestPair)
        selIdx = bestPair;
        selPts = pc_coords(selIdx, :);
        selPw = pc_pw(selIdx);
        selVr = pc_Vr(selIdx);
        bodyCenter = weightedMidpoint(selPts, selPw);
        bodyVelocity = median(selVr);
    else
        if hasRef
            dRef = sqrt(sum((pc_coords - refCenter).^2, 2));
            [~, idxBest] = min(dRef);
        else
            [~, idxBest] = max(pc_pw);
        end

        selIdx = idxBest;
        selPts = pc_coords(selIdx, :);
        selPw = pc_pw(selIdx);
        selVr = pc_Vr(selIdx);

        if hasRef
            bodyCenter = singleBlend * selPts + (1 - singleBlend) * refCenter;
        else
            bodyCenter = selPts;
        end
        bodyVelocity = selVr(1);
    end

    noiseIdx = true(nPts, 1);
    noiseIdx(selIdx) = false;

    clusterRslt.cluster = struct('pc', selPts, 'centroid', bodyCenter, 'velocity', bodyVelocity, 'ghostLabel', 0);
    clusterRslt.noise.pc = pc_coords(noiseIdx, :);
    clusterRslt.pcInput = selPts;
    clusterRslt.pw = selPw(:);
    clusterRslt.clusterIdx = ones(size(selPts, 1), 1);
end

function refCenter = getNearestReferenceCenter(trackConfirm, trackCand, anchorPt)
    refCenters = [];

    if ~isempty(trackConfirm)
        for i = 1:numel(trackConfirm)
            if isfield(trackConfirm(i), 'centroid') && ~isempty(trackConfirm(i).centroid)
                c = trackConfirm(i).centroid;
                if numel(c) == 2 && all(isfinite(c))
                    refCenters = [refCenters; c(:)']; %#ok<AGROW>
                end
            end
        end
    end

    if isempty(refCenters) && ~isempty(trackCand)
        for i = 1:numel(trackCand)
            if isfield(trackCand(i), 'centroid') && ~isempty(trackCand(i).centroid)
                c = trackCand(i).centroid;
                if numel(c) == 2 && all(isfinite(c))
                    refCenters = [refCenters; c(:)']; %#ok<AGROW>
                end
            end
        end
    end

    if isempty(refCenters)
        refCenter = [];
        return;
    end

    d = sqrt(sum((refCenters - anchorPt).^2, 2));
    [~, idx] = min(d);
    refCenter = refCenters(idx, :);
end

function midPt = weightedMidpoint(pts, pw)
    w = sqrt(max(pw(:), 0) + eps);
    if sum(w) <= 0 || any(~isfinite(w))
        midPt = mean(pts, 1);
    else
        midPt = sum(pts .* w, 1) / sum(w);
    end
end