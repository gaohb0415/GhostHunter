% =========================================================================
% 动态 2x3 BBox 鲁棒性对照实验
%
% 每一帧实时显示 2×3:
%   上排:  Reference heatmap | Perturbed heatmap | Robust heatmap
%   下排:  Reference tracking | Perturbed tracking | Robust tracking
%
% 三条链:
%   1) Reference:
%      car_pts_base(iFrm) -> 原始完整链路
%
%   2) Perturbed without refinement:
%      car_pts_base(iFrm) + 同一组小平移扰动 -> 原始完整链路
%
%   3) Perturbed with refinement:
%      car_pts_base(iFrm) + 与第二组完全相同的扰动
%      -> robust_blocker_rectangle_refine(...)
%      -> 原始完整链路
%
% 注意:
%   - 本脚本复用你工程里已有函数文件
%   - 三条链各自维护独立 tracking 状态
%   - 显示层按 frame loop 动态刷新，而不是最后只画一帧
% =========================================================================
close all; clearvars; clear global; clc;

addpath(genpath(pwd));
addpath(genpath('my_tracking'));

%% ================== 全局配置 ==================
cfg = struct();

cfg.START_FRAME   = 180;
cfg.TOTAL_FRAMES  = 207;

cfg.PARAM_K_MAX   = 6;
cfg.IMPROVE_TH    = 0.01;
cfg.BLOCK_SIZE    = 32;

cfg.EGO_VELOCITY  = 0.363;
cfg.RADAR_YAW     = -30;
cfg.FRAME_PERIOD  = 50e-3;

cfg.CFG_LIMIT_ANG = [-90, 90];
cfg.CFG_RES_ANG   = 0.5;

cfg.XLIM = [-11, 11];
cfg.YLIM = [0, 12];

cfg.SOFT_THRESHOLD_DB = -15;
cfg.PEAK_GATE_R = 0.8;
cfg.PEAK_GATE_A = 6.0;

cfg.EXCLUDE_MARGIN = 0.25;

cfg.USE_NMS = 0;
cfg.NMS_MERGE_R = 0.40;
cfg.NMS_MERGE_A = 2.5;

cfg.FOOT_PAIR_DIST_MIN   = 0.15;
cfg.FOOT_PAIR_DIST_MAX   = 0.90;
cfg.FOOT_PAIR_DIST_PREF  = 0.45;
cfg.FOOT_PAIR_DIST_SIGMA = 0.18;
cfg.BODY_REF_SIGMA       = 0.60;
cfg.BODY_SINGLE_BLEND    = 0.65;

cfg.ROI_ARC_RADIUS = 10.8;
cfg.HEATMAP_DYNAMIC_RANGE_DB = 20;

% ===== 基准矩形：保持与你当前主函数一致 =====
cfg.ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
cfg.ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
cfg.car_init_mat = [cfg.ground_truth_world.car.x', cfg.ground_truth_world.car.y'];

%% ================== 雷达配置 ==================
config2243;
try
    load('config.mat', 'resR', 'resV', 'spacingCal', 'cfarParamRA');
catch
    load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal', 'cfarParamRA');
end

cfg.resR = resR;
cfg.resV = resV;
cfg.spacingCal = spacingCal;
cfg.cfarParamRA = cfarParamRA;

cfg.ROI_MARGIN_R = 1;
cfg.ROI_MARGIN_A = 1;
cfg.PAD_R = cfarParamRA.train(1) + cfarParamRA.guard(1) + cfg.ROI_MARGIN_R;
cfg.PAD_A = cfarParamRA.train(2) + cfarParamRA.guard(2) + cfg.ROI_MARGIN_A;

cfg.nAdc = 256;
cfg.full_grid.range = resR * (0:cfg.nAdc-1)';
cfg.full_grid.angle = cfg.CFG_LIMIT_ANG(1):cfg.CFG_RES_ANG:cfg.CFG_LIMIT_ANG(2);

[cfg.Ang_Grid, cfg.Rng_Grid] = meshgrid(cfg.full_grid.angle, cfg.full_grid.range);
cfg.X_Plot = cfg.Rng_Grid .* sind(cfg.Ang_Grid);
cfg.Y_Plot = cfg.Rng_Grid .* cosd(cfg.Ang_Grid);

%% ================== 最简单扰动：随机小平移 ==================
perturb = struct();
perturb.seed = 20260419;   % 保持固定，保证第二组和第三组共用同一组扰动

perturb.range_x   = 0.22;  % m
perturb.range_y   = 0.08;  % m
perturb.range_yaw = 2.5;   % deg

[perturb.dx, perturb.dy, perturb.dyaw] = generate_simple_rigid_perturbation(cfg, perturb);

%% ================== 鲁棒模块参数 ==================
robustParam = struct();
robustParam.enable = true;

robustParam.minRawWeight = 0.35;
robustParam.maxRawWeight = 0.80;

robustParam.dxGrid   = -0.30:0.15:0.30;
robustParam.dyGrid   = -0.10:0.10:0.10;
robustParam.dyawGrid = -4:2:4;

robustParam.sigmaObs   = struct('x', 0.30, 'y', 0.20, 'yaw', 4.5, 'l', 0.25, 'w', 0.18);
robustParam.sigmaLidar = struct('x', 0.25, 'y', 0.20, 'yaw', 4.0, 'l', 0.25, 'w', 0.18);
robustParam.sigmaTemp  = struct('x', 0.25, 'y', 0.18, 'yaw', 3.0, 'l', 0.20, 'w', 0.15);

robustParam.radarRangeMargin = 0.80;
robustParam.radarAngleMargin = 8.0;
robustParam.radarAngleStep = 0.5;
robustParam.radarOuterRangePad = 0.35;
robustParam.radarOuterAnglePad = 3.0;
robustParam.radarContrastBeta = 0.35;

robustParam.wRadarBase = 0.50;
robustParam.wLidarBase = 0.30;
robustParam.wTempBase  = 0.20;

robustParam.finalBlendMin = 0.65;
robustParam.finalBlendMax = 0.90;

robustParam.sizeBlendAlpha = 0.25;
robustParam.minLength = 2.5;
robustParam.maxLength = 6.5;
robustParam.minWidth  = 1.2;
robustParam.maxWidth  = 2.8;

%% ================== 初始化三条链的独立状态 ==================
baseChainState = init_chain_state(cfg);

stateRef  = baseChainState;
statePert = baseChainState;
stateRob  = baseChainState;
stateRob.robustState = [];

%% ================== 创建动态 2x3 图 ==================
hFig = figure('Name', 'Dynamic 2x3 BBox Robustness Comparison', ...
    'Position', [60, 40, 1700, 900]);

axs = gobjects(6, 1);
for k = 1:6
    axs(k) = subplot(2, 3, k);
end

fprintf('================ 动态三链对照开始 ================\n');

%% ================== 外层：按帧同步三链 ==================
for frameIdx = cfg.START_FRAME:cfg.TOTAL_FRAMES
    if ~ishandle(hFig)
        break;
    end

    fprintf('Processing Frame %d / %d ...\n', frameIdx, cfg.TOTAL_FRAMES);

    % ---------- 同一帧只读一次雷达数据 ----------
    try
        radarData = readBin(frameIdx, 0);
    catch
        warning('readBin 在 frame=%d 失败，实验终止。', frameIdx);
        break;
    end

    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);
    antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');
    sorted_data = antArray_Sorted.signal;

    % ---------- 三条链分别处理同一帧 ----------
    [stateRef,  frameRef]  = process_one_frame(cfg, stateRef,  frameIdx, sorted_data, antArray_Sorted, perturb, 'reference', robustParam);
    [statePert, framePert] = process_one_frame(cfg, statePert, frameIdx, sorted_data, antArray_Sorted, perturb, 'perturbed', robustParam);
    [stateRob,  frameRob]  = process_one_frame(cfg, stateRob,  frameIdx, sorted_data, antArray_Sorted, perturb, 'robust', robustParam);

    % ---------- 更新 2x3 动态显示 ----------
    update_display_2x3(axs, cfg, frameRef, framePert, frameRob);

    drawnow;
end

fprintf('================ 动态三链对照完成 ================\n');

%% =========================================================================
%% 单帧处理：三条链复用同一后端逻辑
%% =========================================================================
function [chainState, frameVis] = process_one_frame(cfg, chainState, frameIdx, sorted_data, antArray_Sorted, perturb, modeName, robustParam)

    % 1) 恢复该链自己的 tracking / param / global 状态
    restore_chain_state(chainState);

    global p clusters iFrm trackCand trackConfirm

    iFrm = frameIdx;
    robustState = chainState.robustState;

    % ===== 车框输入 =====
    car_pts_base = get_car_dynamic_coords(frameIdx, cfg.car_init_mat, ...
        cfg.EGO_VELOCITY, cfg.RADAR_YAW, cfg.FRAME_PERIOD);

    switch lower(modeName)
        case 'reference'
            car_pts = car_pts_base;

        case 'perturbed'
            car_pts = apply_rigid_perturbation( ...
                car_pts_base, perturb.dx(frameIdx), perturb.dy(frameIdx), perturb.dyaw(frameIdx));

        case 'robust'
            car_pts_raw = apply_rigid_perturbation( ...
                car_pts_base, perturb.dx(frameIdx), perturb.dy(frameIdx), perturb.dyaw(frameIdx));
            [car_pts, robustState] = robust_blocker_rectangle_refine( ...
                car_pts_raw, sorted_data, cfg.full_grid, cfg.spacingCal, robustState, robustParam);

        otherwise
            error('未知 modeName: %s', modeName);
    end

    % ===== ROI =====
    ShadowMask = generate_universal_shadow_mask(cfg.full_grid, car_pts);
    ROI_target_mask = (ShadowMask == 1);

    pwRA_Clean = zeros(cfg.nAdc, length(cfg.full_grid.angle));
    pc_coords = [];

    if ~any(ROI_target_mask(:))
        clusterRslt = makeEmptyClusterResult();
        clusters(frameIdx) = clusterRslt;

        frameVis = collect_frame_visual(frameIdx, pwRA_Clean, ROI_target_mask, car_pts, ...
            pc_coords, clusters(frameIdx), trackConfirm, trackCand);

        chainState = snapshot_chain_state(robustState);
        return;
    end

    roiInfo = build_local_roi_info(ROI_target_mask, size(ShadowMask), cfg.PAD_R, cfg.PAD_A);

    % ===== RELAX 车体抑制 =====
    all_corner_rhos = sqrt(car_pts.all_x(:).^2 + car_pts.all_y(:).^2);
    car_range_indices = find((cfg.full_grid.range > min(all_corner_rhos) - 0.5) & ...
                             (cfg.full_grid.range < max(all_corner_rhos) + 0.5));

    all_angles = atan2d(car_pts.all_x(:), car_pts.all_y(:));
    search_ang_global = (min(all_angles) - 5):0.5:(max(all_angles) + 5);

    sorted_data_clean = sorted_data;
    for r_idx = car_range_indices'
        snapshot_full = squeeze(sorted_data(:, :, r_idx));
        [M, N_Total] = size(snapshot_full);
        interference_full = zeros(M, N_Total);

        num_blocks = ceil(N_Total / cfg.BLOCK_SIZE);
        for b = 1:num_blocks
            idx_start = (b-1) * cfg.BLOCK_SIZE + 1;
            idx_end   = min(b * cfg.BLOCK_SIZE, N_Total);
            current_indices = idx_start:idx_end;
            snapshot_batch = snapshot_full(:, current_indices);

            [~, alphas, angles] = run_relax_core_batch_v2(snapshot_batch, ...
                cfg.PARAM_K_MAX, cfg.IMPROVE_TH, search_ang_global, M, cfg.spacingCal, 0);

            if ~isempty(alphas)
                interference_full(:, current_indices) = reconstruct_signal_batch_v2( ...
                    alphas, angles, M, length(current_indices), cfg.spacingCal);
            end
        end
        sorted_data_clean(:, :, r_idx) = snapshot_full - interference_full;
    end

    % ===== 局部 Capon =====
    local_range_idx = roiInfo.calc_rows(1):roiInfo.calc_rows(2);
    local_angle_idx = roiInfo.calc_cols(1):roiInfo.calc_cols(2);
    local_angles = cfg.full_grid.angle(local_angle_idx);

    for iRg = local_range_idx
        [pw_subset, ~] = dbf(local_angles', [], sorted_data_clean(:, :, iRg), ...
            antArray_Sorted.arrayPos, [], 'spacingCal', cfg.spacingCal, 'pwAlgo', 'Capon');
        pwRA_Clean(iRg, local_angle_idx) = pw_subset(:).';
    end

    % ===== ROI增强 + 局部CFAR =====
    pwRA_Proc = pwRA_Clean;

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

    local_patch_proc = pwRA_Proc(local_range_idx, local_angle_idx);
    [cfar_r_local, cfar_a_local, ~] = cfar2D(local_patch_proc, cfg.cfarParamRA);

    cfar_iRange = cfar_r_local + roiInfo.calc_rows(1) - 1;
    cfar_iAngle = cfar_a_local + roiInfo.calc_cols(1) - 1;

    if ~isempty(cfar_iRange)
        keep_target = ROI_target_mask(sub2ind(size(ROI_target_mask), cfar_iRange, cfar_iAngle));
        cfar_iRange = cfar_iRange(keep_target);
        cfar_iAngle = cfar_iAngle(keep_target);
    end

    % ===== 主峰约束 =====
    if ~isempty(cfar_iRange)
        roi_map = pwRA_Proc;
        roi_map(~ROI_target_mask) = 0;
        roi_vals = roi_map(ROI_target_mask);

        if isempty(roi_vals) || max(roi_vals(:)) <= 0
            clusterRslt = makeEmptyClusterResult();
        else
            [roi_peak, peak_lin_idx] = max(roi_map(:));
            [peak_r_idx, peak_a_idx] = ind2sub(size(roi_map), peak_lin_idx);
            peak_R = cfg.full_grid.range(peak_r_idx);
            peak_A = cfg.full_grid.angle(peak_a_idx);

            roi_threshold = roi_peak * 10^(cfg.SOFT_THRESHOLD_DB / 10);

            valid_mask_idx = false(length(cfar_iRange), 1);
            for k = 1:length(cfar_iRange)
                r_idx = cfar_iRange(k);
                a_idx = cfar_iAngle(k);
                curr_R = cfg.full_grid.range(r_idx);
                curr_A = cfg.full_grid.angle(a_idx);

                inROI = ROI_target_mask(r_idx, a_idx);
                inPeakGate = abs(curr_R - peak_R) <= cfg.PEAK_GATE_R && ...
                             abs(curr_A - peak_A) <= cfg.PEAK_GATE_A;
                highEnough = pwRA_Proc(r_idx, a_idx) >= roi_threshold;

                if inROI && inPeakGate && highEnough
                    valid_mask_idx(k) = true;
                end
            end

            cfar_iRange = cfar_iRange(valid_mask_idx);
            cfar_iAngle = cfar_iAngle(valid_mask_idx);

            if cfg.USE_NMS
                [cfar_iRange, cfar_iAngle] = suppressNearbyDetectionsRA( ...
                    cfar_iRange, cfar_iAngle, pwRA_Proc, cfg.full_grid, ...
                    cfg.NMS_MERGE_R, cfg.NMS_MERGE_A);
            end

            % ===== 点级坐标 / 贴车剔除 / 逐点测速 =====
            if ~isempty(cfar_iRange)
                pc_R = cfg.full_grid.range(cfar_iRange);
                pc_A = cfg.full_grid.angle(cfar_iAngle)';
                pc_X = pc_R .* sind(pc_A);
                pc_Y = pc_R .* cosd(pc_A);
                pc_coords = [pc_X, pc_Y];
                pc_pw = pwRA_Proc(sub2ind(size(pwRA_Proc), cfar_iRange, cfar_iAngle));

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
                    if d_min < cfg.EXCLUDE_MARGIN
                        valid_idx(k) = false;
                    end
                end

                pc_coords = pc_coords(valid_idx, :);
                pc_pw = pc_pw(valid_idx);
                pc_R = pc_R(valid_idx);
                pc_A = pc_A(valid_idx);

                if ~isempty(pc_coords)
                    pc_Vr = zeros(size(pc_coords, 1), 1);
                    [M_ant, N_chirp] = size(squeeze(sorted_data_clean(:, :, 1)));
                    vel_axis = cfg.resV * (-N_chirp / 2:N_chirp / 2 - 1)';

                    for k = 1:size(pc_coords, 1)
                        curr_R_idx = find(abs(cfg.full_grid.range - pc_R(k)) < 1e-4, 1);
                        curr_Ang = pc_A(k);

                        clean_snap = squeeze(sorted_data_clean(:, :, curr_R_idx));
                        a_vec = exp(1j * pi * (0:M_ant-1)' * cfg.spacingCal * sind(curr_Ang));
                        signal_slow_time = (a_vec' * clean_snap);
                        signal_slow_time = signal_slow_time .* hanning(N_chirp)';
                        dop_spec = abs(fftshift(fft(signal_slow_time, N_chirp)));
                        [~, max_v_idx] = max(dop_spec);
                        pc_Vr(k) = vel_axis(max_v_idx);
                    end

                    clusterRslt = buildHumanCenterCluster(pc_coords, pc_pw, pc_Vr, ...
                        trackConfirm, trackCand, ...
                        cfg.FOOT_PAIR_DIST_MIN, cfg.FOOT_PAIR_DIST_MAX, ...
                        cfg.FOOT_PAIR_DIST_PREF, cfg.FOOT_PAIR_DIST_SIGMA, ...
                        cfg.BODY_REF_SIGMA, cfg.BODY_SINGLE_BLEND);
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

    % ===== ghost =====
    clusterRslt = ghostInit(clusterRslt);
    clusterRslt = ghostLabeling(clusterRslt);
    clusters(frameIdx) = clusterRslt;

    % ===== tracking =====
    if frameIdx == cfg.START_FRAME
        for iCluster = 1:local_struct_length(clusters(frameIdx).cluster, 'centroid')
            trackCand(iCluster) = struct( ...
                'centroid', clusters(frameIdx).cluster(iCluster).centroid, ...
                'kalmanFilter', createNewKF(clusters(frameIdx).cluster(iCluster).centroid, 'motionType', p.motionType), ...
                'presence', 1, ...
                'ghostLabel', clusters(frameIdx).cluster(iCluster).ghostLabel, ...
                'age', 1, ...
                'trajectory', clusters(frameIdx).cluster(iCluster).centroid, ...
                'frame', frameIdx);
        end
    else
        kfPredict();

        if isfield(clusters(frameIdx).cluster, 'velocity') && ~isempty(clusters(frameIdx).cluster)
            vel_data = vertcat(clusters(frameIdx).cluster.velocity);
        else
            vel_data = zeros(local_struct_length(clusters(frameIdx).cluster, 'centroid'), 1);
        end

        if local_struct_length(clusters(frameIdx).cluster, 'centroid') > 0
            clusterNew1 = struct( ...
                'centroid', vertcat(clusters(frameIdx).cluster.centroid), ...
                'ghostLabel', vertcat(clusters(frameIdx).cluster.ghostLabel), ...
                'velocity', vel_data, ...
                'pc', {{clusters(frameIdx).cluster.pc}'});
        else
            clusterNew1 = struct( ...
                'centroid', zeros(0, 2), ...
                'ghostLabel', zeros(0, 1), ...
                'velocity', zeros(0, 1), ...
                'pc', {{}});
        end

        assocRslt1 = trackAssociation(trackConfirm, vertcat(clusterNew1.centroid), ...
            clusterNew1.ghostLabel, p.costConfirm);
        [clusterNew1, assocRslt1] = confirmZoneProcess(clusterNew1, assocRslt1);

        if ~isempty(assocRslt1.unassignedDetections)
            [clusterNew1, assocRslt1] = reactivateLostTracks(clusterNew1, assocRslt1, cfg.FRAME_PERIOD);
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

            candidateZoneProcess_modified(clusterNew2, assocRslt2);
        else
            renewCandidate(0, []);
        end

        renewTrajectory();
    end

    % ===== 保存本帧可视化内容 =====
    frameVis = collect_frame_visual(frameIdx, pwRA_Clean, ROI_target_mask, car_pts, ...
        pc_coords, clusters(frameIdx), trackConfirm, trackCand);

    % ===== 把本链全局状态重新打包保存 =====
    chainState = snapshot_chain_state(robustState);
end

%% =========================================================================
%% 动态显示更新
%% =========================================================================
function update_display_2x3(axs, cfg, frameRef, framePert, frameRob)
    frames = {frameRef, framePert, frameRob};
    topTitles = {'Reference heatmap', 'Perturbed heatmap', 'Robust heatmap'};
    botTitles = {'Reference tracking', 'Perturbed tracking', 'Robust tracking'};

    % 统一本帧上排热力图色轴
    roiMaxAll = 0;
    for k = 1:3
        vals = frames{k}.disp_pw(frames{k}.disp_pw > 0);
        if ~isempty(vals)
            roiMaxAll = max(roiMaxAll, max(vals(:)));
        end
    end
    if roiMaxAll <= 0
        roiMaxAll = 1;
    end
    cmin = roiMaxAll * 10^(-cfg.HEATMAP_DYNAMIC_RANGE_DB / 10);
    cmax = roiMaxAll;

    % ---------- 上排 ----------
    for k = 1:3
        ax = axs(k);
        cla(ax);
        hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
        xlim(ax, cfg.XLIM); ylim(ax, cfg.YLIM);
        xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
        title(ax, sprintf('%s (Frame %d)', topTitles{k}, frames{k}.frame), 'FontWeight', 'bold');

        hp = pcolor(ax, cfg.X_Plot, cfg.Y_Plot, frames{k}.disp_pw);
        set(hp, 'EdgeColor', 'none');
        shading(ax, 'interp');
        colormap(ax, 'jet');
        caxis(ax, [cmin, cmax]);

        draw_roi_boundary_with_arc(ax, frames{k}.car_pts, cfg.ROI_ARC_RADIUS, 'w--', 1.4);
    end

    % ---------- 下排 ----------
    for k = 1:3
        ax = axs(k + 3);
        cla(ax);
        hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
        xlim(ax, cfg.XLIM); ylim(ax, cfg.YLIM);
        xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
        title(ax, sprintf('%s (Frame %d)', botTitles{k}, frames{k}.frame), 'FontWeight', 'bold');

        draw_roi_boundary_with_arc(ax, frames{k}.car_pts, cfg.ROI_ARC_RADIUS, 'k--', 1.4);

        if ~isempty(frames{k}.pc_coords)
            scatter(ax, frames{k}.pc_coords(:,1), frames{k}.pc_coords(:,2), ...
                16, [0.70 0.70 0.70], 'filled');
        end

        if local_struct_length(frames{k}.clustersFrame.cluster, 'centroid') > 0
            all_centroids = vertcat(frames{k}.clustersFrame.cluster.centroid);
            ghost_flags = vertcat(frames{k}.clustersFrame.cluster.ghostLabel);
            if any(ghost_flags)
                ghost_pts = all_centroids(ghost_flags == 1, :);
                scatter(ax, ghost_pts(:,1), ghost_pts(:,2), 80, 'c', 'x', 'LineWidth', 2);
            end
        end

        for iT = 1:local_struct_length(frames{k}.trackConfirm, 'centroid')
            traj_pts = frames{k}.trackConfirm(iT).trajectory;
            if ~isempty(traj_pts)
                plot(ax, traj_pts(:,1), traj_pts(:,2), 'r-', 'LineWidth', 2);
            end
            if ~isempty(frames{k}.trackConfirm(iT).centroid)
                scatter(ax, frames{k}.trackConfirm(iT).centroid(1), ...
                    frames{k}.trackConfirm(iT).centroid(2), 100, 'r', 'p', 'filled');
            end
        end

        for iC = 1:local_struct_length(frames{k}.trackCand, 'centroid')
            traj_pts = frames{k}.trackCand(iC).trajectory;
            if ~isempty(traj_pts)
                plot(ax, traj_pts(:,1), traj_pts(:,2), 'y--', 'LineWidth', 1);
            end
            if ~isempty(frames{k}.trackCand(iC).centroid)
                scatter(ax, frames{k}.trackCand(iC).centroid(1), ...
                    frames{k}.trackCand(iC).centroid(2), 50, 'y', 'o', 'filled');
            end
        end
    end
end

%% =========================================================================
%% 三条链状态隔离：snapshot / restore
%% =========================================================================
function chainState = init_chain_state(cfg)
    global p clusters iFrm trackCand trackConfirm trackLost trackWait ...
           trajectory ovlpRec sepRec multivRec assocRec

    p = trackParamConfig(0);

    % 和你当前主函数对齐的“纯净 tracking 模式”
    p.nFrmLoad = cfg.TOTAL_FRAMES;
    p.iFrmLoad = 1:cfg.TOTAL_FRAMES;

    p.multiverseEn = 0;
    p.ovlpProcEn   = 0;
    p.identifyEn   = 0;
    p.staticEnhEn  = 0;
    p.waitZoneEn   = 0;
    p.backtrackEn  = 0;

    p.trackAlgo = 'KF';
    p.motionType = 'ConstantVelocity';
    p.candWin = 6;
    p.presRatioNew = 0.5;
    p.nFrmNotGhost = 2;
    p.costCand = 0.45;
    p.costConfirm = 0.45;
    p.smthWin = 5;
    p.nSmthNewConfirm = 1;
    p.nFrmLost = 12;

    trackCand = struct('centroid', [], 'kalmanFilter', [], 'presence', [], ...
        'ghostLabel', [], 'age', [], 'trajectory', [], 'frame', []);
    trackConfirm = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], ...
        'name', [], 'pc', [], 'status', [], 'statusAge', [], 'trajectory', [], 'frame', []);
    trackLost = struct('centroid', [], 'iPeople', [], 'name', [], 'trajectory', [], 'frame', []);
    trackWait = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], 'name', [], 'age', []);
    assocRec = struct('frame', 1, 'association', 1:10);

    for f = 1:cfg.TOTAL_FRAMES
        trajectory(f).track = struct('iPeople', [], 'name', [], 'trajectory', [], ...
            'frame', [], 'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []);
        ovlpRec(f).ovlp = struct('idxSet', []);
        sepRec(f).sep = struct('idxSet', [], 'nSeperate', []);
        multivRec(f).multiv = struct('iMultiverse', 1, ...
            'track', struct('iPeople', [], 'name', [], 'trajectory', [], ...
            'frame', [], 'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []), ...
            'brother', [], 'parent', [], 'association', []);
    end

    p.nPpl = 0;
    p.backtrackFlag = 0;
    p.sepDelFlag = 0;

    clusters = repmat(makeEmptyClusterResult(), cfg.TOTAL_FRAMES, 1);
    iFrm = 0;

    chainState = snapshot_chain_state([]);
end

function chainState = snapshot_chain_state(robustState)
    global p clusters iFrm trackCand trackConfirm trackLost trackWait ...
           trajectory ovlpRec sepRec multivRec assocRec

    chainState = struct();
    chainState.param = export_param_store();
    chainState.clusters = clusters;
    chainState.iFrm = iFrm;
    chainState.trackCand = trackCand;
    chainState.trackConfirm = trackConfirm;
    chainState.trackLost = trackLost;
    chainState.trackWait = trackWait;
    chainState.trajectory = trajectory;
    chainState.ovlpRec = ovlpRec;
    chainState.sepRec = sepRec;
    chainState.multivRec = multivRec;
    chainState.assocRec = assocRec;
    chainState.robustState = robustState;
end

function restore_chain_state(chainState)
    global p clusters iFrm trackCand trackConfirm trackLost trackWait ...
           trajectory ovlpRec sepRec multivRec assocRec

    import_param_store(chainState.param);
    p = trackParamShare.param;

    clusters = chainState.clusters;
    iFrm = chainState.iFrm;
    trackCand = chainState.trackCand;
    trackConfirm = chainState.trackConfirm;
    trackLost = chainState.trackLost;
    trackWait = chainState.trackWait;
    trajectory = chainState.trajectory;
    ovlpRec = chainState.ovlpRec;
    sepRec = chainState.sepRec;
    multivRec = chainState.multivRec;
    assocRec = chainState.assocRec;
end

function s = export_param_store()
    h = trackParamShare.param;
    props = properties(h);
    s = struct();
    for i = 1:numel(props)
        s.(props{i}) = h.(props{i});
    end
end

function import_param_store(s)
    h = trackParamShare.param;
    props = fieldnames(s);
    for i = 1:numel(props)
        h.(props{i}) = s.(props{i});
    end
end

%% =========================================================================
%% 扰动 / frame visual / car_pts 辅助
%% =========================================================================
function [dx, dy, dyaw] = generate_simple_rigid_perturbation(cfg, perturb)
    rng(perturb.seed);

    dx   = zeros(cfg.TOTAL_FRAMES, 1);
    dy   = zeros(cfg.TOTAL_FRAMES, 1);
    dyaw = zeros(cfg.TOTAL_FRAMES, 1);

    nUse = cfg.TOTAL_FRAMES - cfg.START_FRAME + 1;

    rx = rand(nUse, 1);
    ry = rand(nUse, 1);
    rz = rand(nUse, 1);

    dx_use   = (2 * rx - 1) * perturb.range_x;
    dy_use   = (2 * ry - 1) * perturb.range_y;
    dyaw_use = (2 * rz - 1) * perturb.range_yaw;

    dx(cfg.START_FRAME:cfg.TOTAL_FRAMES)   = dx_use;
    dy(cfg.START_FRAME:cfg.TOTAL_FRAMES)   = dy_use;
    dyaw(cfg.START_FRAME:cfg.TOTAL_FRAMES) = dyaw_use;
end

function car_pts_new = apply_rigid_perturbation(car_pts, dx, dy, dyaw)
    % 原始四点
    pts = [car_pts.A.x, car_pts.A.y;
           car_pts.B.x, car_pts.B.y;
           car_pts.K.x, car_pts.K.y;
           car_pts.C.x, car_pts.C.y];

    % 中心
    ctr = mean(pts, 1);

    % 先绕中心旋转，再整体平移
    R = [cosd(dyaw), -sind(dyaw);
         sind(dyaw),  cosd(dyaw)];

    pts_shift = pts - ctr;
    pts_rot = (R * pts_shift')';
    pts_new = pts_rot + ctr + [dx, dy];

    % 回填
    car_pts_new = car_pts;
    names = {'A','B','K','C'};
    for i = 1:4
        nm = names{i};
        car_pts_new.(nm).x = pts_new(i,1);
        car_pts_new.(nm).y = pts_new(i,2);
        car_pts_new.(nm).rho = hypot(pts_new(i,1), pts_new(i,2));
        car_pts_new.(nm).theta = atan2d(pts_new(i,1), pts_new(i,2));
    end

    % 保持你当前代码使用的顺序
    car_pts_new.all_x = [car_pts_new.A.x, car_pts_new.C.x, car_pts_new.K.x, car_pts_new.B.x];
    car_pts_new.all_y = [car_pts_new.A.y, car_pts_new.C.y, car_pts_new.K.y, car_pts_new.B.y];

    if isfield(car_pts_new, 'K_Dynamic')
        [~, idx_far] = max(hypot(car_pts_new.all_x(:), car_pts_new.all_y(:)));
        tmpx = [car_pts_new.A.x, car_pts_new.C.x, car_pts_new.K.x, car_pts_new.B.x];
        tmpy = [car_pts_new.A.y, car_pts_new.C.y, car_pts_new.K.y, car_pts_new.B.y];
        car_pts_new.K_Dynamic.x = tmpx(idx_far);
        car_pts_new.K_Dynamic.y = tmpy(idx_far);
        car_pts_new.K_Dynamic.rho = hypot(tmpx(idx_far), tmpy(idx_far));
        car_pts_new.K_Dynamic.theta = atan2d(tmpx(idx_far), tmpy(idx_far));
    end
end

function frameVis = collect_frame_visual(frameIdx, pwRA_Clean, ROI_target_mask, car_pts, ...
    pc_coords, clustersFrame, trackConfirm, trackCand)

    frameVis = struct();
    frameVis.frame = frameIdx;
    frameVis.pwRA_Clean = pwRA_Clean;
    frameVis.ROI_target_mask = ROI_target_mask;
    frameVis.disp_pw = zeros(size(pwRA_Clean));
    frameVis.disp_pw(ROI_target_mask == 1) = pwRA_Clean(ROI_target_mask == 1);

    frameVis.car_pts = car_pts;
    frameVis.pc_coords = pc_coords;
    frameVis.clustersFrame = clustersFrame;
    frameVis.trackConfirm = trackConfirm;
    frameVis.trackCand = trackCand;
end

%% =========================================================================
%% 当前主函数中复用的局部工具
%% =========================================================================
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

function n = local_struct_length(s, fieldName)
    if isempty(s)
        n = 0;
        return;
    end
    if isstruct(s)
        if numel(s) == 1 && isfield(s, fieldName) && isempty(s.(fieldName))
            n = 0;
        else
            n = numel(s);
        end
    else
        n = 0;
    end
end