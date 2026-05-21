function [car_pts_refined, robustState, meta] = robust_blocker_rectangle_refine( ...
    car_pts_raw, sorted_data, full_grid, spacingCal, robustState, param)
% =========================================================================
% robust_blocker_rectangle_refine
%
% 功能：
%   输入当前帧原始遮挡车矩形 car_pts_raw 和当前帧 sorted_data，
%   输出一个修正后的刚性长方形 car_pts_refined（四顶点仍严格保持矩形）。
%
% 设计原则：
%   1) 不改主链后续接口：输出仍然是 car_pts 结构
%   2) 内部修正对象是刚性矩形状态 (x, y, yaw, l, w)，不是四个角点乱修
%   3) 综合 lidar 原始观测、时序连续性、毫米波强散射一致性来选最优矩形
% =========================================================================

    if nargin < 6 || isempty(param)
        param = get_default_param();
    end

    % 默认输出
    car_pts_refined = car_pts_raw;
    meta = struct('rawState', [], 'predState', [], 'seedState', [], ...
        'refinedState', [], 'scoreInfo', []);

    % 允许直接关闭
    if ~isfield(param, 'enable') || ~param.enable
        rawState = rect_struct_to_state(car_pts_raw);
        robustState = init_or_update_state([], rawState);
        meta.rawState = rawState;
        meta.predState = rawState;
        meta.seedState = rawState;
        meta.refinedState = rawState;
        meta.scoreInfo = [];
        return;
    end

    try
        %% 1) 原始矩形 -> 状态
        rawState = rect_struct_to_state(car_pts_raw);

        %% 2) 时序预测
        if isempty(robustState) || ~isfield(robustState, 'state') || isempty(robustState.state)
            predState = rawState;
            hasHistory = false;
        else
            predState = robustState.state;
            hasHistory = true;
        end

        %% 3) 基于 raw / pred 差异构造 seed state
        [seedState, obsReliability, obsDistance] = build_seed_state(rawState, predState, param, hasHistory);

        %% 4) 基于 raw+pred 的局部雷达一致性上下文
        radarCtx = build_local_radar_context(sorted_data, full_grid, spacingCal, rawState, predState, param);

        %% 5) 枚举小范围刚性矩形候选
        candStates = enumerate_candidate_states(seedState, rawState, predState, param);

        %% 6) 对候选打分：lidar fidelity + temporal consistency + radar consistency
        scoreInfo = score_candidates(candStates, rawState, predState, radarCtx, param, obsReliability);

        [~, bestIdx] = max(scoreInfo.totalScore);
        bestState = candStates(bestIdx);

        %% 7) 对 bestState 做最终时序平滑，得到 refinedState
        refinedState = finalize_refined_state(bestState, rawState, predState, scoreInfo, param, hasHistory);

        %% 8) 回投为严格矩形四顶点
        car_pts_refined = rect_state_to_struct(refinedState);

        %% 9) 更新 state
        robustState = init_or_update_state(robustState, refinedState);
        robustState.obsReliability = obsReliability;
        robustState.obsDistance = obsDistance;
        robustState.bestIdx = bestIdx;

        %% 10) meta
        meta.rawState = rawState;
        meta.predState = predState;
        meta.seedState = seedState;
        meta.refinedState = refinedState;
        meta.scoreInfo = scoreInfo;

    catch
        % 出问题就安全回退到 raw，避免破坏主链
        rawState = rect_struct_to_state(car_pts_raw);
        robustState = init_or_update_state(robustState, rawState);
        car_pts_refined = car_pts_raw;

        meta.rawState = rawState;
        meta.predState = rawState;
        meta.seedState = rawState;
        meta.refinedState = rawState;
        meta.scoreInfo = [];
    end
end

%% ========================= 子函数 =========================
function param = get_default_param()
    param = struct();
    param.enable = true;

    param.minRawWeight = 0.35;
    param.maxRawWeight = 0.80;

    param.dxGrid   = -0.30:0.15:0.30;
    param.dyGrid   = -0.10:0.10:0.10;
    param.dyawGrid = -4:2:4;

    param.sigmaObs  = struct('x', 0.30, 'y', 0.20, 'yaw', 4.5, 'l', 0.25, 'w', 0.18);
    param.sigmaLidar = struct('x', 0.25, 'y', 0.20, 'yaw', 4.0, 'l', 0.25, 'w', 0.18);
    param.sigmaTemp  = struct('x', 0.25, 'y', 0.18, 'yaw', 3.0, 'l', 0.20, 'w', 0.15);

    param.radarRangeMargin = 0.80;
    param.radarAngleMargin = 8.0;
    param.radarAngleStep = 0.5;
    param.radarOuterRangePad = 0.35;
    param.radarOuterAnglePad = 3.0;
    param.radarContrastBeta = 0.35;

    param.wRadarBase = 0.50;
    param.wLidarBase = 0.30;
    param.wTempBase  = 0.20;

    param.finalBlendMin = 0.65;
    param.finalBlendMax = 0.90;

    param.sizeBlendAlpha = 0.25;
    param.minLength = 2.5;
    param.maxLength = 6.5;
    param.minWidth  = 1.2;
    param.maxWidth  = 2.8;
end

function robustState = init_or_update_state(robustState, state)
    if isempty(robustState)
        robustState = struct();
    end
    robustState.state = state;
end

function state = rect_struct_to_state(car_pts)
    pts = [car_pts.A.x, car_pts.A.y;
           car_pts.B.x, car_pts.B.y;
           car_pts.K.x, car_pts.K.y;
           car_pts.C.x, car_pts.C.y];

    center = mean(pts, 1);

    vAB = pts(2,:) - pts(1,:);
    vAC = pts(4,:) - pts(1,:);
    vBK = pts(3,:) - pts(2,:);
    vCK = pts(3,:) - pts(4,:);

    width1 = norm(vAB);
    width2 = norm(vCK);
    len1 = norm(vAC);
    len2 = norm(vBK);

    width = mean([width1, width2]);
    len = mean([len1, len2]);

    % 长轴方向定义为 A -> C（对应 y 正方向为前）
    yaw = atan2d(vAC(1), vAC(2));
    yaw = wrap_angle_deg(yaw);

    state = struct();
    state.x = center(1);
    state.y = center(2);
    state.yaw = yaw;
    state.l = len;
    state.w = width;
end

function car_pts = rect_state_to_struct(state)
    yaw = state.yaw;
    len = state.l;
    wid = state.w;
    center = [state.x, state.y];

    u = [sind(yaw), cosd(yaw)];     % length axis
    v = [cosd(yaw), -sind(yaw)];    % width axis

    A = center - 0.5 * len * u - 0.5 * wid * v;
    B = center - 0.5 * len * u + 0.5 * wid * v;
    K = center + 0.5 * len * u + 0.5 * wid * v;
    C = center + 0.5 * len * u - 0.5 * wid * v;

    car_pts = struct();
    car_pts.A = make_corner_struct(A);
    car_pts.B = make_corner_struct(B);
    car_pts.K = make_corner_struct(K);
    car_pts.C = make_corner_struct(C);

    car_pts.all_x = [A(1), B(1), K(1), C(1)];
    car_pts.all_y = [A(2), B(2), K(2), C(2)];
    car_pts.all_theta = atan2d(car_pts.all_x, car_pts.all_y);

    car_pts.center = struct('x', state.x, 'y', state.y);
    car_pts.length = state.l;
    car_pts.width = state.w;
    car_pts.yaw = state.yaw;
end

function corner = make_corner_struct(pt)
    corner = struct();
    corner.x = pt(1);
    corner.y = pt(2);
    corner.theta = atan2d(pt(1), pt(2));
end

function [seedState, obsReliability, obsDistance] = build_seed_state(rawState, predState, param, hasHistory)
    if ~hasHistory
        seedState = rawState;
        obsReliability = 1.0;
        obsDistance = 0.0;
        return;
    end

    d = normalized_state_distance(rawState, predState, param.sigmaObs);
    obsDistance = d;
    obsReliability = exp(-0.5 * d^2);

    wRaw = param.minRawWeight + (param.maxRawWeight - param.minRawWeight) * obsReliability;

    seedState = struct();
    seedState.x = wRaw * rawState.x + (1 - wRaw) * predState.x;
    seedState.y = wRaw * rawState.y + (1 - wRaw) * predState.y;
    seedState.yaw = blend_angle_deg(predState.yaw, rawState.yaw, wRaw);
    seedState.l = clamp_value(wRaw * rawState.l + (1 - wRaw) * predState.l, param.minLength, param.maxLength);
    seedState.w = clamp_value(wRaw * rawState.w + (1 - wRaw) * predState.w, param.minWidth, param.maxWidth);
end

function radarCtx = build_local_radar_context(sorted_data, full_grid, spacingCal, rawState, predState, param)
    unionStates = [rawState, predState];

    allR = [];
    allA = [];
    for i = 1:numel(unionStates)
        [rSpan, aSpan] = get_state_range_angle_span(unionStates(i));
        allR = [allR, rSpan]; %#ok<AGROW>
        allA = [allA, aSpan]; %#ok<AGROW>
    end

    rMin = max(min(full_grid.range), min(allR) - param.radarRangeMargin);
    rMax = min(max(full_grid.range), max(allR) + param.radarRangeMargin);
    aMin = max(min(full_grid.angle), min(allA) - param.radarAngleMargin);
    aMax = min(max(full_grid.angle), max(allA) + param.radarAngleMargin);

    rangeIdx = find(full_grid.range >= rMin & full_grid.range <= rMax);
    angleGrid = aMin:param.radarAngleStep:aMax;

    if isempty(rangeIdx)
        [~, idx0] = min(abs(full_grid.range - mean(allR)));
        rangeIdx = idx0;
    end
    if isempty(angleGrid)
        angleGrid = mean(allA);
    end

    nR = numel(rangeIdx);
    nA = numel(angleGrid);
    radarMap = zeros(nR, nA);

    for i = 1:nR
        snap = squeeze(sorted_data(:, :, rangeIdx(i)));
        radarMap(i, :) = bartlett_power_1d(snap, angleGrid, spacingCal);
    end

    radarMap = log1p(max(radarMap, 0));

    radarCtx = struct();
    radarCtx.rangeIdx = rangeIdx;
    radarCtx.rangeAxis = full_grid.range(rangeIdx);
    radarCtx.angleAxis = angleGrid;
    radarCtx.map = radarMap;
end

function p = bartlett_power_1d(snapshot, angleGrid, spacingCal)
    [M, ~] = size(snapshot);
    antIdx = (0:M-1).';

    A = exp(1j * pi * antIdx * spacingCal * sind(angleGrid));
    beam = A' * snapshot;          % [nAng, Nsnap]
    p = mean(abs(beam).^2, 2).';   % row vector
end

function candStates = enumerate_candidate_states(seedState, rawState, predState, param)
    [DX, DY, DYaw] = ndgrid(param.dxGrid, param.dyGrid, param.dyawGrid);
    nCand = numel(DX);

    candStates = repmat(seedState, nCand + 2, 1);

    for i = 1:nCand
        candStates(i).x = seedState.x + DX(i);
        candStates(i).y = seedState.y + DY(i);
        candStates(i).yaw = wrap_angle_deg(seedState.yaw + DYaw(i));
        candStates(i).l = seedState.l;
        candStates(i).w = seedState.w;
    end

    % 把 raw / pred 也显式加进去，避免搜索格点遗漏
    candStates(end-1) = rawState;
    candStates(end)   = predState;
end

function scoreInfo = score_candidates(candStates, rawState, predState, radarCtx, param, obsReliability)
    nCand = numel(candStates);

    lidarScore = zeros(nCand, 1);
    tempScore  = zeros(nCand, 1);
    radarScore = zeros(nCand, 1);

    for i = 1:nCand
        lidarDist = normalized_state_distance(candStates(i), rawState, param.sigmaLidar);
        tempDist  = normalized_state_distance(candStates(i), predState, param.sigmaTemp);

        lidarScore(i) = exp(-0.5 * lidarDist^2);
        tempScore(i)  = exp(-0.5 * tempDist^2);
        radarScore(i) = compute_radar_consistency_score(candStates(i), radarCtx, param);
    end

    radarScoreNorm = normalize_to_01(radarScore);
    lidarScoreNorm = normalize_to_01(lidarScore);
    tempScoreNorm  = normalize_to_01(tempScore);

    % 观测越不可靠，越提高 radar / temporal 占比
    wRadar = param.wRadarBase + (1 - obsReliability) * 0.15;
    wLidar = param.wLidarBase - (1 - obsReliability) * 0.08;
    wTemp  = param.wTempBase  - (1 - obsReliability) * 0.07;

    wRadar = max(wRadar, 0.10);
    wLidar = max(wLidar, 0.10);
    wTemp  = max(wTemp, 0.10);

    wSum = wRadar + wLidar + wTemp;
    wRadar = wRadar / wSum;
    wLidar = wLidar / wSum;
    wTemp  = wTemp  / wSum;

    totalScore = wRadar * radarScoreNorm + ...
                 wLidar * lidarScoreNorm + ...
                 wTemp  * tempScoreNorm;

    scoreInfo = struct();
    scoreInfo.lidarScore = lidarScore;
    scoreInfo.tempScore = tempScore;
    scoreInfo.radarScore = radarScore;
    scoreInfo.lidarScoreNorm = lidarScoreNorm;
    scoreInfo.tempScoreNorm = tempScoreNorm;
    scoreInfo.radarScoreNorm = radarScoreNorm;
    scoreInfo.totalScore = totalScore;
    scoreInfo.weights = [wRadar, wLidar, wTemp];
end

function radarScore = compute_radar_consistency_score(state, radarCtx, param)
    [rSpan, aSpan] = get_state_range_angle_span(state);

    innerMaskR = radarCtx.rangeAxis >= min(rSpan) & radarCtx.rangeAxis <= max(rSpan);
    innerMaskA = radarCtx.angleAxis >= min(aSpan) & radarCtx.angleAxis <= max(aSpan);

    outerMaskR = radarCtx.rangeAxis >= (min(rSpan) - param.radarOuterRangePad) & ...
                 radarCtx.rangeAxis <= (max(rSpan) + param.radarOuterRangePad);
    outerMaskA = radarCtx.angleAxis >= (min(aSpan) - param.radarOuterAnglePad) & ...
                 radarCtx.angleAxis <= (max(aSpan) + param.radarOuterAnglePad);

    if ~any(innerMaskR) || ~any(innerMaskA)
        radarScore = -1e6;
        return;
    end

    innerMask = innerMaskR(:) * innerMaskA(:).';
    outerMask = outerMaskR(:) * outerMaskA(:).';
    ringMask = outerMask & ~innerMask;

    innerVals = radarCtx.map(innerMask);
    if isempty(innerVals)
        radarScore = -1e6;
        return;
    end

    innerMean = mean(innerVals);

    if any(ringMask(:))
        outerVals = radarCtx.map(ringMask);
        outerMean = mean(outerVals);
    else
        outerMean = 0;
    end

    % 内部强、外部弱，则得分高
    radarScore = innerMean - param.radarContrastBeta * outerMean;
end

function refinedState = finalize_refined_state(bestState, rawState, predState, scoreInfo, param, hasHistory)
    if ~hasHistory
        refinedState = bestState;
        refinedState.l = clamp_value(rawState.l, param.minLength, param.maxLength);
        refinedState.w = clamp_value(rawState.w, param.minWidth, param.maxWidth);
        return;
    end

    confBest = max(scoreInfo.totalScore);
    alpha = param.finalBlendMin + confBest * (param.finalBlendMax - param.finalBlendMin);
    alpha = clamp_value(alpha, param.finalBlendMin, param.finalBlendMax);

    refinedState = struct();
    refinedState.x = alpha * bestState.x + (1 - alpha) * predState.x;
    refinedState.y = alpha * bestState.y + (1 - alpha) * predState.y;
    refinedState.yaw = blend_angle_deg(predState.yaw, bestState.yaw, alpha);

    % 尺度不做搜索，只做低速平滑更新
    refinedState.l = clamp_value( ...
        (1 - param.sizeBlendAlpha) * predState.l + param.sizeBlendAlpha * rawState.l, ...
        param.minLength, param.maxLength);

    refinedState.w = clamp_value( ...
        (1 - param.sizeBlendAlpha) * predState.w + param.sizeBlendAlpha * rawState.w, ...
        param.minWidth, param.maxWidth);
end

function d = normalized_state_distance(s1, s2, sigma)
    dx = (s1.x - s2.x) / sigma.x;
    dy = (s1.y - s2.y) / sigma.y;
    dyaw = angle_diff_deg(s1.yaw, s2.yaw) / sigma.yaw;
    dl = (s1.l - s2.l) / sigma.l;
    dw = (s1.w - s2.w) / sigma.w;
    d = sqrt(dx.^2 + dy.^2 + dyaw.^2 + dl.^2 + dw.^2);
end

function [rSpan, aSpan] = get_state_range_angle_span(state)
    tmp = rect_state_to_struct(state);
    xs = tmp.all_x(:);
    ys = tmp.all_y(:);
    rhos = sqrt(xs.^2 + ys.^2);
    angs = atan2d(xs, ys);

    rSpan = [min(rhos), max(rhos)];
    aSpan = [min(angs), max(angs)];
end

function x = normalize_to_01(x)
    x = x(:);
    xmin = min(x);
    xmax = max(x);
    if ~isfinite(xmin) || ~isfinite(xmax)
        x = 0.5 * ones(size(x));
        return;
    end
    if abs(xmax - xmin) < 1e-12
        x = 0.5 * ones(size(x));
    else
        x = (x - xmin) / (xmax - xmin);
    end
end

function a = wrap_angle_deg(a)
    a = mod(a + 180, 360) - 180;
end

function d = angle_diff_deg(a, b)
    d = wrap_angle_deg(a - b);
end

function a = blend_angle_deg(a0, a1, w1)
    % 从 a0 朝 a1 插值，权重 w1 属于 a1
    d = angle_diff_deg(a1, a0);
    a = wrap_angle_deg(a0 + w1 * d);
end

function v = clamp_value(v, lb, ub)
    v = max(lb, min(ub, v));
end