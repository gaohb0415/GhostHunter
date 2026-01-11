% mmWaveMatlab主函数 - 速度补偿对比版 (纯热力图对比)
% 功能: 10秒动态鬼探头 + 物理层干扰对消 (仅对比成像效果)
% ---------------------------------------------------------
%% 1. 初始化与配置
close all; clear; clc; 
addpath(genpath(pwd));

% --- 动力学参数 ---
EGO_VELOCITY = 0.363;        % 雷达车速度 (m/s)
RADAR_YAW    = -30;          % 雷达安装偏角 (度)
TOTAL_FRAMES = 210; 
FRAME_PERIOD = 50e-3;

% --- 扫描参数 ---
CFG_LIMIT_ANG = [-90, 90]; 
CFG_RES_ANG   = 0.5; 
CFG_LIMIT_R   = [];

% --- 载入配置 ---
config2243; 
try load('config.mat', 'resR', 'resV', 'lambda', 'tChirpIntvl'); 
catch, load('.\config\config\config.mat', 'resR', 'resV', 'lambda', 'tChirpIntvl'); end

% --- 网格构建 ---
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);

% =====================【实验真值数据】=====================
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
ped_path_x = [3.44, 3.44, 1.11];
ped_path_y = [5.8, 8.62, 8.62];
ped_init_mat = [ped_path_x', ped_path_y'];

%% 2. 预创建绘图对象 (1x2 布局)
hFig = figure('Name', 'Compensation Effect Analysis (Heatmap Only)', 'Position', [50, 100, 1400, 600]);

% === 子图1: 无补偿 RELAX + Capon ===
ax1 = subplot(1, 2, 1); hold on; axis equal; grid on;
h_pcolor1 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor1, 'EdgeColor', 'none'); shading interp; colormap(ax1, 'jet');
h_car_ov1 = plot(nan, nan, 'k--', 'LineWidth', 1); % 车框
h_ped_ov1 = plot(nan, nan, 'w:', 'LineWidth', 2);  % 行人真值
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str1 = title('1. 无补偿 (Baseline) -> RELAX -> Capon');

% === 子图2: 有补偿 RELAX + Capon ===
ax2 = subplot(1, 2, 2); hold on; axis equal; grid on;
h_pcolor2 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor2, 'EdgeColor', 'none'); shading interp; colormap(ax2, 'jet');
h_car_ov2 = plot(nan, nan, 'k--', 'LineWidth', 1); % 车框
h_ped_ov2 = plot(nan, nan, 'w:', 'LineWidth', 2);  % 行人真值
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str2 = title('2. 有速度补偿 (Compensated) -> RELAX -> Capon');

%% 3. 循环
for iFrm = 1 : 5 : 210  % 保持你之前的测试步长
    
    current_time = (iFrm - 1) * FRAME_PERIOD;
    
    % --- 读取数据 ---
    try radarData_Raw = readBin(iFrm, 0); catch, break; end
    
    % =========================================================
    % 核心处理环节: 构建两条链路
    % =========================================================
    
    % --- 链路 A: Baseline (无补偿) ---
    radarData_Base = radarData_Raw;
    
    % --- 链路 B: Compensated (速度补偿) ---
    % 补偿因子原理: 消除 TDM MIMO 带来的多普勒相位偏移
    % 假设车是静止的，雷达以 EGO_VELOCITY 靠近，相对速度 v_rel = EGO_VELOCITY
    radarData_Comp = radarData_Raw;
    [~, ~, ~, nTx] = size(radarData_Raw);
    for iTx = 1 : nTx
        % 计算该 Tx 发射时的延时 (相对于 Tx1)
        dt = (iTx - 1) * tChirpIntvl; 
        % 生成补偿因子 (对 Tx 维度进行校正)
        comp_factor = exp(1j * 4 * pi * EGO_VELOCITY * dt / lambda);
        radarData_Comp(:, :, :, iTx) = radarData_Comp(:, :, :, iTx) * comp_factor;
    end
    
    % --- 1. 分别做 FFT ---
    [fftBase, ~] = fftRange(radarData_Base, 'pcEn', 0, 'drawEn', 0); 
    [fftComp, ~] = fftRange(radarData_Comp, 'pcEn', 0, 'drawEn', 0); 

    % --- 2. 坐标计算 & Mask生成 ---
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ped_pts_radar = transform_world_to_radar(ped_init_mat, EGO_VELOCITY, RADAR_YAW, current_time);
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
    
    % --- 3. 分别执行 RELAX 物理层对消 ---
    fftBase_Clean = run_relax_cancellation(fftBase, car_pts, full_grid);
    fftComp_Clean = run_relax_cancellation(fftComp, car_pts, full_grid);
    
    % --- 4. Capon 成像 (都只看 ROI) ---
    procMask = ShadowMask; 
    
    % 对无补偿数据成像
    [pwRA_Base, ~] = dbfProc1D(fftBase_Clean, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', procMask, 'pcEn', 0, 'drawEn', 0);
        
    % 对有补偿数据成像
    [pwRA_Comp, ~] = dbfProc1D(fftComp_Clean, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', procMask, 'pcEn', 0, 'drawEn', 0);

    % =========================================================
    % 绘图更新
    % =========================================================
    if ~ishandle(hFig), break; end 
    
    % --- 通用几何数据 ---
    car_x = [car_pts.all_x; car_pts.all_x(1)];
    car_y = [car_pts.all_y; car_pts.all_y(1)];
    ped_x = ped_pts_radar(:,1);
    ped_y = ped_pts_radar(:,2);
    
    % --- 图1: 无补偿热力图 ---
    set(h_pcolor1, 'CData', pwRA_Base);
    set(h_car_ov1, 'XData', car_x, 'YData', car_y);
    set(h_ped_ov1, 'XData', ped_x, 'YData', ped_y);
    delete(findobj(ax1, 'Type', 'contour'));
    contour(ax1, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    set(title_str1, 'String', ['1. 无补偿 (Baseline) Frame: ' num2str(iFrm)]);

    % --- 图2: 有补偿热力图 ---
    set(h_pcolor2, 'CData', pwRA_Comp);
    set(h_car_ov2, 'XData', car_x, 'YData', car_y);
    set(h_ped_ov2, 'XData', ped_x, 'YData', ped_y);
    delete(findobj(ax2, 'Type', 'contour'));
    contour(ax2, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    set(title_str2, 'String', ['2. 有速度补偿 (Compensated) Frame: ' num2str(iFrm)]);

    drawnow limitrate; 
end

%% ================= 辅助函数1: RELAX 封装 =================
function fftClean = run_relax_cancellation(fftData, car_pts, grid_cfg)
    fftClean = fftData;
    
    % 1. 确定范围
    car_center_rho = (car_pts.A.rho + car_pts.K.rho) / 2;
    dist_mask = abs(grid_cfg.range - car_center_rho) < 2.5; 
    car_range_indices = find(dist_mask);
    
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);
    min_car_ang = min(all_angles) - 2; 
    max_car_ang = max(all_angles) + 2;
    
    % 2. 逐距离门处理
    for r_idx = car_range_indices'
        raw_slice = squeeze(fftData(r_idx, :, :, :));
        
        if ndims(raw_slice) == 3 
             [nCh, nRx, nTx] = size(raw_slice);
             raw_slice = reshape(raw_slice, nCh, nRx*nTx);
        end
        snapshot = raw_slice.'; 
        [M, ~] = size(snapshot);
        
        % RELAX 参数
        K_Comps = 6; 
        rel_angles = zeros(1, K_Comps);
        rel_alphas = zeros(1, K_Comps); 
        residual = snapshot;
        
        search_ang = min_car_ang : 0.5 : max_car_ang; 
        A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang)); 
        
        % Step A: 初始化搜索
        for k = 1:K_Comps
            spec = sum(abs(A_scan' * residual), 2);
            [~, idx] = max(spec);
            rel_angles(k) = search_ang(idx);
            a_k = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            rel_alphas(k) = (a_k' * residual(:,1)) / (a_k' * a_k); 
            residual = residual - rel_alphas(k) * a_k;
        end
        
        % Step B: 迭代优化
        MAX_RELAX_ITER = 5;
        for iter = 1 : MAX_RELAX_ITER
            for k = 1 : K_Comps
                data_k = snapshot; 
                for other = 1 : K_Comps
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
        
        % Step C: 重构干扰并对消
        total_interference = zeros(size(snapshot));
        for k = 1 : K_Comps
            a_final = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            total_interference = total_interference + rel_alphas(k) * a_final;
        end
        snapshot_clean = snapshot - total_interference;
        
        % 归位
        try
            fftClean(r_idx, :, :) = snapshot_clean.'; 
        catch
        end
    end
end

%% ================= 辅助函数2: 通用坐标变换 =================
function pts_radar = transform_world_to_radar(pts_world, v, yaw, t)
    dy = v * t;
    X_trans = pts_world(:, 1);
    Y_trans = pts_world(:, 2) - dy;
    theta = -yaw;
    X_radar = X_trans * cosd(theta) - Y_trans * sind(theta);
    Y_radar = X_trans * sind(theta) + Y_trans * cosd(theta);
    pts_radar = [X_radar, Y_radar];
end