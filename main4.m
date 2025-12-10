% mmWaveMatlab主函数 - 最终全功能版 (含干扰对消 + ROI Capon)
% 功能: 10秒动态鬼探头 + 物理层干扰对消 (Signal Purification)
% 场景: 雷达向前运动，右偏15度，车静止，人沿预定轨迹运动
%% 1. 初始化与配置
close all; clear; clc;
addpath(genpath(pwd));
% --- 动力学参数 ---
EGO_VELOCITY = 0.538;        % 雷达车速度 (m/s)
RADAR_YAW    = -30;          % 雷达安装偏角 (度)
TOTAL_FRAMES = 210; 
FRAME_PERIOD = 50e-3;
% --- 扫描参数 ---
CFG_LIMIT_ANG = [-90, 90]; 
CFG_RES_ANG   = 0.5; 
CFG_LIMIT_R   = [];
% --- 载入配置 ---
config2243; 
try load('config.mat', 'resR', 'resV'); 
catch, load('.\config\config\config.mat', 'resR', 'resV'); end
% --- 网格构建 ---
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);

% =====================【实验真值数据】=====================
ground_truth_world.car.x = [ 1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [ 5.38, 5.38, 10.11, 10.11];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
ped_path_x = [0,    4.47,  4.47]; 
ped_path_y = [11.87, 11.87, 9.3];
ped_init_mat = [ped_path_x', ped_path_y'];
% ===============================================================

%% 2. 预创建绘图对象
hFig = figure('Name', 'Ghost Probe Interference Cancellation', 'Position', [50, 50, 1400, 900]);
% === 子图1: 几何模型 ===
ax1 = subplot(2, 2, 1); hold on; axis equal; grid on;
plot(0, 0, 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
h_car_fill = fill(nan, nan, [0.8 0.8 0.8], 'FaceAlpha', 0.5);
h_car_edge = plot(nan, nan, 'k-', 'LineWidth', 1.5);
h_ped_geom = plot(nan, nan, 'g--', 'LineWidth', 2); 
title_str1 = title('1. 几何模型');
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');

% === 子图2: Range-FFT ===
ax2 = subplot(2, 2, 2); hold on; grid on;
h_rfft_line = plot(nan, nan, 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
h_xline_A = xline(nan, 'r--', 'A', 'LineWidth', 1, 'LabelVerticalAlignment', 'bottom');
xlim([0, 10]); ylim([40, 120]); 
xlabel('距离 (m)'); ylabel('幅度 (dB)');
title('2. Range-FFT');

% === 子图3: 原始热力图 (未处理) ===
ax3 = subplot(2, 2, 3); hold on; axis equal; grid on;
h_pcolor1 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor1, 'EdgeColor', 'none'); shading interp; colormap(ax3, 'jet');
h_ped_overlay1 = plot(nan, nan, 'w:', 'LineWidth', 2.5); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title('3. 原始数据 Capon (旁瓣污染严重)');

% === 子图4: RELAX + ROI Capon ===
ax4 = subplot(2, 2, 4); hold on; axis equal; grid on;
h_pcolor2 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor2, 'EdgeColor', 'none'); shading interp; colormap(ax4, 'jet');
h_car_overlay = plot(nan, nan, 'm-', 'LineWidth', 2);
h_ped_overlay2 = plot(nan, nan, 'w:', 'LineWidth', 2.5); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str4 = title('4. 干扰对消 + ROI Capon (净化后)');

%% 3. 循环
for iFrm = 157 : 1 : TOTAL_FRAMES
    
    current_time = (iFrm - 1) * FRAME_PERIOD;
    
    % --- 计算部分 ---
    try radarData = readBin(iFrm, 0); catch, break; end
    
    % 1. FFT
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0); 
    
    % 2. 坐标计算
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ped_pts_radar = transform_world_to_radar(ped_init_mat, EGO_VELOCITY, RADAR_YAW, current_time);
    
    % 3. Mask 生成 (用于划定 ROI，也用于告诉RELAX算法车在哪里)
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
    PHASE_NAME = 'Interference Cancellation...';
    
    % =====================================================================
    % 【Step 4.5: 物理层干扰对消 (Signal Purification)】
    % 核心思想: 模拟车的信号(含旁瓣)，从原始数据中减去它
    % =====================================================================
    
    % 1. 复制一份数据用于净化，保留原始数据用于对比
    fftRsltRg_Clean = fftRsltRg; 
    
    % 2. 确定车的范围 (Range Domain)
    % 简单做法: 取车几何中心前后 1.5米 的范围
    car_center_rho = (car_pts.A.rho + car_pts.K.rho) / 2;
    dist_mask = abs(full_grid.range - car_center_rho) < 2.5; % 稍微放宽一点范围，覆盖整个车身
    car_range_indices = find(dist_mask);
    
    % 3. 确定车的角度范围 (Angle Domain)
    % 直接使用包含所有顶点的 all_x 和 all_y 数组
    % 注意: 根据你的坐标系定义 (x = r*sin, y = r*cos), 角度应该是 atan2(x, y)
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);
    
    min_car_ang = min(all_angles) - 2; % 留一点余量
    max_car_ang = max(all_angles) + 2;

    
    % 4. 逐个距离门进行“清洗”
    for r_idx = car_range_indices'
        % A. 提取快拍 & 预处理
        raw_slice = squeeze(fftRsltRg(r_idx, :, :, :));
        if ndims(raw_slice) == 3 
             [nCh, nRx, nTx] = size(raw_slice);
             raw_slice = reshape(raw_slice, nCh, nRx*nTx);
        end
        snapshot = raw_slice.'; % -> [M天线 x N快拍]
        [M, N_Snaps] = size(snapshot);
        
        % =========================================================
        % === RELAX 算法核心: 构建完美的车辆干扰模型 ===
        % =========================================================
        
        K_Comps = 3; % 假设车身由 3 个主要散射中心组成
        
        % 存储每个分量的参数
        rel_angles = zeros(1, K_Comps);
        rel_alphas = zeros(1, K_Comps); % 暂时假设单快拍或相干
        
        % --- 阶段 1: 初始化 (就是之前的 CLEAN 过程) ---
        residual = snapshot;
        search_ang = min_car_ang : 0.5 : max_car_ang; % 仅在车身范围内搜索
        A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang)); % 预计算导向矢量库
        
        for k = 1:K_Comps
            % Beamforming 找最大值
            spec = sum(abs(A_scan' * residual), 2);
            [~, idx] = max(spec);
            rel_angles(k) = search_ang(idx);
            
            % 投影求幅度
            a_k = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            rel_alphas(k) = (a_k' * residual(:,1)) / (a_k' * a_k); % 简化取第1个快拍幅值
            
            % 更新残差
            residual = residual - rel_alphas(k) * a_k;
        end
        
        % --- 阶段 2: RELAX 迭代优化 ---
        % 反复打磨这 3 个点，让它们配合得天衣无缝
        MAX_RELAX_ITER = 5; % 迭代 5 次通常足够收敛
        
        for iter = 1 : MAX_RELAX_ITER
            for k = 1 : K_Comps
                % 1. "反悔": 把当前要优化的第 k 个点加回来
                %    (或者说：从原始数据中，把除了 k 以外的所有点减掉)
                data_k = snapshot; % 从原始数据开始
                for other = 1 : K_Comps
                    if other ~= k
                        a_other = exp(-1j * pi * (0:M-1)' * sind(rel_angles(other)));
                        data_k = data_k - rel_alphas(other) * a_other;
                    end
                end
                
                % 2. "重算": 在更纯净的环境下重新估计第 k 个点
                %    (依然限制在车身角度范围内)
                spec = sum(abs(A_scan' * data_k), 2);
                [~, idx] = max(spec);
                rel_angles(k) = search_ang(idx); % 更新角度
                
                % 更新幅度
                a_new = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
                rel_alphas(k) = (a_new' * data_k(:,1)) / (a_new' * a_new);
            end
        end
        
        % =========================================================
        % === 最终净化: 减去优化后的车辆模型 ===
        % =========================================================
        
        % 构建最终的车辆总信号 (Total Interference)
        total_interference = zeros(size(snapshot));
        for k = 1 : K_Comps
            a_final = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            total_interference = total_interference + rel_alphas(k) * a_final;
        end
        
        % 执行减法
        snapshot_clean = snapshot - total_interference;
        
        % 放回数据矩阵
        try
            fftRsltRg_Clean(r_idx, :, :) = snapshot_clean.'; 
        catch
             fftRsltRg_Clean(r_idx, :, :, :) = reshape(snapshot_clean.', [1, size(raw_slice,1), size(raw_slice,2)/size(raw_slice,3), size(raw_slice,3)]);
        end
    end
    
    % =====================================================================
    

    % 5. Capon 成像
    % 图3: 用原始数据 (展示脏的情况)
    [pwRA_Full, ~] = dbfProc1D(fftRsltRg, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', [], 'pcEn', 0, 'drawEn', 0);
        
    % 图4: 用【净化后的数据】 + 【Mask】
    % 这就是你的核心思路：先减去车的影响，再用 Mask 聚焦 ROI
    [pwRA_Roi, ~] = dbfProc1D(fftRsltRg_Clean, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', ShadowMask, 'pcEn', 0, 'drawEn', 0);
        
    % --- 绘图部分 ---
    if ~ishandle(hFig), break; end 
    
    % 更新车身
    car_x = [car_pts.all_x; car_pts.all_x(1)];
    car_y = [car_pts.all_y; car_pts.all_y(1)];
    set(h_car_fill, 'XData', car_x, 'YData', car_y);
    set(h_car_edge, 'XData', car_x, 'YData', car_y);
    set(h_car_overlay, 'XData', car_x, 'YData', car_y);
    set(h_ped_geom, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    set(h_ped_overlay1, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    set(h_ped_overlay2, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    
    % 更新标题与轮廓
    set(title_str1, 'String', {['Frame ' num2str(iFrm)], ['\color{red}' PHASE_NAME]});
    delete(findobj(ax1, 'Type', 'contour')); 
    contour(ax1, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'm--', 'LineWidth', 1);
    delete(findobj(ax3, 'Type', 'contour'));
    contour(ax3, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    delete(findobj(ax4, 'Type', 'contour'));
    contour(ax4, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    
    % 更新 Range-FFT
    mag_data = abs(fftRsltRg);
    range_profile = 20 * log10(mean(mag_data, [2, 3, 4]) + 1e-6);
    set(h_rfft_line, 'XData', full_grid.range, 'YData', range_profile);
    set(h_xline_A, 'Value', car_pts.A.rho);
    
    % 更新热力图
    set(h_pcolor1, 'CData', pwRA_Full); 
    set(h_pcolor2, 'CData', pwRA_Roi); 
    
    drawnow limitrate; 
end

%% ================= 辅助函数: 通用坐标变换 =================
function pts_radar = transform_world_to_radar(pts_world, v, yaw, t)
    dy = v * t;
    X_trans = pts_world(:, 1);
    Y_trans = pts_world(:, 2) - dy;
    theta = -yaw;
    X_radar = X_trans * cosd(theta) - Y_trans * sind(theta);
    Y_radar = X_trans * sind(theta) + Y_trans * cosd(theta);
    pts_radar = [X_radar, Y_radar];
end