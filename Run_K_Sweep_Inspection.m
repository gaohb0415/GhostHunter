function Run_K_Sweep_Inspection(target_frame)
% RUN_K_SWEEP_INSPECTION 单帧 K 值扫描查看工具 (单窗口全览版)
% 输入: target_frame (整数)，例如 50, 100
% 功能: 
% 1. 计算该帧的物理场景和距离。
% 2. 生成 1 张 Baseline (无RELAX) + 19 张 RELAX (K=2~20) 的热力图。
% 3. 在一个全屏窗口中以 4x5 网格一次性展示所有结果。
% 4. 【修正】正确计算遮挡车辆“左下角”到雷达的距离 (使用 car_pts.A)。

%% 1. 初始化与环境配置
clc; close all;
fprintf('======================================================\n');
fprintf('正在初始化环境，准备分析第 [%d] 帧...\n', target_frame);
fprintf('======================================================\n');

addpath(genpath(pwd));

% --- 动力学参数 ---
EGO_VELOCITY = 0.363;        
RADAR_YAW    = -30;          
FRAME_PERIOD = 50e-3;
CFG_LIMIT_ANG = [-90, 90]; 
CFG_RES_ANG   = 0.5; 
CFG_LIMIT_R   = [];

% --- 载入配置 ---
try config2243; catch, warning('config2243 not found in path'); end
try load('config.mat', 'resR', 'resV'); catch, load('.\config\config\config.mat', 'resR', 'resV'); end

% --- 网格构建 ---
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);

% --- 真值数据 ---
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
ped_path_x = [3.44, 3.44, 1.11];
ped_path_y = [5.8, 8.62, 8.62];
ped_init_mat = [ped_path_x', ped_path_y'];

%% 2. 单帧数据读取与预处理
iFrm = target_frame;
current_time = (iFrm - 1) * FRAME_PERIOD;

% 2.1 读取数据
try 
    radarData = readBin(iFrm, 0); 
catch
    error('无法读取第 %d 帧数据，请检查路径或帧数范围。', iFrm);
end

% 2.2 基础 Range-FFT
[fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);

% 2.3 几何计算
car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
ped_pts_radar = transform_world_to_radar(ped_init_mat, EGO_VELOCITY, RADAR_YAW, current_time);

% --- 【修正位置】 计算距离信息 (用于标题显示) ---
% Index 1 (x=1.11, y=3.63) 对应 car_pts.A，这是物理上的左下角
dist_to_BL = car_pts.A.rho; 

% 计算雷达到车辆最近点的距离 (物理最近)
dist_min = min([car_pts.A.rho, car_pts.B.rho, car_pts.C.rho, car_pts.K.rho]);

% 2.4 生成 Mask (ROI)
ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);
procMask = ShadowMask; 

%% 3. 核心循环：K 值扫描 (Baseline + K=2~20)
total_cases = 20; 
results_img = cell(1, total_cases);
titles_str  = cell(1, total_cases);

% --- 3.1 计算 Baseline (无 RELAX) ---
fprintf('Computing Baseline (No RELAX)...\n');
[pwRA_Base, ~] = dbfProc1D(fftRsltRg, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', procMask, 'pcEn', 0, 'drawEn', 0);
results_img{1} = pwRA_Base;
titles_str{1}  = 'Baseline (Original)';

% --- 3.2 循环计算 K = 2 ~ 20 ---
h_wait = waitbar(0, '正在暴力扫描 K 值 (2~20)...');

% 准备 RELAX 公共参数
car_center_rho = (car_pts.A.rho + car_pts.K.rho) / 2;
dist_mask = abs(full_grid.range - car_center_rho) < 2.5;
car_range_indices = find(dist_mask);
all_angles = atan2d(car_pts.all_x, car_pts.all_y);
min_car_ang = min(all_angles) - 2;
max_car_ang = max(all_angles) + 2;

for k_target = 2 : 20
    waitbar((k_target-1)/19, h_wait, sprintf('Processing RELAX K = %d ...', k_target));
    
    fftRsltRg_Temp = fftRsltRg; % Copy data
    
    % === 开始 RELAX 处理 ===
    for r_idx = car_range_indices'
        raw_slice = squeeze(fftRsltRg(r_idx, :, :, :));
        if ndims(raw_slice) == 3
            [nCh, nRx, nTx] = size(raw_slice);
            raw_slice = reshape(raw_slice, nCh, nRx*nTx);
        end
        snapshot = raw_slice.';
        [M, ~] = size(snapshot);
        
        K_Comps = k_target;
        rel_angles = zeros(1, K_Comps);
        rel_alphas = zeros(1, K_Comps);
        residual = snapshot;
        search_ang = min_car_ang : 0.5 : max_car_ang;
        A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang));
        
        % Init
        for k = 1:K_Comps
            spec = sum(abs(A_scan' * residual), 2);
            [~, idx] = max(spec);
            rel_angles(k) = search_ang(idx);
            a_k = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            rel_alphas(k) = (a_k' * residual(:,1)) / (a_k' * a_k);
            residual = residual - rel_alphas(k) * a_k;
        end
        
        % Iteration
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
        
        % Reconstruction & Subtraction
        total_interference = zeros(size(snapshot));
        for k = 1 : K_Comps
            a_final = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            total_interference = total_interference + rel_alphas(k) * a_final;
        end
        snapshot_clean = snapshot - total_interference;
        try fftRsltRg_Temp(r_idx, :, :) = snapshot_clean.'; catch; end
    end
    % === RELAX 结束 ===
    
    [pwRA_K, ~] = dbfProc1D(fftRsltRg_Temp, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', procMask, 'pcEn', 0, 'drawEn', 0);
    
    results_img{k_target} = pwRA_K; 
    titles_str{k_target}  = sprintf('RELAX K=%d', k_target);
end
close(h_wait);

%% 4. 全屏单一窗口绘图 (4x5 Grid)
fprintf('正在生成全景图表...\n');
ped_x = ped_pts_radar(:,1); ped_y = ped_pts_radar(:,2);
car_x = [car_pts.all_x; car_pts.all_x(1)]; car_y = [car_pts.all_y; car_pts.all_y(1)];

% 创建全屏窗口
hF = figure('Name', sprintf('Frame %d K-Sweep Overview', iFrm), ...
            'Units', 'normalized', 'OuterPosition', [0 0 1 1], ... % 强制全屏
            'Color', 'w');

% 添加总标题 (Super Title)
sgtitle(sprintf('Frame: %d | Dist to Bottom-Left (Pt A): %.2fm', ...
        iFrm, dist_to_BL), 'FontSize', 16, 'FontWeight', 'bold');

for i = 1 : 20
    ax = subplot(4, 5, i); % 4行5列
    hold on; axis equal; grid on;
    
    % 绘制热力图
    hP = pcolor(X_Plot, Y_Plot, results_img{i});
    set(hP, 'EdgeColor', 'none'); shading interp; colormap(ax, 'jet');
    
    % 叠加轮廓
    plot(car_x, car_y, 'm-', 'LineWidth', 1); 
    plot(ped_x, ped_y, 'w:', 'LineWidth', 1.5);   
    
    % 坐标轴与装饰 (为了节省空间，可适当简化字体)
    xlim([-11, 11]); ylim([0, 12]);
    if i > 15, xlabel('X(m)'); else, set(gca, 'XTickLabel', []); end % 只在最后一行显示X轴
    if mod(i,5)==1, ylabel('Y(m)'); else, set(gca, 'YTickLabel', []); end % 只在第一列显示Y轴
    
    title(titles_str{i}, 'FontSize', 11, 'FontWeight', 'bold');
    
    % 绘制ROI框
    contour(X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'k--', 'LineWidth', 0.5);
end

fprintf('分析完成！已在单一窗口展示所有 K 值结果。\n');

end

% ================= 辅助函数 =================
function pts_radar = transform_world_to_radar(pts_world, v, yaw, t)
    dy = v * t;
    X_trans = pts_world(:, 1);
    Y_trans = pts_world(:, 2) - dy;
    theta = -yaw;
    X_radar = X_trans * cosd(theta) - Y_trans * sind(theta);
    Y_radar = X_trans * sind(theta) + Y_trans * cosd(theta);
    pts_radar = [X_radar, Y_radar];
end