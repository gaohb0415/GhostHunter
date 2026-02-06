% mmWaveMatlab主函数 - 分块处理最终版 (Batch Processing RELAX)
% 功能: 通过将时间切片，彻底解决动目标多普勒相位旋转问题
% ---------------------------------------------------------
%% 1. 初始化与配置
close all; clear; clc; 
addpath(genpath(pwd));

% =====================【核心参数配置】=====================
PARAM_K_MAX = 6;       % 最大搜索点数
IMPROVE_TH  = 0.01;    % 改善率阈值 (恢复到正常值 1%)
BLOCK_SIZE  = 32;      % 【核心】：将128个chirp切分为32个一组，共4组
% ============================================================

% =====================【显示功能开关】=====================
SHOW_K_POINTS = 0;           
ENABLE_FULL_ANGLE_VIEW = 0;     % 是否进行全角度搜索
% =========================================================

% --- 动力学参数 ---
EGO_VELOCITY = 0.363;        
RADAR_YAW    = -30;          
TOTAL_FRAMES = 210; 
FRAME_PERIOD = 50e-3;

% --- 扫描参数 ---
CFG_LIMIT_ANG = [-90, 90];      % 雷达扫描的边界角度
CFG_RES_ANG   = 0.5;            % 扫描分辨率和步长
CFG_LIMIT_R   = [];

% --- 载入配置 ---
config2243; 
try load('config.mat', 'resR', 'resV', 'spacingCal'); 
catch, load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal'); end
if ~exist('spacingCal', 'var'); spacingCal = 1; end

% --- 网格构建 --- （极坐标系 → 直角坐标系）
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)';                                   % ADC采样点数决定距离
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);        % CFG扫描参数决定角度
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);          % 构建（距离点 × 网格点）网格矩阵
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);

% =====================【实验真值数据】=====================
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
ped_path_x = [3.44, 3.44, 1.11];
ped_path_y = [5.8, 8.62, 8.62];
ped_init_mat = [ped_path_x', ped_path_y'];



%% 2. 预创建绘图对象 (1x3 布局)
%% 创建多个图片的窗口，后续数据更新的时候直接渲染就可以，防止画面闪烁，增强流畅性
hFig = figure('Name', 'Final Fix: Batch Processing RELAX', 'Position', [50, 100, 1600, 500]);

% 子图1: 原始
ax1 = subplot(1, 3, 1); hold on; axis equal; grid on;
h_pcolor_raw = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_raw, 'EdgeColor', 'none'); shading interp; colormap(ax1, 'jet');
h_ped_ov_raw = plot(nan, nan, 'w:', 'LineWidth', 2); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str1 = title('1. Original');

% 子图2: 修复版 RELAX
ax2 = subplot(1, 3, 2); hold on; axis equal; grid on;
h_pcolor_clean = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor_clean, 'EdgeColor', 'none'); shading interp; colormap(ax2, 'jet');
h_ped_ov_clean = plot(nan, nan, 'w:', 'LineWidth', 2); 

if SHOW_K_POINTS; visStr = 'on'; else; visStr = 'off'; end
h_kpts_global = plot(nan, nan, 'rs', 'MarkerSize', 6, 'LineWidth', 1.5, 'Visible', visStr); 

xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str2 = title('2. Batch Processed RELAX');

% 子图3: K值监控
ax3 = subplot(1, 3, 3); hold on; grid on;
h_bar_k = bar(full_grid.range, zeros(size(full_grid.range)), 'FaceColor', [0.2 0.6 0.8]);
xlim([0, 15]); ylim([0, PARAM_K_MAX + 1]); 
xlabel('Range (m)'); ylabel('Adaptive K');
title_str3 = title('3. K Profile');
yline(PARAM_K_MAX, 'r--', 'Limit');


%% 3. 循环
fprintf('================ STARTING BATCH FIX ================\n');
fprintf('Processing RELAX in small time-blocks to handle Doppler.\n');

% 从第14帧附近开始跑，验证是否崩溃
% 外层大循环的是帧数，每次都是一帧一帧处理（可以调整步进）
for iFrm = 170 : 1 : 210 
    
    current_time = (iFrm - 1) * FRAME_PERIOD;
    

    % 状态复位
    % coll：干扰位置
    % counts_per_bin：每个距离门进行了多少次的RELAX迭代
    % 帧循环的时候这两个参数必须都进行重置，不然会遗留上次旧数据
    k_coll.R = []; k_coll.Ang = [];
    k_counts_per_bin = zeros(size(full_grid.range)); 
    

    % 雷达信号预处理
    % 先读出当前帧原始的ADC数据
    % 将时域信号转换为距离域信号（原始数据的第一维度含义从采样点变成了距离门索引）
    % 获得了距离信息，但是还没有角度信息
    try radarData = readBin(iFrm, 0); catch, break; end
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);


    
    % --- 通道排序 ---
    % 将物理天线数据重组为"虚拟天线阵列"
    antArray_Sorted = virtualArray1D(fftRsltRg, 'DBF');     % 虚拟孔径扩展
    sorted_data = antArray_Sorted.signal; % [12, 128, 256]

    
    
    % --- 几何计算 ---
    % 计算当前帧中真值位置（行人路径 + 车辆真值位置） + 掩膜生成
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ped_pts_radar = transform_world_to_radar(ped_init_mat, EGO_VELOCITY, RADAR_YAW, current_time);
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);    % 生成干扰掩膜
    



    % --- 动态距离门 ---
    % 在纵向距离维度锁定干扰
    % 找出哪些距离门中有车
    all_corner_rhos = sqrt(car_pts.all_x.^2 + car_pts.all_y.^2);    % 确定车的四个角点
    % 找到干扰车的最近点和最远点
    car_rho_min = min(all_corner_rhos);
    car_rho_max = max(all_corner_rhos);
    % 只保留车身范围内的距离点，并且前后各自放宽0.5米
    dist_mask = (full_grid.range > (car_rho_min - 0.5)) & (full_grid.range < (car_rho_max + 0.5));
    car_range_indices = find(dist_mask);
    


    % --- 搜索范围 ---（从全图搜索变成局部搜索，减少计算量）
    % 在横向（角度维度）锁定干扰
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);          % 计算视角
    % 确定扇形边界
    min_car_ang = min(all_angles) - 5; 
    max_car_ang = max(all_angles) + 5;
    % 生成角度的搜索列表
    search_ang_global = min_car_ang : 0.5 : max_car_ang; 
    

    % 清除噪音之后的干净取值：初始化为原始数据
    sorted_data_clean = sorted_data; 


    
    % 从动态距离门开始进行搜索
    for r_idx = car_range_indices'

        % 当前距离门对应的物理距离
        current_R = full_grid.range(r_idx);
        
        % 提取完整快拍: [M=12, N=128]
        snapshot_full = squeeze(sorted_data(:, :, r_idx)); 
        [M, N_Total] = size(snapshot_full);
        
        % 初始化全零干扰信号，循环完成之后就会填满成为完整的干扰信号估计值，然后使用原始信号减去它进行噪声消除
        interference_full = zeros(M, N_Total);
        
        % ========================================================
        % 【核心方案】：分块循环 (Batch Processing Loop)
        % ========================================================
        % 将 128 个 chirp 切分成块，每块独立跑 RELAX
        num_blocks = ceil(N_Total / BLOCK_SIZE);
        max_k_in_blocks = 0; % 记录这一帧用到的最大K
        
        for b = 1:num_blocks
            % 1. 确定当前块的起止索引（将128个chirp进行拆分）
            idx_start = (b-1) * BLOCK_SIZE + 1;
            idx_end   = min(b * BLOCK_SIZE, N_Total);
            current_indices = idx_start : idx_end;
            
            % 2. 提取子快拍 [M, 32]
            % 从完整的大矩阵 snapshot_full (12 × 128) 中，抠出了一个小矩阵 (12 × 32)。
            snapshot_batch = snapshot_full(:, current_indices);
            [~, N_Batch] = size(snapshot_batch);
            
            % 3. 跑 RELAX 
            % 能量检测与日志控制，算一下这一小块信号的能量有多大
            slice_energy = sum(abs(snapshot_batch(:)).^2);
            do_print = (slice_energy > 1e9 && b==1); % 只打印第一块的信息避免刷屏
            

            % 调用算法
            % 最后获得alphas，angles两个结果
            % alphas：干扰源的强度
            % angles：干扰源的角度
            % 提取干扰车在短时间内在什么角度（angles），有着什么样的信号（alphas）
            [~, alphas, angles] = run_relax_core_batch(snapshot_batch, PARAM_K_MAX, IMPROVE_TH, ...
                search_ang_global, M, do_print, current_R, spacingCal);
            
            % 4. 记录 K 值
            if length(alphas) > max_k_in_blocks
                max_k_in_blocks = length(alphas);
            end
            

            % 5. 重构这一小块的干扰并填回去
            if ~isempty(alphas)

                % 伪造干扰信号，已知了干扰的角度和幅度，生成一个完美的干扰波形
                interf_batch = reconstruct_signal_batch(alphas, angles, M, N_Batch, spacingCal);

                % 将这个干扰小块填写到刚才的干扰信号矩阵中
                interference_full(:, current_indices) = interf_batch;
                
                % 收集调试点 (只收集第一块的，代表这一帧)
                if b == 1
                    k_coll.R = [k_coll.R; repmat(current_R, length(angles), 1)];
                    k_coll.Ang = [k_coll.Ang; angles.'];
                end
            end
        end
        
        k_counts_per_bin(r_idx) = max_k_in_blocks;
        
        % 执行减法 (从完整数据中减去拼凑好的干扰)
        sorted_data_clean(:, :, r_idx) = snapshot_full - interference_full;
    end


    % 下面是绘图部分
    
    % --- 绘图更新 ---
    if ENABLE_FULL_ANGLE_VIEW; procMask = ones(size(ShadowMask)); else; procMask = ShadowMask; end
    
    % 1. 原始图
    [pwRA_Raw, ~] = dbfProc1D(fftRsltRg, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, 'limitR', CFG_LIMIT_R, 'Mask', procMask);
    
    % 2. Clean 图
    pwRA_Clean_Matrix = zeros(nAdc, length(full_grid.angle));
    for iRg = 1 : nAdc
        sig_current = sorted_data_clean(:, :, iRg);
        if isempty(procMask)
             [pw_row, ~] = dbf(full_grid.angle', [], sig_current, antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
             pwRA_Clean_Matrix(iRg, :) = pw_row;
        else
             valid_idx = find(procMask(iRg, :) == 1);
             if ~isempty(valid_idx)
                 ang_subset = full_grid.angle(valid_idx)';
                 [pw_subset, ~] = dbf(ang_subset, [], sig_current, antArray_Sorted.arrayPos, [], 'spacingCal', spacingCal);
                 pwRA_Clean_Matrix(iRg, valid_idx) = pw_subset;
             end
        end
    end
    pwRA_Clean = pwRA_Clean_Matrix; 
    
    if ~ishandle(hFig), break; end
    
    set(h_ped_ov_raw, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    set(h_ped_ov_clean, 'XData', ped_pts_radar(:,1), 'YData', ped_pts_radar(:,2));
    
    axes_list = [ax1, ax2];
    for ax = axes_list
        delete(findobj(ax, 'Type', 'contour'));
        contour(ax, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1);
    end
    
    set(h_pcolor_raw, 'CData', pwRA_Raw);
    set(h_pcolor_clean, 'CData', pwRA_Clean);
    
    if SHOW_K_POINTS && ~isempty(k_coll.R)
        pol2cart_loc = @(r, deg) [r .* sind(deg), r .* cosd(deg)]; 
        xy = pol2cart_loc(k_coll.R, k_coll.Ang);
        set(h_kpts_global, 'XData', xy(:,1), 'YData', xy(:,2), 'Visible', 'on');
    else
        set(h_kpts_global, 'XData', nan, 'YData', nan, 'Visible', 'off');
    end
    
    set(h_bar_k, 'YData', k_counts_per_bin);
    title_suffix = [' (Frame ' num2str(iFrm) ')' ];
    set(title_str1, 'String', ['1. Original' title_suffix]);
    drawnow;
end



%% ================= 信号处理 =================

% run_relax_core_batch： 在混乱的信号中，找到最强的几个干扰源，并且计算一下角度和强度
% 输出：
% angles：干扰源的最优角度估计
% alphas：干扰源的最优复振幅估计
function [residual, rel_alphas, rel_angles] = run_relax_core_batch(input_signal, K_MAX, improve_th, search_ang, M, do_print, range_val, spacingCal)
    
    % 构建搜索网格，所有可能出现干扰的角度
    min_a = min(search_ang);
    max_a = max(search_ang);
    high_res_search_ang = min_a : 0.1 : max_a; 
    
    residual = input_signal;
    rel_angles_buf = zeros(1, K_MAX); rel_alphas_buf = zeros(1, K_MAX);
    
    total_raw_energy = sum(abs(input_signal(:)).^2);
    prev_energy = total_raw_energy;
    
    actual_k = 0; 
    
    % 构建导向矢量矩阵：每一个候选角度对应的"理想信号波形"
    A_scan = exp(1j * pi * (0:M-1)' * spacingCal * sind(high_res_search_ang)); 
    
    if do_print
        fprintf('>> Bin %.2fm | E: %.2e | ', range_val, total_raw_energy);
    end
    

    % 核心逻辑: 找最大值 -> 算出它的波形 -> 从总信号里减去它 -> 检查减完后能量降了多少。
    for k = 1:K_MAX
        % 扫描谱峰
        spec = sum(abs(A_scan' * residual), 2); 
        [~, idx] = max(spec);       % 找到最强峰值
        
        % 参数估计
        curr_ang = high_res_search_ang(idx);
       
        a_k = exp(1j * pi * (0:M-1)' * spacingCal * sind(curr_ang));
        curr_alpha = (a_k' * residual(:,1)) / (a_k' * a_k);
        
        % 信号剥离
        temp_residual = residual - curr_alpha * a_k;
        current_energy = sum(abs(temp_residual(:)).^2);
        
        % 收敛检查（能量下降率）
        improvement = (prev_energy - current_energy) / prev_energy;
        
        if do_print
            if improvement < 0
                fprintf('[BAD: Drop=%.2f%%] ', improvement*100); 
            else
                fprintf('[K%d: +%.2f%%] ', k, improvement*100); 
            end
        end
        
        % 剥离已经没有效果了就就停止
        if k > 1 && improvement < improve_th
            break; 
        end
        
        rel_angles_buf(k) = curr_ang; rel_alphas_buf(k) = curr_alpha;
        residual = temp_residual; prev_energy = current_energy; actual_k = k;
    end
    
    if do_print; fprintf('\n'); end 
    
    if actual_k == 0; rel_alphas = []; rel_angles = []; return; end
    rel_angles = rel_angles_buf(1:actual_k); rel_alphas = rel_alphas_buf(1:actual_k);
    

    % actual_k：上面初次选定好的嫌疑点数量
    % rel_angles：嫌疑点对应的初始位置
    % rel_alphas：对应的初始强度

    % 迭代优化
    MAX_ITER = 5;
    for iter = 1:MAX_ITER
        for k = 1:actual_k
            % 准备数据：加回第 k 个信号
            data_k = input_signal;          % 最原始信号拿回来
            for other = 1:actual_k
                if other ~= k
                    % 减去除了 k 以外的所有其他干扰
                    a_other = exp(1j * pi * (0:M-1)' * spacingCal * sind(rel_angles(other))); 
                    data_k = data_k - rel_alphas(other) * a_other; 
                end
            end
            % 将当前的残差信号与上面的导向矢量矩阵做比对匹配
            % spec中哪个匹配值最大，就说明信号最可能是在哪个角度来的
            spec = sum(abs(A_scan' * data_k), 2); [~, idx] = max(spec); rel_angles(k) = high_res_search_ang(idx);
            a_new = exp(1j * pi * (0:M-1)' * spacingCal * sind(rel_angles(k))); rel_alphas(k) = (a_new' * data_k(:,1)) / (a_new' * a_new);
        end
    end
end


% reconstruct_signal_batch：负责把算出来的数学参数（角度、幅度），变回雷达能理解的时间序列信号
function total_sig = reconstruct_signal_batch(alphas, angles, M, N_Snaps, spacingCal)
    total_sig = zeros(M, N_Snaps);
    for k = 1:length(alphas)
        a_vec = exp(1j * pi * (0:M-1)' * spacingCal * sind(angles(k))); 
        total_sig = total_sig + alphas(k) * a_vec; 
    end
end

% transform_world_to_radar：处理运动学真值。 
function pts_radar = transform_world_to_radar(pts_world, v, yaw, t)
    dy = v * t; X_trans = pts_world(:, 1); Y_trans = pts_world(:, 2) - dy; theta = -yaw;
    X_radar = X_trans * cosd(theta) - Y_trans * sind(theta); Y_radar = X_trans * sind(theta) + Y_trans * cosd(theta);
    pts_radar = [X_radar, Y_radar];
end