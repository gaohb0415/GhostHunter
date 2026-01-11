% mmWaveMatlab主函数 - 对比版 
% 功能: 10秒动态鬼探头 + 物理层干扰对消
% ---------------------------------------------------------

%% 1. 初始化与配置
close all; clear; clc; 
addpath(genpath(pwd));

% =====================【功能开关】=====================
% 0 = 仅显示 ROI (原模式): 能够看清弱目标，背景干净
% 1 = 显示全角度 (-90~+90): 能够直观对比RELAX把车身消除了多少
ENABLE_FULL_ANGLE_VIEW = 0; 
% ======================================================

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
try load('config.mat', 'resR', 'resV'); 
catch, load('.\config\config\config.mat', 'resR', 'resV'); end

% --- 网格构建 ---
% 构建一张初始的直角坐标系的表格图，用于盛放后面的直角坐标数据
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
full_grid.angle = CFG_LIMIT_ANG(1) : CFG_RES_ANG : CFG_LIMIT_ANG(2);
[Ang_Grid, Rng_Grid] = meshgrid(full_grid.angle, full_grid.range);

% 极坐标系转换成直角坐标系的坐标
X_Plot = Rng_Grid .* sind(Ang_Grid); 
Y_Plot = Rng_Grid .* cosd(Ang_Grid);

% =====================【实验真值数据】=====================
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
ped_path_x = [3.44, 3.44, 1.11];
ped_path_y = [5.8, 8.62, 8.62];
ped_init_mat = [ped_path_x', ped_path_y'];
% ===============================================================

%% 2. 预创建绘图对象
% 预先创建四个图的空图对象，后续每次for循环的时候都是在空图上直接渲染新数据，不必每个空图都重新渲染
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
xlim([0, 10]); ylim([40, 120]); 
xlabel('距离 (m)'); ylabel('幅度 (dB)');
title('2. Range-FFT');

% === 子图3: 对照组 (原始数据) ===
ax3 = subplot(2, 2, 3); hold on; axis equal; grid on;
h_pcolor1 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor1, 'EdgeColor', 'none'); shading interp; colormap(ax3, 'jet');
h_ped_overlay1 = plot(nan, nan, 'w:', 'LineWidth', 2.5); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str3 = title('3. 原始数据 (无RELAX)');

% === 子图4: 实验组 (RELAX数据) ===
ax4 = subplot(2, 2, 4); hold on; axis equal; grid on;
h_pcolor2 = pcolor(X_Plot, Y_Plot, zeros(size(X_Plot)));
set(h_pcolor2, 'EdgeColor', 'none'); shading interp; colormap(ax4, 'jet');
h_car_overlay = plot(nan, nan, 'm-', 'LineWidth', 2);
h_ped_overlay2 = plot(nan, nan, 'w:', 'LineWidth', 2.5); 
xlim([-11, 11]); ylim([0, 12]); xlabel('X (m)'); ylabel('Y (m)');
title_str4 = title('4. 干扰对消 (有RELAX)');



%% 3. 循环
% 
for iFrm = 1 : 5 : 210


    % 数据准备与先验辅助定位阶段
    % 进行RELAX之前利用groundTruth，精确对准干扰源

    current_time = (iFrm - 1) * FRAME_PERIOD;        % 当前物理时间，用于校准行人和车辆这一时刻运动位置

    % --- 计算部分 ---
    try radarData = readBin(iFrm, 0); catch, break; end

    % 1. FFT：ADC原始数据转换到距离域
    [fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);

    % 2. 坐标计算：计算干扰车相对于雷达的精确坐标
    car_pts = get_car_dynamic_coords(iFrm, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);
    ped_pts_radar = transform_world_to_radar(ped_init_mat, EGO_VELOCITY, RADAR_YAW, current_time);



    % 3. Mask 生成：ShadowMask后续进行Capon的时候用来确保只在ROI上面进行Capon
    ShadowMask = generate_universal_shadow_mask(full_grid, car_pts);

    % =====================================================================
    % 【Step 4.5: 物理层干扰对消 (Signal Purification)】
    % =====================================================================

    PHASE_NAME = 'Interference Cancellation...';
    fftRsltRg_Clean = fftRsltRg;           % fftRsltRg_Clean：RELAX之后减去了脏数据的fftRsltRg数据

    % 确定车的范围 (Range Domain)
    % 计算车辆的几何中心，然后取几何中心前后2.5m范围内的数据才进行处理
    car_center_rho = (car_pts.A.rho + car_pts.K.rho) / 2;
    dist_mask = abs(full_grid.range - car_center_rho) < 2.5;
    car_range_indices = find(dist_mask);


    % 确定车的角度范围 (Angle Domain)
    % 角度维锁定：计算车身四个角的角度，找出最小的角和最大的角
    all_angles = atan2d(car_pts.all_x, car_pts.all_y);
    min_car_ang = min(all_angles) - 2;
    max_car_ang = max(all_angles) + 2;



    % 逐个距离门进行“清洗”
    for r_idx = car_range_indices'

        % 数据输入REALX之前，必须把数据整理成标准的阵列信号处理的格式
        % 2. 提取切片：从巨大的三维矩阵中，取出当前这“一米”的数据
        raw_slice = squeeze(fftRsltRg(r_idx, :, :, :));

        % 3. 维度重整：[通道 x 接收 x 发射] -> [虚拟天线总数 x 快拍数]
        % 这一步非常关键！它把物理天线排列成了数学上的一个长向量。

        if ndims(raw_slice) == 3
            [nCh, nRx, nTx] = size(raw_slice);
            raw_slice = reshape(raw_slice, nCh, nRx*nTx);
        end

        % 4. 转置：snapshot 是 [M天线 x N快拍]
        % 这就是 RELAX 的输入 "y"。每一列代表某个时刻所有天线收到的一组数据。

        snapshot = raw_slice.';
        [M, N_Snaps] = size(snapshot);

        % === RELAX 算法 ===
        K_Comps = 3; % 设定我们要找 3 个强散射中心（车头、B柱、车尾）

        % 准备两个空数组，用来存“嫌疑人”的特征（角度和能量）
        rel_angles = zeros(1, K_Comps);
        rel_alphas = zeros(1, K_Comps);

        % --- 初始化 ---
        residual = snapshot; % 开始时，残差就是原始信号

        % 5. 构建字典 (Steering Vector Dictionary)
        % 我们只在“车所在的角域”内搜索 (min_car_ang ~ max_car_ang)，节省计算量
        search_ang = min_car_ang : 0.5 : max_car_ang;

        % 生成导向矢量矩阵 A_scan。
        % 这是一个“标准答案库”。每一列代表一个角度的理想信号相位。
        % 公式：a(theta) = exp(-j * pi * n * sin(theta))
        A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang));

        % 6. 循环 K 次，依次找出最强的 3 个点
        for k = 1:K_Comps

            % [核心步骤] 匹配滤波 (Matched Filter) / 波束形成
            % 拿残差信号去和字典里的每个角度做内积。
            % abs(...) 算的是相关性的模长。sum(..., 2) 是对所有快拍求和（非相干积累），提高信噪比。

            spec = sum(abs(A_scan' * residual), 2);

            % 找到峰值：这一步就是在问“现在哪个角度最亮？”
            [~, idx] = max(spec);
            rel_angles(k) = search_ang(idx); % 记录下这个可疑的角度

            % 7. 估计幅度 (Complex Amplitude Estimation)
            % 既然知道角度了，生成该角度的导向矢量 a_k
            a_k = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));

            % 最小二乘解 (Least Squares)：计算这个方向上的信号到底有多强(alpha)
            % 公式：alpha = (a^H * y) / (a^H * a)
            rel_alphas(k) = (a_k' * residual(:,1)) / (a_k' * a_k);

            % 8. 消除 (Subtraction)
            % 从残差里减去这个刚才找到的信号。
            % residual(新) = residual(旧) - alpha * a(theta)
            residual = residual - rel_alphas(k) * a_k;
        end

        % --- 迭代优化 ---
        MAX_RELAX_ITER = 5; % 给它 5 次修正机会

        for iter = 1 : MAX_RELAX_ITER
            % 对每一个已经找到的嫌疑人 k (1, 2, 3) 进行轮询修正
            for k = 1 : K_Comps

                % 9. “加回”操作 (Add Back)
                % 这是最精髓的一步！
                % 我们想重新估计第 k 个点，怎么做？
                % 我们把原始信号 snapshot 拿来，把“除了 k 以外的其他所有嫌疑人”都减掉。
                % 剩下的 data_k 里面就只包含：第 k 个点的信号 + 噪声。
                data_k = snapshot;
                for other = 1 : K_Comps
                    if other ~= k
                        % 生成其他人的信号模型
                        a_other = exp(-1j * pi * (0:M-1)' * sind(rel_angles(other)));
                        % 减去其他人
                        data_k = data_k - rel_alphas(other) * a_other;
                    end
                end

                % 10. 重搜 (Re-estimation)
                % 现在 data_k 很干净了（排除了其他强干扰），我们再测一次角度。
                % 这次测出来的角度，比初始化阶段要准得多！
                spec = sum(abs(A_scan' * data_k), 2);
                [~, idx] = max(spec);

                % 更新参数：用更准的值覆盖旧值
                rel_angles(k) = search_ang(idx);
                a_new = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
                rel_alphas(k) = (a_new' * data_k(:,1)) / (a_new' * a_new);
            end
            % 这一层循环结束后，3 个点的参数互相配合，误差越来越小。
        end


        % --- 最终净化 ---
        total_interference = zeros(size(snapshot));

        % 11. 重构干扰信号 (Interference Reconstruction)
        % 利用刚才算出来的“精准角度”和“精准幅度”，数学合成一个完美的遮挡车回波。
        for k = 1 : K_Comps
            a_final = exp(-1j * pi * (0:M-1)' * sind(rel_angles(k)));
            % 把 3 个强点的信号累加起来
            total_interference = total_interference + rel_alphas(k) * a_final;
        end

        % 12. 最终对消
        % 原始数据 - 完美的车 = 弱目标(行人) + 噪声
        snapshot_clean = snapshot - total_interference;

        % 13. 归位
        % 把清洗干净的数据放回原来的矩阵里，准备送去画图
        try
            fftRsltRg_Clean(r_idx, :, :) = snapshot_clean.';
        catch
            % ... (reshape 处理)
        end
    end

    % =====================================================================
    % 5. Capon 成像 (根据开关决定使用哪个 Mask)
    % =====================================================================

    if ENABLE_FULL_ANGLE_VIEW
        % 模式A: 全角度显示 (Mask 全为1，不遮挡任何东西)
        procMask = ones(size(ShadowMask));
        view_str = ' (Full View)';
    else
        % 模式B: 仅显示 ROI (使用 ShadowMask 遮挡非ROI区域)
        procMask = ShadowMask;
        view_str = ' (ROI Only)';
    end

    % 图3: 原始数据
    [pwRA_Baseline, ~] = dbfProc1D(fftRsltRg, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', procMask, 'pcEn', 0, 'drawEn', 0);

    % 图4: 净化数据
    [pwRA_Roi, ~] = dbfProc1D(fftRsltRg_Clean, 'limitAng', CFG_LIMIT_ANG, 'resAng', CFG_RES_ANG, ...
        'limitR', CFG_LIMIT_R, 'Mask', procMask, 'pcEn', 0, 'drawEn', 0);

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

    % 更新图3/4标题，指示当前模式
    set(title_str3, 'String', ['3. 原始数据 + Capon' view_str]);
    set(title_str4, 'String', ['4. RELAX净化 + Capon' view_str]);

    delete(findobj(ax1, 'Type', 'contour'));
    contour(ax1, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'm--', 'LineWidth', 1);

    % 无论什么模式，都把 ROI 的框画出来作为参考
    delete(findobj(ax3, 'Type', 'contour'));
    contour(ax3, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);
    delete(findobj(ax4, 'Type', 'contour'));
    contour(ax4, X_Plot, Y_Plot, ShadowMask, [0.5 0.5], 'w--', 'LineWidth', 1.5);

    % 更新 Range-FFT
    mag_data = abs(fftRsltRg);
    range_profile = 20 * log10(mean(mag_data, [2, 3, 4]) + 1e-6);
    set(h_rfft_line, 'XData', full_grid.range, 'YData', range_profile);

    % 更新热力图
    set(h_pcolor1, 'CData', pwRA_Baseline);
    set(h_pcolor2, 'CData', pwRA_Roi);

    % =======================================================
    % 【已删除】强制统一色标范围 (Lock CLim)
    % 现在使用 MATLAB 默认的 Auto-Scaling
    % =======================================================

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