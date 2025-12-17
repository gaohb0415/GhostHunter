%% RELAX 算法参数敏感性分析实验 (Parameter Sensitivity Analysis)
% 目的: 
% 1. 确定最佳散射中心数 K (Elbow Point)
% 2. 确定最佳迭代次数 Iterations (Convergence)
% 场景: Ghost Probe 车辆强遮挡场景
% 作者: 2025研究生
% -----------------------------------------------------------

close all; clear; clc;
addpath(genpath(pwd));

%% 1. 实验配置
TARGET_FRAME = 160;   % <--- 在这里修改你想分析的帧数 (建议选车在雷达附近的帧)
MAX_TEST_K   = 10;    % K 的扫描范围 1~10
MAX_TEST_ITER= 10;    % Iter 的扫描范围 1~10

% 加载雷达配置
config2243; 
try load('config.mat', 'resR', 'resV', 'spacingCal'); 
catch, load('.\config\config\config.mat', 'resR', 'resV', 'spacingCal'); end

% 动力学参数 (用于计算车的位置)
EGO_VELOCITY = 0.538; % 雷达车速
RADAR_YAW    = -30;   % 雷达偏角
FRAME_PERIOD = 50e-3;

% 网格与真值 (直接复制 main4 的逻辑)
nAdc = 256; 
full_grid.range = resR * (0 : nAdc - 1)'; 
% 真值数据
ground_truth_world.car.x = [ 1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [ 5.38, 5.38, 10.11, 10.11];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];

%% 2. 数据准备 (读取指定帧 + 提取ROI数据)
fprintf('正在读取并处理第 %d 帧数据...\n', TARGET_FRAME);

try 
    radarData = readBin(TARGET_FRAME, 0); 
catch
    error('无法读取 Frame %d，请检查数据路径!', TARGET_FRAME);
end

% A. Range-FFT
[fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);

% B. 虚拟阵列重排 (关键! 确保天线顺序正确)
antArray = virtualArray1D(fftRsltRg, 'DBF');
antPos = antArray.arrayPos; % 获取真实物理位置 [1 x M]

% C. 计算车的位置 (用于锁定 ROI)
current_time = (TARGET_FRAME - 1) * FRAME_PERIOD;
car_pts = get_car_dynamic_coords(TARGET_FRAME, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);

% D. 确定车的 距离范围 & 角度范围
car_center_rho = (car_pts.A.rho + car_pts.K.rho) / 2;
dist_mask = abs(full_grid.range - car_center_rho) < 2.5; % 选中车身所在的距离门
car_range_indices = find(dist_mask);

all_angles = atan2d(car_pts.all_x, car_pts.all_y);
min_car_ang = min(all_angles) - 2; 
max_car_ang = max(all_angles) + 2;
search_grid = min_car_ang : 0.5 : max_car_ang; % 仅在车身角度内搜索

if isempty(car_range_indices)
    error('当前帧 (%d) 车不在探测范围内，请换一帧 (如 150-180)!', TARGET_FRAME);
end

fprintf('  - 车辆距离范围: %.2f m ~ %.2f m (%d bins)\n', ...
    full_grid.range(car_range_indices(1)), full_grid.range(car_range_indices(end)), length(car_range_indices));
fprintf('  - 车辆角度范围: %.2f deg ~ %.2f deg\n', min_car_ang, max_car_ang);

%% 3. 实验一: 扫描 K (Scatter Points)
% 固定 Iterations = 5，观察 K 增加时残差的变化
FIXED_ITER = 5;
residue_E1 = zeros(1, MAX_TEST_K);

fprintf('\n>>> 开始实验 1: 扫描 K (1-%d), 固定 Iter=%d\n', MAX_TEST_K, FIXED_ITER);

% 这里的残差是所有车身 Range Bin 残差的总和
for k_val = 1 : MAX_TEST_K
    total_energy_k = 0;
    
    % 对车身覆盖的每一个距离门分别做 RELAX
    for r_idx = car_range_indices'
        snapshot = antArray.signal(:, :, r_idx); % [M x N]
        
        % 调用 RELAX 封装函数
        e_bin = run_relax_experiment(snapshot, antPos, k_val, FIXED_ITER, search_grid);
        total_energy_k = total_energy_k + e_bin;
    end
    
    residue_E1(k_val) = total_energy_k;
    fprintf('  K=%d: Residual Energy = %.2f\n', k_val, total_energy_k);
end

%% 4. 实验二: 扫描 Iterations (Convergence)
% 固定 K = 3 (基于实验1的假设), 观察 Iter 增加时残差的变化
FIXED_K = 3;
residue_E2 = zeros(1, MAX_TEST_ITER);

fprintf('\n>>> 开始实验 2: 扫描 Iterations (1-%d), 固定 K=%d\n', MAX_TEST_ITER, FIXED_K);

for iter_val = 1 : MAX_TEST_ITER
    total_energy_i = 0;
    
    for r_idx = car_range_indices'
        snapshot = antArray.signal(:, :, r_idx);
        e_bin = run_relax_experiment(snapshot, antPos, FIXED_K, iter_val, search_grid);
        total_energy_i = total_energy_i + e_bin;
    end
    
    residue_E2(iter_val) = total_energy_i;
    fprintf('  Iter=%d: Residual Energy = %.2f\n', iter_val, total_energy_i);
end

%% 5. 绘图与分析 (Paper Ready Figures)
hFig = figure('Name', 'Parameter Tuning for RELAX', 'Position', [100, 100, 1200, 500]);

% --- 子图1: K 的选择 (Elbow Curve) ---
subplot(1, 2, 1);
% 归一化残差以便观察 (相对于 K=1)
y_data_k = 10 * log10(residue_E1); 
plot(1:MAX_TEST_K, y_data_k, '-ro', 'LineWidth', 2, 'MarkerFaceColor', 'w', 'MarkerSize', 8);
hold on;
xline(3, 'b--', 'Selected K=3', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', 'FontSize', 12);
grid on;
xlabel('Number of Scattering Centers ($K$)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Residual Energy (dB)', 'Interpreter', 'latex', 'FontSize', 14);
title('Impact of $K$ on Clutter Suppression', 'Interpreter', 'latex', 'FontSize', 16);
% 添加注解: 拐点
text(3.2, y_data_k(3), '\leftarrow Elbow Point', 'FontSize', 12, 'Color', 'blue');



% --- 子图2: Iterations 的收敛性 ---
subplot(1, 2, 2);
y_data_i = 10 * log10(residue_E2);
plot(1:MAX_TEST_ITER, y_data_i, '-bs', 'LineWidth', 2, 'MarkerFaceColor', 'w', 'MarkerSize', 8);
hold on;
xline(5, 'r--', 'Selected I=5', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', 'FontSize', 12);
grid on;
xlabel('Relaxation Iterations ($I$)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Residual Energy (dB)', 'Interpreter', 'latex', 'FontSize', 14);
title('Convergence Analysis', 'Interpreter', 'latex', 'FontSize', 16);
% 添加注解: 收敛
text(5.2, y_data_i(5), '\leftarrow Converged', 'FontSize', 12, 'Color', 'red');

fprintf('\n实验完成。请查看生成的图表。\n');