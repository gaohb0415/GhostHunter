% 脚本名称: param_sensitivity_scan.m
% 功能: 参数敏感性扫描 (K值优选)，修复转置维度错误版
% ---------------------------------------------------------
close all; clear; clc; 
addpath(genpath(pwd));

% =====================【配置参数】=====================
FRM_IDX = 133;        % 选择一帧车身信号最典型的
K_SCAN_RANGE = 1:20;  % 扫描 K 从 1 到 20
EGO_VELOCITY = 0.363; % 自车速度 m/s
FRAME_PERIOD = 50e-3;
RADAR_YAW    = -30;

% --- 载入配置 ---
config2243; 
try load('config.mat', 'resR', 'resV', 'lambda', 'tChirpIntvl', 'spacingCal'); 
catch, load('.\config\config\config.mat', 'resR', 'resV', 'lambda', 'tChirpIntvl', 'spacingCal'); end

% =====================【数据准备】=====================
fprintf('正在读取第 %d 帧数据...\n', FRM_IDX);
try radarData_Raw = readBin(FRM_IDX, 0); catch, error('数据读取失败，请检查路径'); end
[nAdc, nChirp, nRx, nTx, ~] = size(radarData_Raw);

% --- 1. 准备 Baseline 数据 (无补偿) ---
radarData_Base = radarData_Raw;

% --- 2. 准备 Compensated 数据 (有补偿) ---
radarData_Comp = radarData_Raw;
for iTx = 1:nTx
    dt = (iTx - 1) * tChirpIntvl; 
    comp_factor = exp(1j * 4 * pi * EGO_VELOCITY * dt / lambda);
    radarData_Comp(:, :, :, iTx) = radarData_Comp(:, :, :, iTx) * comp_factor;
end

% --- 3. 执行 Range-FFT ---
[fftBase, ~] = fftRange(radarData_Base, 'pcEn', 0, 'drawEn', 0);
[fftComp, ~] = fftRange(radarData_Comp, 'pcEn', 0, 'drawEn', 0);

% --- 4. 几何计算 (锁定 ROI 区域) ---
ground_truth_world.car.x = [1.11, 2.87, 2.87, 1.11];
ground_truth_world.car.y = [3.63, 3.63, 7.97, 7.97];
car_init_mat = [ground_truth_world.car.x', ground_truth_world.car.y'];
car_pts = get_car_dynamic_coords(FRM_IDX, car_init_mat, EGO_VELOCITY, RADAR_YAW, FRAME_PERIOD);

% 找出车所在的 Range Bins
car_center_rho = (car_pts.A.rho + car_pts.K.rho) / 2;
r_grid = resR * (0:nAdc-1);
dist_mask = abs(r_grid - car_center_rho) < 2.5; % 车前后 2.5m
car_range_indices = find(dist_mask);

% 计算搜索角度范围
all_angles = [car_pts.A.theta, car_pts.B.theta, car_pts.C.theta, car_pts.K.theta];
search_ang_range = (min(all_angles)-3) : 0.5 : (max(all_angles)+3);

% =====================【主循环：扫描 K】=====================
resid_power_base = zeros(length(K_SCAN_RANGE), 1);
resid_power_comp = zeros(length(K_SCAN_RANGE), 1);

fprintf('开始参数扫描 (K = 1 ~ %d)...\n', max(K_SCAN_RANGE));
fprintf('|  K  | Base Resid (dB) | Comp Resid (dB) |  Gap (dB)  |\n');
fprintf('|-----|-----------------|-----------------|------------|\n');

for i = 1 : length(K_SCAN_RANGE)
    k_val = K_SCAN_RANGE(i);
    
    % 计算无补偿组的残差能量
    resid_power_base(i) = calc_relax_residual(fftBase, car_range_indices, k_val, search_ang_range);
    
    % 计算有补偿组的残差能量
    resid_power_comp(i) = calc_relax_residual(fftComp, car_range_indices, k_val, search_ang_range);
    
    gap = resid_power_base(i) - resid_power_comp(i);
    fprintf('| %3d |     %8.2f    |     %8.2f    |  %8.2f  |\n', ...
        k_val, resid_power_base(i), resid_power_comp(i), gap);
end

% =====================【绘图分析】=====================
figure('Name', 'Parameter Sensitivity Analysis', 'Position', [100, 100, 900, 700]);

subplot(2,1,1);
plot(K_SCAN_RANGE, resid_power_base, 'b-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'b'); hold on;
plot(K_SCAN_RANGE, resid_power_comp, 'r-^', 'LineWidth', 1.5, 'MarkerFaceColor', 'r');
grid on;
legend('Baseline (无补偿)', 'Ours (有补偿)', 'Location', 'northeast');
xlabel('Model Order (K)'); ylabel('Residual Energy (dB)');
title(['Residual Energy vs. Model Order K (Frame ' num2str(FRM_IDX) ')']);

subplot(2,1,2);
gain_curve = resid_power_base - resid_power_comp;
bar(K_SCAN_RANGE, gain_curve, 'FaceColor', [0.2 0.6 0.8]);
grid on;
xlabel('Model Order (K)'); ylabel('Performance Gain (dB)');
title('Performance Gain (Difference)');
% 标出最大值
[max_gain, max_idx] = max(gain_curve);
best_k = K_SCAN_RANGE(max_idx);
hold on;
plot(best_k, max_gain + 0.2, 'rhexagram', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
text(best_k, max_gain + 1.0, sprintf('Optimal K=%d\nGain=%.2fdB', best_k, max_gain), ...
    'HorizontalAlignment', 'center', 'Color', 'r', 'FontWeight', 'bold');
ylim([min(0, min(gain_curve)-1), max_gain + 2]);

%% ================= 辅助函数 1: 计算 RELAX 残差能量 (维度修复版) =================
function total_resid_db = calc_relax_residual(fftData, r_idxs, K, search_ang)
    % 0. 安全检查
    if isempty(r_idxs)
        total_resid_db = -100; 
        return; 
    end

    sum_resid_pow = 0;
    
    % 1. 鲁棒地获取维度信息
    sz = size(fftData); 
    % 假设 dim2 永远是 Chirp/Doppler 维 (TI板卡标准)
    nChirp = sz(2);
    
    % 【核心修复 step 1】先拿一个切片试算 M (虚拟天线数)
    % 这样可以避开手动计算 prod(sz(3:end)) 可能带来的错误
    test_slice_nd = fftData(r_idxs(1), :, :, :);
    test_slice = reshape(test_slice_nd, nChirp, []); % 自动推导第二维
    M = size(test_slice, 2); % 动态获取准确的 M 值
    
    % 2. 构造字典 A_scan (只生成一次)
    A_scan = exp(-1j * pi * (0:M-1)' * sind(search_ang)); 
    
    for r = r_idxs'
        % 3. 数据提取与重塑
        raw_slice_nd = fftData(r, :, :, :); 
        
        % 【核心修复 step 2】强制 Reshape
        % 无论 raw_slice_nd 是 3D 还是 4D，都强转为 [Chirps, Antennas]
        raw_slice = reshape(raw_slice_nd, nChirp, []); 
        
        y = raw_slice.'; % 转置为 [Antennas, Chirps] -> [M, N]
        
        % === 极速版 RELAX 核心 ===
        residual = y;
        rel_ang = zeros(1, K); rel_amp = zeros(1, K);
        
        % Init
        for k=1:K
            spec = sum(abs(A_scan' * residual), 2);
            [~,id] = max(spec);
            rel_ang(k) = search_ang(id);
            ak = exp(-1j*pi*(0:M-1)'*sind(rel_ang(k)));
            rel_amp(k) = (ak'*residual(:,1))/(ak'*ak);
            residual = residual - rel_amp(k)*ak;
        end
        
        % Iter (固定 5 次)
        for iter=1:5
             for k=1:K
                dk = y;
                for o=1:K, if o~=k, ao=exp(-1j*pi*(0:M-1)'*sind(rel_ang(o))); dk=dk-rel_amp(o)*ao; end; end
                spec=sum(abs(A_scan'*dk),2); [~,id]=max(spec); rel_ang(k)=search_ang(id);
                anew=exp(-1j*pi*(0:M-1)'*sind(rel_ang(k))); rel_amp(k)=(anew'*dk(:,1))/(anew'*anew);
             end
        end
        
        % Reconstruct
        intf = zeros(size(y));
        for k=1:K, intf=intf+rel_amp(k)*exp(-1j*pi*(0:M-1)'*sind(rel_ang(k))); end
        
        % 最终残差
        resid_final = y - intf;
        sum_resid_pow = sum_resid_pow + sum(abs(resid_final(:)).^2);
    end
    
    total_resid_db = 10*log10(sum_resid_pow + 1e-10);
end