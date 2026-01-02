%% RELAX 算法病理诊断脚本 (维度修复版)
clc; clear; close all;
addpath(genpath(pwd));

% --- 1. 读取单帧数据 ---
target_frame = 160; % 车贴脸的一帧
config2243; 
try load('config.mat'); catch, load('.\config\config\config.mat'); end
radarData = readBin(target_frame, 0);
[fftRsltRg, ~] = fftRange(radarData, 'pcEn', 0, 'drawEn', 0);

% --- 2. 关键：天线重排 ---
antArray = virtualArray1D(fftRsltRg, 'DBF');
antPos = antArray.arrayPos; % [1 x M]
% 强制转为列向量，防止维度混淆
antPos = antPos(:); 
M = length(antPos); % 天线数量

% --- 3. 锁定车所在的距离门 ---
% 简单粗暴，直接找能量最大的那个距离门（肯定就是车）
% dim=2 是快拍维，mean后变成 [nAnt x nRg]，再sum变成 [1 x nRg]
mag_profile = sum(abs(squeeze(mean(antArray.signal, 2))), 1); 
[~, car_idx] = max(mag_profile);
fprintf('诊断位置: Frame %d, Range Bin %d\n', target_frame, car_idx);

% --- 4. 提取信号 (关键修复部分) ---
raw_slice = squeeze(antArray.signal(:, :, car_idx)); % 通常是 [M x N]

% 【自动维度修正】确保 snapshot 是 [M天线 x N快拍]
[d1, d2] = size(raw_slice);
if d1 == M
    snapshot = raw_slice; % 维度正确 [M x N]
elseif d2 == M
    snapshot = raw_slice.'; % 维度反了，转置回来 [M x N]
else
    error('维度错误: 信号维度 (%d x %d) 与天线数 M=%d 不匹配!', d1, d2, M);
end

[M_check, N_Snaps] = size(snapshot);
fprintf('信号维度检查: [天线数=%d, 快拍数=%d]\n', M_check, N_Snaps);

% --- 5. 运行 RELAX (K=1, 只消最强点) ---
K_test = 1; 
Iter_test = 5;
search_ang = -60:0.5:60; 

% 构造字典: [M x G]
% antPos 是 [M x 1], sind(...) 是 [1 x G] -> 结果 [M x G]
A_dict = exp(1j * pi * antPos * sind(search_ang(:)')); 

residual = snapshot;
est_indices = zeros(1, K_test);
est_alphas = zeros(K_test, N_Snaps);

fprintf('开始 RELAX 迭代诊断...\n');

% Init (CLEAN)
for k=1:K_test
    % A_dict': [G x M]
    % residual: [M x N]
    % spec: [G x N] -> sum -> [G x 1]
    spec = sum(abs(A_dict' * residual), 2); 
    [~, idx] = max(spec);
    est_indices(k) = idx;
    
    a_vec = A_dict(:, idx); % [M x 1]
    % 投影求幅度: (a'*r) / (a'*a)
    est_alphas(k,:) = (a_vec' * residual) ./ (a_vec' * a_vec);
    
    residual = residual - a_vec * est_alphas(k,:);
end

% Optimize (RELAX)
for iter=1:Iter_test
    for k=1:K_test
        a_old = A_dict(:, est_indices(k));
        data_k = residual + a_old * est_alphas(k,:);
        
        spec = sum(abs(A_dict' * data_k), 2);
        [~, best_idx] = max(spec);
        est_indices(k) = best_idx;
        
        a_new = A_dict(:, best_idx);
        est_alphas(k,:) = (a_new' * data_k) ./ (a_new' * a_new);
        
        residual = data_k - a_new * est_alphas(k,:);
    end
end

% --- 6. 画图诊断 (最关键的一步) ---
% 计算空间谱 (DBF)
% 原始信号谱
bf_orig_mat = A_dict' * snapshot; % [G x N]
bf_orig = 10*log10(sum(abs(bf_orig_mat).^2, 2) + 1e-6); % 转成功率谱 dB

% 净化后信号谱
bf_clean_mat = A_dict' * residual;
bf_clean = 10*log10(sum(abs(bf_clean_mat).^2, 2) + 1e-6);

% 归一化以便对比形状 (可选，这里直接画绝对值看能量下降)
figure('Position', [200, 200, 900, 600]);
plot(search_ang, bf_orig, 'r-', 'LineWidth', 2, 'DisplayName', '处理前 (Original)');
hold on;
plot(search_ang, bf_clean, 'b--', 'LineWidth', 2, 'DisplayName', '处理后 (Residual)');

% 标出 RELAX 选中的角度
detected_ang = search_ang(est_indices(1));
xline(detected_ang, 'k:', 'LineWidth', 1.5, 'DisplayName', ['RELAX选中角度: ' num2str(detected_ang) '°']);

grid on; legend;
xlabel('Angle (deg)'); ylabel('Power Spectrum (dB)');
title(['RELAX 效果诊断 (K=' num2str(K_test) ', Frame ' num2str(target_frame) ')']);
subtitle('判据: 蓝线在峰值处应显著低于红线 (凹陷)。若蓝线更高或没变，则存在相位误差。');