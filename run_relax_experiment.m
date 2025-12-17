function final_residual_energy = run_relax_experiment(snapshot, antPos, K_val, Iter_val, search_angles)
% RUN_RELAX_EXPERIMENT 执行一次完整的 RELAX 算法并返回残差能量
% 输入:
%   snapshot:      [M天线 x N快拍] 的复数数据 (必须是空间有序的)
%   antPos:        [1 x M] 天线真实物理位置
%   K_val:         假设的散射中心数量 (Components)
%   Iter_val:      最大迭代次数
%   search_angles: 搜索的角度网格 (行向量)
% 输出:
%   final_residual_energy: 最终剩余的能量值 (标量)

    [M, N_Snaps] = size(snapshot);
    
    % 1. 预计算导向矢量字典 (Steering Vector Dictionary)
    % 维度: [M x Angle_Grid_Size]
    % 注意: antPos(:) 确保是列向量, search_angles(:)' 确保是行向量
    A_dict = exp(1j * pi * antPos(:) * sind(search_angles(:)')); 
    
    % 初始化参数存储
    est_indices = zeros(1, K_val); % 存储每个分量对应的角度索引
    est_alphas  = zeros(K_val, N_Snaps); % 存储每个分量的复振幅 [K x N]
    
    % 初始化残差
    residual = snapshot;
    
    % ==========================================================
    % RELAX 算法主流程
    % ==========================================================
    
    % RELAX 的逻辑是：先估计第1个，再估计第2个... 
    % 但在这个实验中，我们主要测试"在固定K下的迭代优化效果"
    % 所以我们先用 CLEAN 贪婪算法初始化 K 个点，然后进入 RELAX 循环优化
    
    % --- Step A: 初始化 (贪婪搜索 K 个点) ---
    for k = 1 : K_val
        % 1. 粗搜索最大值
        % spec: [1 x nAngles]
        spec = sum(abs(A_dict' * residual), 2); 
        [~, idx] = max(spec);
        est_indices(k) = idx;
        
        % 2. 估计幅度 alpha = (a' * r) / (a' * a)
        % a_vec: [M x 1]
        a_vec = A_dict(:, idx);
        % alpha: [1 x N] (每个快拍有一个幅度)
        est_alphas(k, :) = (a_vec' * residual) ./ (a_vec' * a_vec);
        
        % 3. 更新残差
        residual = residual - a_vec * est_alphas(k, :);
    end
    
    % --- Step B: 迭代优化 (Relaxation Process) ---
    % 如果 Iter_val 为 0 或 1，其实上面的初始化已经算了一次了
    % 这里的迭代是指"回头看"，修正之前的估计
    
    for iter = 1 : Iter_val
        for k = 1 : K_val
            % 1. "反悔": 把第 k 个分量加回来
            % data_for_k 就是"原始数据 - 其他所有分量"
            data_for_k = residual; % 当前残差
            
            % 恢复第 k 个分量
            a_old = A_dict(:, est_indices(k));
            component_k = a_old * est_alphas(k, :);
            data_for_k = data_for_k + component_k; 
            
            % 2. "重搜": 在 data_for_k 基础上重新找最佳角度
            spec = sum(abs(A_dict' * data_for_k), 2);
            [~, best_idx] = max(spec);
            est_indices(k) = best_idx; % 更新角度索引
            
            % 3. "重估": 更新幅度
            a_new = A_dict(:, best_idx);
            est_alphas(k, :) = (a_new' * data_for_k) ./ (a_new' * a_new);
            
            % 4. "更新残差": 减去新的第 k 分量
            residual = data_for_k - a_new * est_alphas(k, :);
        end
    end
    
    % ==========================================================
    % 计算最终残差能量 (F-norm 的平方)
    % ==========================================================
    final_residual_energy = sum(abs(residual(:)).^2);

end