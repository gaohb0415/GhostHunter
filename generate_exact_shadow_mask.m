function Mask = generate_exact_shadow_mask(radar_grid, car_pts)
% GENERATE_EXACT_SHADOW_MASK (最终修正版)
% 逻辑：生成车后阴影区 Mask。
% 阴影区定义为：距离大于车身背面(AK, BK) 且 在射线OA, OB夹角内的区域。

    %% 1. 初始化
    nRg = length(radar_grid.range);
    nAng = length(radar_grid.angle);
    Mask = zeros(nRg, nAng);
    
    % 提取关键距离
    r_A = car_pts.A.rho; 
    r_B = car_pts.B.rho; 
    r_K = car_pts.K.rho; 
    
    % 设定最大探测距离 (10米)
    r_Max = 10.0;
    
    % 提取关键角度 (用于边界锁定)
    ang_A = car_pts.A.theta; % 右边界 (通常为正)
    ang_B = car_pts.B.theta; % 左边界 (通常为负)
    
    % 确保角度大小关系正确 (Left < Right)
    % 如果坐标系定义不同，这里可能会反，sort一下保证通用性
    boundary_angs = sort([ang_A, ang_B]);
    min_cone_ang = boundary_angs(1); % 左边界 (B)
    max_cone_ang = boundary_angs(2); % 右边界 (A)

    %% 2. 逐层计算
    for i = 1 : nRg
        r = radar_grid.range(i);
        
        % === 阶段 1: 无遮挡 或 超出范围 ===
        if r < r_A || r > r_Max
            continue; 
        end
        
        % 准备当前的有效角度区间列表
        active_intervals = [];
        
        % === 阶段 2: 单边切入 (A < r < B) ===
        % 此时只切入右侧 AK 边。左侧还没到 B，属于车前空地，不需 Mask。
        % 右侧阴影：从动态交点到 A
        if r >= r_A && r < r_B
            theta_cross = solve_intersect(r, car_pts.A, car_pts.K);
            if ~isnan(theta_cross)
                % 阴影区是 [交点, A] (即靠近 A 的外侧)
                active_intervals = [active_intervals; sort([theta_cross, ang_A])];
            end
            
        % === 阶段 3: 双边过渡 (B < r < K) ===
        % 此时切入两侧。阴影分为左右两瓣，中间是车前空地。
        % 左瓣：[B, BK交点]
        % 右瓣：[AK交点, A]
        elseif r >= r_B && r < r_K
            % 计算右侧 AK 交点
            theta_cross_AK = solve_intersect(r, car_pts.A, car_pts.K);
            if ~isnan(theta_cross_AK)
                active_intervals = [active_intervals; sort([theta_cross_AK, ang_A])];
            end
            
            % 计算左侧 BK 交点
            theta_cross_BK = solve_intersect(r, car_pts.B, car_pts.K);
            if ~isnan(theta_cross_BK)
                active_intervals = [active_intervals; sort([ang_B, theta_cross_BK])];
            end
            
        % === 阶段 4: 全阴影区 (r >= K) ===
        % 车身已过，左右两瓣合二为一，覆盖全视场
        elseif r >= r_K
            active_intervals = [min_cone_ang, max_cone_ang];
        end
        
        % === 3. 填入 Mask ===
        for k = 1 : size(active_intervals, 1)
            lower_lim = active_intervals(k, 1);
            upper_lim = active_intervals(k, 2);
            
            % 找出角度轴上落在范围内的索引
            idx = (radar_grid.angle >= lower_lim) & (radar_grid.angle <= upper_lim);
            Mask(i, idx) = 1;
        end
    end
end

%% === 辅助函数 ===
function ang = solve_intersect(r, P1, P2)
    % 求圆 r 与线段 P1-P2 的交点
    dx = P2.x - P1.x;
    dy = P2.y - P1.y;
    a = dx^2 + dy^2;
    b = 2 * (P1.x * dx + P1.y * dy);
    c = P1.x^2 + P1.y^2 - r^2;
    delta = b^2 - 4*a*c;
    
    if delta < 0, ang = NaN; return; end
    
    t1 = (-b - sqrt(delta)) / (2*a);
    t2 = (-b + sqrt(delta)) / (2*a);
    
    % 找到落在线段内 [0, 1] 的 t
    valid_t = [];
    if t1 >= -1e-4 && t1 <= 1+1e-4, valid_t = t1;
    elseif t2 >= -1e-4 && t2 <= 1+1e-4, valid_t = t2; end
    
    if isempty(valid_t)
        ang = NaN;
    else
        x_cross = P1.x + valid_t * dx;
        y_cross = P1.y + valid_t * dy;
        ang = atan2d(x_cross, y_cross);
    end
end