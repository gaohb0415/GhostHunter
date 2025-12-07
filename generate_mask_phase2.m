function Mask = generate_mask_phase2(radar_grid, car_pts)
% GENERATE_MASK_PHASE2 阶段二专用：减法建模
% 逻辑：ROI = (视锥扇形) - (雷达与车身围成的阻挡区)

    %% 1. 准备直角坐标网格
    [Ang_Grid, Rng_Grid] = meshgrid(radar_grid.angle, radar_grid.range);
    X_Grid = Rng_Grid .* sind(Ang_Grid);
    Y_Grid = Rng_Grid .* cosd(Ang_Grid);
    
    %% 2. 确定视锥 (Cone)
    % 在阶段二，右切点是 C，左切点是 B (或K，取最小)
    % 假设右偏为正
    theta_right = car_pts.C.theta; 
    theta_left  = car_pts.B.theta; 
    
    % 简单的容错：取所有角的最大最小值作为视锥边界
    all_thetas = [car_pts.A.theta, car_pts.B.theta, car_pts.C.theta, car_pts.K.theta];
    cone_min = min(all_thetas);
    cone_max = max(all_thetas);
    
    r_Max = 10.0; % 最大探测距离
    
    % 生成扇形 Mask (包含车身和阴影)
    Mask_Cone = (Ang_Grid <= cone_max) & (Ang_Grid >= cone_min) & (Rng_Grid <= r_Max);
    
    %% 3. 定义阻挡多边形 (Blocker)
    % 使用凸包 (Convex Hull) 自动处理 O, A, B, C, K 的围合顺序
    % 这涵盖了“雷达与车之间的空地”以及“车身本身”
    pts_x = [0, car_pts.A.x, car_pts.B.x, car_pts.C.x, car_pts.K.x];
    pts_y = [0, car_pts.A.y, car_pts.B.y, car_pts.C.y, car_pts.K.y];
    
    k = convhull(pts_x, pts_y); % 获取凸包索引
    hull_x = pts_x(k);
    hull_y = pts_y(k);
    
    % 生成阻挡 Mask (在多边形内的点 = 1)
    Mask_Blocker = inpolygon(X_Grid, Y_Grid, hull_x, hull_y);
    
    %% 4. 执行减法
    % 阴影 = 扇形 扣除 阻挡
    Mask = Mask_Cone & (~Mask_Blocker);
    
    % 确保是逻辑矩阵
    Mask = double(Mask); 
end