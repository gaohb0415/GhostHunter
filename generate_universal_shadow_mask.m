function Mask = generate_universal_shadow_mask(radar_grid, car_pts)
% GENERATE_UNIVERSAL_SHADOW_MASK 通用几何阴影生成 (减法逻辑)
% 逻辑：ROI = (最大视线夹角构成的扇形) - (雷达原点与车身构成的阻挡多边形)
% 适用性：覆盖 Phase 1 和 Phase 2 所有阶段，无需切换逻辑

    %% 1. 准备直角坐标网格
    % 将雷达的极坐标网格转换为直角坐标 X, Y，用于多边形判断
    [Ang_Grid, Rng_Grid] = meshgrid(radar_grid.angle, radar_grid.range);
    X_Grid = Rng_Grid .* sind(Ang_Grid);
    Y_Grid = Rng_Grid .* cosd(Ang_Grid);
    
    %% 2. 第一步：定义“大扇形视锥” (The Cone)
    % 找出所有关键点中，角度最大(最右)和最小(最左)的值
    % 这决定了阴影区的左右边界
    all_angles = [car_pts.A.theta, car_pts.B.theta, car_pts.C.theta, car_pts.K.theta];
    
    theta_min = min(all_angles); % 左边界 (Left Tangent)
    theta_max = max(all_angles); % 右边界 (Right Tangent)
    
    r_Max = 15; % 最大探测距离
    
    % 生成扇形 Mask
    % 逻辑：在左右切线夹角之间，且距离在最大范围内
    Mask_Cone = (Ang_Grid >= theta_min) & (Ang_Grid <= theta_max) & (Rng_Grid <= r_Max);
    
    %% 3. 第二步：定义“阻挡多边形” (The Blocker)
    % 这是一个由“雷达原点”和“车身轮廓”围成的封闭区域
    % 在这个区域内的点，要么是空气(雷达和车之间)，要么是车身，反正不是阴影。
    
    % 收集所有顶点：原点 + 车身4点
    poly_x = [0, car_pts.A.x, car_pts.B.x, car_pts.K.x, car_pts.C.x];
    poly_y = [0, car_pts.A.y, car_pts.B.y, car_pts.K.y, car_pts.C.y];
    
    % 使用 convhull 自动计算凸包索引
    % 这样不需要手动关心点的连接顺序(顺时针/逆时针)，算法会自动把它们围成一个圈
    k = convhull(poly_x, poly_y);
    hull_x = poly_x(k);
    hull_y = poly_y(k);
    
    % 生成阻挡 Mask (使用 MATLAB 内置的高效多边形判断)
    Mask_Blocker = inpolygon(X_Grid, Y_Grid, hull_x, hull_y);
    
    %% 4. 第三步：执行减法 (Subtraction)
    % 核心逻辑：阴影 = 扇形 - 阻挡区
    Mask = Mask_Cone & (~Mask_Blocker);
    
    % 确保输出为 double 类型 (0或1)
    Mask = double(Mask);
    
end