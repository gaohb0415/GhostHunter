function car_pts = get_car_dynamic_coords(iFrm, car_init_world, radar_vel, radar_yaw, dt)
% GET_CAR_DYNAMIC_COORDS (升级版: 输出所有物理角点)
% 增加了 C 点输出，用于阶段切换判断

    %% 1. 物理位移 & 2. 坐标变换 (保持不变)
    current_time = (iFrm - 1) * dt;
    dy = radar_vel * current_time; 
    
    X_world = car_init_world(:, 1);
    Y_world = car_init_world(:, 2);
    
    X_trans = X_world; 
    Y_trans = Y_world - dy;
    
    theta_rot = -radar_yaw; 
    X_radar = X_trans * cosd(theta_rot) - Y_trans * sind(theta_rot);
    Y_radar = X_trans * sind(theta_rot) + Y_trans * cosd(theta_rot);
    
    rho = sqrt(X_radar.^2 + Y_radar.^2);
    theta = atan2d(X_radar, Y_radar); 
    
    %% 3. 关键点识别 (物理绑定)
    % 假设输入真值顺序是矩形的顺时针或逆时针排列
    % 根据你的数据: P1(1.65, 0.63), P2(3.44, 0.63), P3(3.44, 3.48), P4(1.65, 3.48)
    % 在雷达看来(右偏15度)，P1是右后(A)，P2是右前(C)，P3是左前(K)，P4是左后(B)
    % 我们直接通过索引绑定物理身份，这样最稳健
    
    % 辅助打包函数
    pack_pt = @(idx) struct('x', X_radar(idx), 'y', Y_radar(idx), ...
                            'rho', rho(idx), 'theta', theta(idx));
    
    car_pts.A = pack_pt(1); % 右后 (Phase 1 切点)
    car_pts.C = pack_pt(2); % 右前 (Phase 2 切点)
    car_pts.K = pack_pt(3); % 左前 (最远点)
    car_pts.B = pack_pt(4); % 左后 (左切点)
   
    
    % 调试用的所有点
    car_pts.all_x = X_radar;
    car_pts.all_y = Y_radar;
    
    % 为了兼容旧逻辑，重新计算动态最远点(万一K不是P3)
    [~, idx_far] = max(rho);
    car_pts.K_Dynamic = pack_pt(idx_far); 

    
end