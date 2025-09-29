function matchRslt = trajectoryIdMatch(track, frmSeg)
% 轨迹-人员姓名匹配
% 删除了有关orientation绘图的代码, 有需要时需查阅代码备份
% 输入:
% 1. tracks: 轨迹结构体
%  - .iPeople: 轨迹计数ID
%  - .name: 人员姓名ID
%  - .trajectory: 轨迹记录
%  - .frame: 帧记录
%  - .status: 轨迹状态
%  - .pcLast: 最后帧的点云
%  - .kalmanFilter: 卡尔曼滤波器
%  - .statusAge: 状态年龄
% 2. frmSeg: 参与匹配的帧
% 输出:
% matchRslt: 匹配结果
% - .match: 轨迹-人员姓名匹配表
% - .sumCost: 一个平行宇宙中全部轨迹的总cost
% 作者: 刘涵凯
% 更新: 2023-6-8

%% 参数对象及载入读取统计记录
p = trackParamShare.param;
load('config.mat', 'tFrm')
load(p.handleDataImu)

%% 追踪数据预处理
nTraj = structLength(track, 'iPeople');
for iTraj = 1 : nTraj
    % 初始化时间和朝向
    track(iTraj).time = 0;
    track(iTraj).orientation = 0;
    % 根据时间区间提取有效数据
    track(iTraj).trajectory = track(iTraj).trajectory(track(iTraj).frame >= frmSeg(1) & ...
        track(iTraj).frame <= frmSeg(end), :);
    track(iTraj).frame = track(iTraj).frame(track(iTraj).frame >= frmSeg(1) & ...
        track(iTraj).frame <= frmSeg(end));
    % 计算朝向
    [track(iTraj).orientation, iFrmMove] = trajOrientation(track(iTraj).trajectory);
    % 提取有效区间
    track(iTraj).time = tFrm * (track(iTraj).frame(iFrmMove) - 1);
    track(iTraj).orientation = track(iTraj).orientation(iFrmMove);
end

%% IMU数据预处理
nDev = length(dataPerDev);
for iDev = 1 : nDev
    imu(iDev).id = dataPerDev(iDev).id;
    imu(iDev).orientation = deg2rad(dataPerDev(iDev).data(:, 14));
    % 万向锁校准
    eulX =  deg2rad(dataPerDev(iDev).data(:, 12));
    eulY =  deg2rad(dataPerDev(iDev).data(:, 13));
    calFactor = 0;
    if prctile(eulY, 25) > pi / 4
        calFactor = -1;
    elseif prctile(eulY, 75) < -pi / 4
        calFactor = 1;
    end
    imu(iDev).orientation = unwrap(imu(iDev).orientation) + calFactor .* (unwrap(eulX) - eulX(1));
    % 在开头-5秒重复0秒的数据, 在结尾+5秒重复末尾数据, 从而让插值不报错
    imu(iDev).time = [dataPerDev(iDev).data(1, 1) - 5; dataPerDev(iDev).data(:, 1); dataPerDev(iDev).data(end, 1) + 5];
    imu(iDev).orientation = [imu(iDev).orientation(1); imu(iDev).orientation; imu(iDev).orientation(end)]; % rad
    imu(iDev).orientation = smoothdata(unwrap(imu(iDev).orientation), 'movmean', 1 * rateImu); % 平滑
end

%% 计算cost
% 初始化rho
rho = zeros(nTraj, nDev);
nFrmMatch = zeros(1, nTraj);
timeShift = -2 : 0.1 : 2; % 在多个timeShift下进行计算, 并寻找最优
for iTime = 1 : length(timeShift)
    for iTraj = 1 : nTraj
        % 参与匹配的帧数
        nFrmMatch(iTraj) = length(track(iTraj).time);
        if nFrmMatch(iTraj) < p.nFrmThId
            % 若有效时间小于阈值, 则不参与匹配
            rho(iTraj, iDev, iTime) = -1;
        else
            for iDev = 1 : nDev
                oriImu = interp1(imu(iDev).time, unwrap(imu(iDev).orientation), track(iTraj).time + timeShift(iTime), "linear");
                % 当前采用的cost公式:
                rho(iTraj, iDev, iTime) = abs(sum(exp(1i * (track(iTraj).orientation - oriImu)))) / nFrmMatch(iTraj);
            end
        end
    end
    similarity(iTime) = sum(max(rho(:, :, iTime), [], 2));
end
[~ , iBestTime] = max(similarity);
rho = rho(:, :, iBestTime);
cost = 1 - rho;

%% 匈牙利算法关联
[assignments, unassignedTracks, ~] = assignDetectionsToTracks(cost, p.costImu);

%% 写入匹配结果
% 初始化
matchRslt.match = string(zeros(nTraj, 2));
matchRslt.sumCost = 0;
% 轨迹索引
cntTraj = 1;
% 分两部分记录, 首先是配对成功部分
for iMatch = 1 : size(assignments, 1)
    matchRslt.match(iMatch, 1) = string(track(assignments(iMatch, 1)).iPeople);
    matchRslt.match(iMatch, 2) = p.name(imu(assignments(iMatch, 2)).id);
    cntTraj = cntTraj + 1;
    matchRslt.sumCost = matchRslt.sumCost + cost(assignments(iMatch, 1), assignments(iMatch, 2));
end
% 然后是配对失败部分
for iUnassigned = 1 : length(unassignedTracks)
    matchRslt.match(cntTraj, 1) = string(track(unassignedTracks(iUnassigned)).iPeople);
    % 将配对失败轨迹的姓名设为"N", 即"None"
    matchRslt.match(cntTraj, 2) = "N";
    cntTraj = cntTraj + 1;
    % 配对失败的轨迹, 以惩罚cost作为cost
    matchRslt.sumCost = matchRslt.sumCost + p.costPenal;
end
end
