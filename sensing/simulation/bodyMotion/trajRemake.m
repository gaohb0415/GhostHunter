% 基于现有轨迹执行轨迹重组
% 作者: 刘涵凯
% 更新: 2024-11-29

close all; clear

%% 参数对象
p = simParamConfig;

%% 现有轨迹处理
load(p.handleTrajectory) % 载入现有轨迹
nTraj = structLength(traj, 'pos'); % 现有轨迹条数
nFrmAll = cellfun(@(x) size(x, 1), {traj.pos})'; % 各现有轨迹时间长度 帧
trajAll = vertcat(traj.pos); % 将现有轨迹全部相连以便于后续计算
traj = [];
iStart = [1; cumsum(nFrmAll(1:end-1)) + 1]; % 各轨迹的起始索引
iEnd = iStart + nFrmAll - 1; % 各轨迹的结束索引
trajWin = [iStart, iEnd];

%% 轨迹重组
nRmk = 200; % 生成的重组轨迹条数
nLink = zeros(nRmk, 1); % 各重组轨迹中的连接点个数
iRmk = 1; % 初始化重组轨迹索引
while iRmk <= nRmk
    % 随机选取一条现有轨迹作为起始点
    trajSource.idx = ceil(nTraj * rand(1)); 
    trajSource.pos = trajAll(trajWin(trajSource.idx, 1) : trajWin(trajSource.idx, 2), :);
    trajSource.nFrm = nFrmAll(trajSource.idx);
    trajSource.iFrm = 1; % 从现有轨迹的该帧开始复制
    % 初始化重组轨迹
    trajNew = zeros(p.nFrmRmk, 2);
    % 三个与轨迹连接判定相关的参数
    n = 0; % 当前连续复制次数
    k = p.k; % 每k帧执行一次判定
    N = p.NRp; % 最大连续复制次数
    linkFlag = 0; % 变为1时执行一次轨迹连接
    iFrmLink = []; % 进行轨迹连接的帧索引

    %% 轨迹连接
    for iFrm = 1 : p.nFrmRmk
        if trajSource.iFrm > trajSource.nFrm % 若到达该现有轨迹结尾, 则执行轨迹连接
            linkFlag = 1;
        elseif mod(iFrm, k) == 0 % 每k帧执行一次判定
            if rand(1) < (n / N) ^ 2
                linkFlag = 1;
            end
        end
        if linkFlag % 执行轨迹连接
            linkFlag = 0;
            iFrmLink = [iFrmLink, iFrm]; % 记录连接帧
            % 寻找最适合连接的新现有轨迹
            d = vecnorm(trajAll - trajNew(iFrm - 1, :), 2, 2); % 现有轨迹坐标与重组轨迹最后坐标的距离
            d(trajWin(trajSource.idx, 1) : trajWin(trajSource.idx, 2)) = inf; % 忽略当前现有轨迹
            [~, iMin] = min(d); % 新现有轨迹索引
            % 更新当前现有轨迹信息
            trajSource.idx = find(iMin >= trajWin(:, 1) & iMin <= trajWin(:, 2));
            trajSource.pos = trajAll(trajWin(trajSource.idx, 1) : trajWin(trajSource.idx, 2), :);
            trajSource.nFrm = nFrmAll(trajSource.idx);
            trajSource.iFrm = iMin - trajWin(trajSource.idx, 1) + 1;
            n = 0; % 重置当前连续复制次数
        end
        % 执行一帧轨迹复制
        trajNew(iFrm, :) = trajSource.pos(trajSource.iFrm, :);
        trajSource.iFrm =trajSource.iFrm + 1;
        n = n + 1;
    end

    %% 平滑重组轨迹
    smoothWin = 15;
    for i = 1 : length(iFrmLink)
        idx = (iFrmLink(i) - smoothWin) : (iFrmLink(i) + smoothWin);
        if idx(1) < 1
            idx = idx - idx(1) + 1;
        end
        if idx(end) > p.nFrm
            idx = idx - (idx(end) - p.nFrm);
        end
        trajNew(idx, :) = smoothdataV3(trajNew(idx, :), 1, 'movmean', 5);
    end
    trajNew = smoothdataV3(trajNew, 1, 'movmean', 10); % 整体再平滑一次

    % 记录重组轨迹
    traj(iRmk).pos = trajNew; 
    [traj(iRmk).ori, ~] = trajOrientation(trajNew); 
    iRmk = iRmk + 1;
end

% save(p.handleRemakeTrajectory, 'traj')

