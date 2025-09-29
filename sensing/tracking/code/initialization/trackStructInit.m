function trackStructInit
% 追踪初始化
% 主要是初始化常用变量及结构体
% 作者: 刘涵凯
% 更新: 2023-3-7

%% 参数对象及全局变量
p = trackParamShare.param;
global clusters trackCand trackConfirm trackLost trackWait trajectory ovlpRec sepRec multivRec assocRec

%% 常用变量
p.backtrackFlag = 0;                  % 轨迹回溯标志
p.sepDelFlag = 0;                      % 分离记录删除标志
p.identLockFrm = 0;                  % 身份识别锁定帧
p.nPpl = 0;                                 % 人数
p.nMultiv = 1;                            % 平行宇宙数
p.anchorFrmMultiv = 0;             % 平行宇宙锚点帧

%% 轨迹区
% 候选区
trackCand = struct('centroid', [], 'kalmanFilter', [], 'presence', [], 'ghostLabel', [], 'age', [], 'trajectory', [], 'frame', []);
% 确立区
trackConfirm = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], 'name', [], 'pc', [], ...
    'status', [], 'statusAge', [], 'trajectory', [], 'frame', []);
% 丢失区
trackLost = struct('centroid', [], 'iPeople', [], 'name', [], 'trajectory', [], 'frame', []);
% 等待区
trackWait = struct('centroid', [], 'kalmanFilter', [], 'iPeople', [], 'name', [], 'age', []);

for iFrm = 1 : p.nFrmLoad
    % 轨迹记录. 其"track"没有实际意义, 只是为了便于结构体处理
    trajectory(iFrm).track = struct('iPeople', [], 'name', [], 'trajectory', [], 'frame', [], 'status', [], ...
        'pcLast', [], 'kalmanFilter', [], 'statusAge', []);
    % 轨迹重叠记录. 其"ovlp"没有实际意义, 只是为了便于结构体处理
    ovlpRec(iFrm).ovlp = struct('idxSet', []);
    % 轨迹分离记录. 其"sep"没有实际意义, 只是为了便于结构体处理
    sepRec(iFrm).sep = struct('idxSet', [], 'nSeperate', []);
    % 轨迹平行宇宙记录. 其"multiv"没有实际意义, 只是为了便于结构体处理
    % association属性中, 第一列为主宇宙中的ID, 第二列为平行宇宙中的ID
    multivRec(iFrm).multiv = struct('iMultiverse', 1, 'track', struct('iPeople', [], 'name', [], ...
        'trajectory', [], 'frame', [], 'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []), ...
        'brother', [], 'parent', [], 'association', []);
end

%% iPeople关联表记录
assocRec = struct('frame', 1, 'association', 1 : 10); % 应该不会超过10人

%% 第一帧直接将点云簇存入候选区
for iCluster = 1 : structLength(clusters(p.iFrmLoad(1)).cluster, 'centroid')
    clusters(p.iFrmLoad(1)) = ghostLabeling(clusters(p.iFrmLoad(1))); % 鬼影标记
    trackCand(iCluster) = struct('centroid', clusters(p.iFrmLoad(1)).cluster(iCluster).centroid, ...
        'kalmanFilter', createNewKF(clusters(p.iFrmLoad(1)).cluster(iCluster).centroid, 'motionType', p.motionType), ...
        'presence', 1, ...
        'ghostLabel', clusters(p.iFrmLoad(1)).cluster(iCluster).ghostLabel, ...
        'age', 1, ...
        'trajectory', clusters(p.iFrmLoad(1)).cluster(iCluster).centroid, 'frame', p.iFrmLoad(1));
end
