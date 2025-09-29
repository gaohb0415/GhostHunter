function clusterRslt2D = pcCluster2D(pc, varargin)
% 2D点云DBSCAN聚类
% 支持XY/VY点云
% 输入:
% 1. pc: 点云坐标
% 2. varargin:
%     - pcType: 点云类型. 'XY'; 'VY'
%     - epsilon: DBSCAN的epsilon参数
%     - minpts: DBSCAN的minpts参数
%     - pw: 点云反射强度, 输入有效值时, 以加权计算质心, 绘图时以点的大小表示反射强度
%     - limitXV: 点云及绘图x/v坐标范围
%     - limitY: 点云及绘图y坐标范围
%     - drawEn: 是否绘图. 0-否; 1-是
% 输出:
% clusterRslt2D: 2D点云聚类结果
% - .cluster: 簇
%    * .pc: 簇内点云坐标
%    * .centroid: 簇质心坐标
% - .noise: 离散点
%    * .pc: 离散点坐标
% - .pcInput: 输入的点云坐标
% - .pw: 各点云的强度
% - .clusterIdx: 簇序号
% 作者: 刘涵凯
% 更新: 2024-6-22

%% 无点云则直接退出
if isempty(pc)
    warning('无点云, 无法聚类');
    clusterRslt2D.cluster = struct('pc', [], 'centroid', []);
    clusterRslt2D.noise = struct('pc', []);
    clusterRslt2D.pcInput = [];
    clusterRslt2D.pw = [];
    clusterRslt2D.clusterIdx = [];
    return
end

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('pcType', 'XY');
p.addOptional('epsilon', []);
p.addOptional('minpts', 6);
p.addOptional('pw', []);
p.addOptional('limitXV', []);
p.addOptional('limitY', []);
p.addOptional('drawEn', 0);
p.parse(varargin{:});
pcType = p.Results.pcType;
epsilon = p.Results.epsilon;
minpts = p.Results.minpts;
pw = p.Results.pw;
limitXV = p.Results.limitXV;
limitY = p.Results.limitY;
drawEn = p.Results.drawEn;
% 根据点云类型设置参数
switch pcType
    case 'XY'
        if isempty(epsilon); epsilon = 0.25; end
    case 'VY'
        if isempty(epsilon); epsilon = 10; end
end

%% 删除超出边界的点云
if ~isempty(limitXV)
    if ~isempty(pw); pw(pc(:, 1) < limitXV(1) | pc(:, 1) > limitXV(2)) = []; end
    pc(pc(:, 1) < limitXV(1) | pc(:, 1) > limitXV(2), :) = [];
end
if ~isempty(limitY)
    if ~isempty(pw); pw(pc(:, 2) < limitY(1) | pc(:, 2) > limitY(2)) = []; end
    pc(pc(:, 2) < limitY(1) | pc(:, 2) > limitY(2), :) = [];
end

%% 若所有点云都已删除, 则直接退出
if isempty(pc)
    warning('设定坐标范围内点云, 无法聚类');
    clusterRslt2D.cluster = struct('pc', [], 'centroid', []);
    clusterRslt2D.noise = struct('pc', []);
    clusterRslt2D.pcInput = [];
    clusterRslt2D.pw = [];
    clusterRslt2D.clusterIdx = [];
    return
end

%% DBSCAN聚类
clusterRslt2D.cluster = {};
clusterRslt2D.noise = {};
switch pcType
    case 'XY'
        clusterIdx = dbscan(pc, epsilon, minpts);
    case 'VY'
        % 将绝对速度和距离转化为bin索引后聚类
        load('config.mat', 'resR', 'resV')
        clusterIdx = dbscan(ceil([pc(:, 1) / resV, pc(:, 2) / resR]), epsilon, minpts);
end

%% 聚类结果处理
if max(clusterIdx) == -1
    warning('聚类失败')
    clusterRslt2D.cluster = struct('pc', [], 'centroid', []);
else
    for iCluster = 1 : max(clusterIdx)
        % 簇内点云坐标
        clusterRslt2D.cluster(iCluster).pc = pc(clusterIdx == iCluster, :);
        % 计算簇质心坐标
        if isempty(pw)
            clusterRslt2D.cluster(iCluster).centroid = mean(clusterRslt2D.cluster(iCluster).pc, 1);
        else
            clusterRslt2D.cluster(iCluster).centroid(1) = clusterRslt2D.cluster(iCluster).pc(:, 1)' * ...
                pw(clusterIdx == iCluster) / sum(pw(clusterIdx == iCluster));
            clusterRslt2D.cluster(iCluster).centroid(2) = clusterRslt2D.cluster(iCluster).pc(:, 2)' * ...
                pw(clusterIdx == iCluster) / sum(pw(clusterIdx == iCluster));
        end
    end
end
% 离散点坐标
clusterRslt2D.noise.pc = pc(clusterIdx == -1, :);
% 输入的点云坐标
clusterRslt2D.pcInput = pc;
% 各点云的强度
clusterRslt2D.pw = pw;
% 输入点云的聚类结果(簇编号)
clusterRslt2D.clusterIdx = clusterIdx;

%% 绘图
if drawEn; drawPc2D(pc, 'clusterIdx', clusterIdx, 'pcType', pcType, 'pw', pw, 'limitXV', limitXV, 'limitY', limitY); end
