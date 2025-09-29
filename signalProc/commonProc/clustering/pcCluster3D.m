function clusterRslt3D = pcCluster3D(pc, varargin)
% 3D点云DBSCAN聚类
% 支持XYZ/XYV点云
% 输入: 
% 1. pc: 点云坐标
% 2. varargin:
%     - pcType: 点云类型. 'XYV'; 'XYZ'
%     - epsilon: DBSCAN的epsilon参数
%     - minpts: DBSCAN的minpts参数
%     - vel: 点云速度, 用于以颜色表示XYZ点云的速度, 在展示聚类结果时无作用
%     - pw: 点云反射强度, 输入有效值时, 以加权计算质心, 绘图时以点的大小表示反射强度
%     - limitX: 点云及绘图x坐标范围
%     - limitY: 点云及绘图y坐标范围
%     - limitZV: 点云及绘图z坐标范围
%     - drawEn: 是否绘图. 0-否; 1-是
% 输出: 
% clusterRslt3D: 3D点云聚类结果
% - .cluster: 簇
%    * .pc: 簇内点云坐标
%    * .centroid: 簇质心坐标
% - .noise: 离散点
%    * .pc: 离散点坐标
% - .pcInput: 输入的点云坐标
% - .clusterIdx: 簇序号
% 作者: 刘涵凯
% 更新: 2024-6-22

%% 无点云则直接退出
if isempty(pc)
    warning('无点云, 无法聚类');
    clusterRslt3D.cluster = struct('pc', [], 'centroid', []);
    clusterRslt3D.noise = struct('pc', []);
    clusterRslt3D.pcInput = [];
    clusterRslt3D.clusterIdx = [];
    return
end

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('pcType', 'XYZ');
p.addOptional('epsilon', []);
p.addOptional('minpts', 6);
p.addOptional('vel', []);
p.addOptional('pw', []);
p.addOptional('limitX', []);
p.addOptional('limitY', []);
p.addOptional('limitZV', []);
p.addOptional('drawEn', 1);
p.parse(varargin{:});
pcType = p.Results.pcType;
epsilon = p.Results.epsilon;
minpts = p.Results.minpts;
vel = p.Results.vel;
pw = p.Results.pw;
limitX = p.Results.limitX;
limitY = p.Results.limitY;
limitZV = p.Results.limitZV;
drawEn = p.Results.drawEn;
% 根据点云类型设置参数
switch pcType
    case 'XYZ'
        if isempty(epsilon)
            epsilon = 0.5;
        end
    case 'XYV'
        if isempty(epsilon)
            epsilon = 15;
        end
end

%% 删除超出边界的点云
if ~isempty(limitX)
    iDel = pc(:, 1) < limitX(1) | pc(:, 1) > limitX(2);
    if ~isempty(pw); pw(iDel) = []; end
    if ~isempty(vel); vel(iDel) = []; end
    pc(iDel, :) = [];
end
if ~isempty(limitY)
    iDel = pc(:, 2) < limitY(1) | pc(:, 2) > limitY(2);
    if ~isempty(pw); pw(iDel) = []; end
    if ~isempty(vel); vel(iDel) = []; end
    pc(iDel, :) = [];
end
if ~isempty(limitZV)
    iDel = pc(:, 3) < limitZV(1) | pc(:, 3) > limitZV(2);
    if ~isempty(pw); pw(iDel) = []; end
    if ~isempty(vel); vel(iDel) = []; end
    pc(iDel, :) = [];
end

%% 若所有点云都已删除, 则直接退出
if isempty(pc)
    warning('设定坐标范围内点云, 无法聚类');
    clusterRslt3D.cluster = struct('pc', [], 'centroid', []);
    clusterRslt3D.noise = struct('pc', []);
    clusterRslt3D.pcInput = [];
    clusterRslt3D.clusterIdx = [];
    return
end

%% DBSCAN聚类
clusterRslt3D.cluster = {};
clusterRslt3D.noise = {};
switch pcType
    case 'XYZ'
        clusterIdx = dbscan(pc, epsilon, minpts);
    case 'XYV'
        % 将绝对速度和坐标转化为bin索引后聚类
        load('config.mat', 'resR', 'resV')
        clusterIdx = dbscan(ceil([pc(:, 1) / resR, pc(:, 2) / resR, pc(:, 3) / resV]), epsilon, minpts);
end

%% 聚类结果处理
if max(clusterIdx) == -1
    warning('聚类失败')
    clusterRslt3D.cluster = struct('pc', [], 'centroid', []);
else
    for iCluster = 1 : max(clusterIdx)
        % 簇内点云坐标
        clusterRslt3D.cluster(iCluster).pc = pc(clusterIdx == iCluster, :);
        % 计算簇质心坐标
        if isempty(pw)
            clusterRslt3D.cluster(iCluster).centroid = mean(clusterRslt3D.cluster(iCluster).pc, 1);
        else
            clusterRslt3D.cluster(iCluster).centroid(1) = clusterRslt3D.cluster(iCluster).pc(:, 1)' * ...
                pw(clusterIdx == iCluster, :) / sum(pw(clusterIdx == iCluster, :));
            clusterRslt3D.cluster(iCluster).centroid(2) = clusterRslt3D.cluster(iCluster).pc(:, 2)' * ...
                pw(clusterIdx == iCluster, :) / sum(pw(clusterIdx == iCluster, :));
            clusterRslt3D.cluster(iCluster).centroid(3) = clusterRslt3D.cluster(iCluster).pc(:, 3)' * ...
                pw(clusterIdx == iCluster, :) / sum(pw(clusterIdx == iCluster, :));
        end
    end
end
% 离散点坐标
clusterRslt3D.noise.pc = pc(clusterIdx == -1, :);
% 输入的点云坐标
clusterRslt3D.pcInput = pc;
% 输入点云的聚类结果(簇编号)
clusterRslt3D.clusterIdx = clusterIdx;

%% 绘图
if drawEn; drawPc3D(pc, 'clusterIdx', clusterIdx, 'pcType', pcType, 'vel', vel, 'pw', pw, 'limitX', limitX, 'limitY', limitY, 'limitZV', limitZV); end
