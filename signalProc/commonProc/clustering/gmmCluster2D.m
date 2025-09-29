function clusters = gmmCluster2D(pc, nCluster, varargin)
% 2D点云DBSCAN聚类
% 支持笛卡尔坐标点云和RD点云
% 输入: 
% 1. pc: 点云坐标
% 2. nCluster: 聚类簇数
% 3. varargin:
%     - gmmInitial: GMM聚类初始参数 % https://ww2.mathworks.cn/help/stats/fitgmdist.html#namevaluepairarguments
%     - pw: 点云反射强度, 输入有效值时, 以加权计算质心
% 输出: 
% clusters: 2D点云聚类结果
% - .pc: 簇内点云
% - .centroid: 簇质心坐标
% 作者: 刘涵凯
% 更新: 2022-7-19

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('gmmInitial', []);
p.addOptional('pw', []);
p.parse(varargin{:});
gmmInitial = p.Results.gmmInitial;
pw = p.Results.pw;

%% GMM聚类
if ~isempty(gmmInitial)
    gmFit = fitgmdist(pc, nCluster, 'RegularizationValue', 0.0001, 'Start', gmmInitial);
else
    gmFit = fitgmdist(pc, nCluster, 'RegularizationValue', 0.0001);
end
clusterIdx = cluster(gmFit, pc); % 簇索引

for iCluster = 1 : nCluster
    idxPc = find(clusterIdx == iCluster);
    if isempty(idxPc)
        % 若没能聚类出该子簇, 则将全部点云导入该子簇
        idxPc = find(clusterIdx);
    end
    % 簇内点云坐标
    pcTemp = pc(idxPc, :);
    clusters.pc(iCluster) = {pcTemp};
    % 簇内点云坐标
    pcTemp = pc(idxPc, :);
    clusters.pc(iCluster) = {pcTemp};
    % 计算簇质心位置
    if isempty(pw)
        clusters.centroid(iCluster, :) = mean(pcTemp, 1);
    else
        clusters.centroid(iCluster, 1) = pcTemp(:, 1)' * pw(idxPc, :) / sum(pw(idxPc, :));
        clusters.centroid(iCluster, 2) = pcTemp(:, 2)' * pw(idxPc, :) / sum(pw(idxPc, :));
    end
end
clusters.pc = clusters.pc';
