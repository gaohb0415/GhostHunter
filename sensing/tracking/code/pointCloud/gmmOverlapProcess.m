function [cluster, clusterIdx] = gmmOverlapProcess(cluster, clusterIdx, iClusterOvlp, pcOvlp, nPplOvlp, initPos)
% 通过GMM聚类对重叠点云进行分离
% 输入: 
% 1. cluster: 聚类结果
%     - .pc: 簇内点云坐标
%     - .centroid: 簇质心坐标
%     - .ghostLabel: 鬼影标记
% 2. clusterIdx: 聚类结果索引
% 3. iClusterOvlp: 重叠簇在cluster中的索引
% 4. pcOvlp: 重叠点云
% 5. nPplOvlp: 重叠人数
% 6. initPos: 初始位置
% 输出: 
% 1. clusters: 同输入, 更新后的聚类结果
% 2. clusterIdx: 同输入, 更新后的聚类结果索引
% 作者: 刘涵凯
% 更新: 2023-3-9

%% GMM聚类
% 自定义初始参数
sigma = zeros(2, 2, nPplOvlp);
for iPeopleOvlp = 1 : nPplOvlp
    % 这里我也不知道为什么这么设置
    % https://ww2.mathworks.cn/help/stats/fitgmdist.html#namevaluepairarguments
    sigma(:, :, iPeopleOvlp) = iPeopleOvlp * [1 1; 1 2];
end
pComp = ones(1, nPplOvlp) / nPplOvlp;
gmmInit = struct('mu', initPos, 'Sigma', sigma, 'ComponentProportion', pComp);
%聚类
clusterGmm = gmmCluster2D(pcOvlp, nPplOvlp, 'gmmInitial', gmmInit);

%% 更新簇及索引
cluster = struct('centroid', vertcat(cluster.centroid(1 : iClusterOvlp - 1, :), ...
    clusterGmm.centroid, cluster.centroid(iClusterOvlp + 1 : end, :)), ...
    'ghostLabel', vertcat(cluster.ghostLabel(1 : iClusterOvlp - 1), ...
    zeros(nPplOvlp, 1), cluster.ghostLabel(iClusterOvlp + 1 : end)), ... % 将新簇鬼影标记初始化为0
    'pc', {vertcat(cluster.pc(1 : iClusterOvlp - 1), ...
    clusterGmm.pc, cluster.pc(iClusterOvlp + 1 : end))});
clusterIdx(iClusterOvlp + 1 : end) = clusterIdx(iClusterOvlp + 1 : end) + nPplOvlp - 1;
