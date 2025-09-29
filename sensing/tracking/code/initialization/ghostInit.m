function clusters = ghostInit(clusters)
% 初始化鬼影标记
% 输入: 
% clusters: 聚类结果
% - .cluster: 簇
%    * .centroid: 簇质心坐标
%    * 省略其他属性
% - 省略其他属性
% 输出: 
% clusters: 同输入, 但增加以下属性
% - .cluster:
%    * .ghostLabel: 鬼影标记
% 作者: 刘涵凯
% 更新: 2023-3-9

nCluster = structLength(clusters.cluster, 'centroid');
if ~nCluster
    % 若无点云簇, 则仅增加ghostLabel属性
    clusters.cluster.ghostLabel = [];
else
    % 将点云簇的ghostLabel初始化为0
    for iCluster = 1 : nCluster
        clusters.cluster(iCluster).ghostLabel = 0;
    end
end
