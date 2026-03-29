%{

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

% 计算一下刚才DBSSCAN一共聚合了多少个目标簇
% 追踪逻辑中：
% 1代表 Ghost 鬼影目标
% 0代表 Real_Target 真实目标
nCluster = structLength(clusters.cluster, 'centroid');
if ~nCluster
    % 若无点云簇, 则仅增加ghostLabel属性
    clusters.cluster.ghostLabel = [];
else
    % 默认初始化的时候先都认为所有的点云簇都是真实目标
    % 将点云簇的ghostLabel初始化为0
    for iCluster = 1 : nCluster
        clusters.cluster(iCluster).ghostLabel = 0;
    end
end
%}

function clusters = ghostInit(clusters)
% 初始化鬼影标记

nCluster = structLength(clusters.cluster, 'centroid');

if nCluster == 0
    % 空簇时，直接重建为带 ghostLabel 字段的空结构体
    clusters.cluster = struct( ...
        'pc', [], ...
        'centroid', [], ...
        'velocity', [], ...
        'ghostLabel', []);
else
    for iCluster = 1 : nCluster
        clusters.cluster(iCluster).ghostLabel = 0;
    end
end
end