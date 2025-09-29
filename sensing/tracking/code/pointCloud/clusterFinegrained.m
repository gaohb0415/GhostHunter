function clusterOut = clusterFinegrained(clusterIn)
% 对可能存在人员重叠的点云簇进行细粒度聚类
% 输入: 
% clusterIn: 输入聚类结果
% - .pc: 簇内点云坐标
% - .centroid: 簇质心坐标
% - .ghostLabel: 鬼影标记
% 输出: 
% clusterOut: 同输入, 更新后的聚类结果
% 作者: 刘涵凯
% 更新: 2023-3-9

%% 参数对象
p = trackParamShare.param;

%% 细粒度聚类
clusterOut = clusterIn; % 初始化聚类结果
clusterIdx = 1 : length(clusterIn.ghostLabel); % 初始化聚类结果索引
for iCluster = 1 : length(clusterIn.ghostLabel)
    pcOvlp = cell2mat(clusterIn.pc(iCluster));
    % 凸包计算
    bound = convhull(pcOvlp);
    bound(end) = [];
    % 对凸包顶点排列组合
    combo = nchoosek(bound, 2);
    % 计算距离最大值, 即簇内点云间最大距离
    d = sqrt((pcOvlp(combo(:, 1), 1) - pcOvlp(combo(:, 2), 1)) .^ 2 + ...
        (pcOvlp(combo(:, 1), 2) - pcOvlp(combo(:, 2), 2)) .^ 2);
    [dMax, idxMax] = max(d);
    if dMax > p.persWidthTh
        % 重叠人数
        nPplOvlp = ceil(dMax / p.persWidth);
        % 原iClusterOvlp簇在当前clusterOut中的索引
        iClusterOvlpNew = clusterIdx(iCluster);
        % 将最大距离连线等分, 作为初始位置
        vertex1 = pcOvlp(combo(idxMax, 1), :);
        vertex2 = pcOvlp(combo(idxMax, 2), :);
        xLinspace = linspace(vertex1(1), vertex2(1), nPplOvlp + 2);
        yLinspace = linspace(vertex1(2), vertex2(2), nPplOvlp + 2);
        initPos = [xLinspace(2 : end - 1)', yLinspace(2 : end - 1)'];
        % GMM聚类
        [clusterOut, clusterIdx] = gmmOverlapProcess(clusterOut, clusterIdx, iClusterOvlpNew, pcOvlp, nPplOvlp, initPos);
    end
end
