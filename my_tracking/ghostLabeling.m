function clusters = ghostLabeling(clusters)
% 鬼影标记
% 输入: 
% clusters: 聚类结果
% - .cluster: 簇
%    * .pc: 簇内点云坐标
%    * .centroid: 簇质心坐标
%    * .ghostLabel: 鬼影标记
% - .noise: 离散点
%    * .pc: 离散点坐标
% - .pcInput: 输入的点云坐标
% - .pw: 各点云的强度
% - .clusterIdx: 簇序号
% 输出: 
% clusters: 同输入
% 作者: 刘涵凯
% 更新: 2023-3-8

%% 参数对象
p = trackParamShare.param;
if ~p.ghostSupprEn; return; end

%% 各簇相对于雷达的距离和角度
nCluster = structLength(clusters.cluster, 'centroid');
d = zeros(nCluster, 1);
az = zeros(nCluster, 1);
for iCluster = 1 : nCluster
    d(iCluster) = norm(clusters.cluster(iCluster).centroid);
    az(iCluster) = acotd(clusters.cluster(iCluster).centroid(2) / clusters.cluster(iCluster).centroid(1));
end

%% 鬼影判定
for iCluster = 1 : nCluster
    % 待判定簇中全部点相对于雷达的角度
    azAllPc = acotd(clusters.cluster(iCluster).pc(:, 2) ./ clusters.cluster(iCluster).pc(:, 1));
    % 最左/最右角度
    azLeft = min(azAllPc);
    azRight = max(azAllPc);
    % 与待判定簇相比, 距雷达更近的簇的索引
    idxFront = find(d < d(iCluster));

    for iFront = 1 : length(idxFront)
        % 该居前簇中全部点相对于雷达的角度
        azAllPcFront = acotd(clusters.cluster(idxFront(iFront)).pc(:, 2) ./ ...
            clusters.cluster(idxFront(iFront)).pc(:, 1));
        % 角度宽度
        azWidth = max(azAllPcFront) - min(azAllPcFront);
        % 待判定簇鬼影判定的左右角度阈值
        % 由居前簇的角度范围、保护角度、保护因子决定
        % 例如, 左阈值=最小角度-(保护角度+保护因子*角度宽度)
        azThLeft = min(azAllPcFront) - azWidth * p.guardFactor - p.guardAz;
        azThRight = max(azAllPcFront) + azWidth * p.guardFactor + p.guardAz;

        % 以居前簇为顶点, 待判定簇偏移居前簇-雷达连线的角度
        difPos = clusters.cluster(iCluster).centroid - clusters.cluster(idxFront(iFront)).centroid;
        azRltv = acotd(difPos(2) / difPos(1));
        azShift = abs(azRltv - az(idxFront(iFront)));
        % 偏移角度阈值
        if azWidth < p.azFrontWidthTh
            % 若居前簇角度宽度较窄, 则设定较大的偏移角度阈值下限
            azShiftThMin = p.azShiftTh(2);
        else
            azShiftThMin = p.azShiftTh(3);
        end

        if azShift < p.azShiftTh(1) && ...
                (azShift < azShiftThMin || ...
                (azLeft > azThLeft && azLeft < azThRight) || ...
                (azRight > azThLeft && azRight < azThRight))
            % 判定为鬼影的条件: 
            % 1. 必须满足偏移角度<偏移角度阈值上限
            % 2. 其次, 满足"偏移角度<偏移角度阈值下限"或"待判定簇点云在居前簇点云角度范围内"
            clusters.cluster(iCluster).ghostLabel = 1;
            break % 结束此待判定簇的判定
        end
    end
end
