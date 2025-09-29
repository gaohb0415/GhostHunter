function [cluster, assocRslt] = confirmZoneProcess(cluster, assocRslt)
% 确立区处理
% 输入:
% 1. cluster: 聚类结果
%     - .pc: 簇内点云坐标
%     - .centroid: 簇质心坐标
%     - .ghostLabel: 鬼影标记
% 2. assocRslt: 关联结果
%     - .cost: 航迹关联cost
%     - .assignments: 关联成功的连线
%     - .unassignedTracks: 关联失败的轨迹
%     - .unassignedDetections: 关联失败的点云簇
% 输入:
% 1. cluster: 同输入, 更新后的聚类结果
% 2. assocRslt: 同输入, 更新后的关联结果
% 作者: 刘涵凯
% 更新: 2023-3-12

%% 参数对象及全局变量
p = trackParamShare.param;
global iFrm trackConfirm

%% 静态目标增强
if p.staticEnhEn && ~isempty(assocRslt.unassignedTracks)
    % 若存在剩余轨迹
    if mod(iFrm, p.staticEnhIntvl) == 0
        % 点云生成较为耗时, 所以并非每帧都进行静态目标增强
        [cluster, assocRslt] = staticalEnhance(trackConfirm, assocRslt, p.costConfirm);
    end
end

%% 若重叠区轨迹跳跃到鬼影嫌疑簇, 则否决此关联
if ~isempty(assocRslt.assignments)
    iDel = [];
    for iAssign = 1 : size(assocRslt.assignments, 1)
        idxTrack = assocRslt.assignments(iAssign, 1);
        idxDet = assocRslt.assignments(iAssign, 2);
        if strcmp(trackConfirm(idxTrack).status, "overlap")
            if cluster.ghostLabel(idxDet)
                if assocRslt.cost(idxTrack, idxDet) > p.costConfirm
                    % 以costConfirm为跳跃阈值
                    iDel = [iDel; iAssign];
                    assocRslt.unassignedTracks = [assocRslt.unassignedTracks; idxTrack];
                    assocRslt.unassignedDetections = [assocRslt.unassignedDetections; idxDet];
                end
            end
        end
    end
    assocRslt.assignments(iDel, :) = [];
end

%% 点云重叠判断及处理
if p.ovlpProcEn && ~isempty(assocRslt.unassignedTracks) && ~isempty(assocRslt.assignments)
    [cluster, assocRslt] = overlapProcess(cluster, assocRslt);
end

%% 更新确立区
renewConfirm(cluster, assocRslt);
