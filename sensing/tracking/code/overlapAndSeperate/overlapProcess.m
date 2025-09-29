function [clusterOut, assocRsltOut] = overlapProcess(clusterIn, assocRsltIn)
% 对确立区-点云簇关联的剩余轨迹进行点云重叠判断与处理
% 输入: 
% 1. frame: 帧索引
% 2. assocRsltIn: 输入的关联结果
%     - .cost: 航迹关联cost
%     - .assignments: 关联成功的连线
%     - .unassignedTracks: 关联失败的轨迹
%     - .unassignedDetections: 关联失败的点云簇
% 3. clusterIn: 输入聚类结果
%     - .pc: 簇内点云坐标
%     - .centroid: 簇质心坐标
%     - .ghostLabel: 鬼影标记
% 输出: 
% 1. clusterOut: 同输入, 更新后的聚类结果
% 2. assocRsltOut: 同输入, 重新关联的结果
% 作者: 刘涵凯
% 更新: 2023-3-9

%% 参数对象及全局变量
p = trackParamShare.param;
global trackConfirm

%% 点云重叠判断
ovlpRec = overlapJudge(clusterIn, assocRsltIn);
nOvlp = structLength(ovlpRec, 'idxSet');
% 将trackConfirm的各iPeople组成序列, 用于寻找点云簇索引
iPplOfConfirm = vertcat(trackConfirm.iPeople);

%% GMM点云重叠处理
clusterOut = clusterIn; % 初始化聚类结果
clusterIdx = 1 : length(clusterIn.ghostLabel); % 初始化聚类结果索引
for iOvlp = 1 : nOvlp
    % 重叠轨迹的iPeople
    idxSet = ovlpRec(iOvlp).idxSet;
    nPplOvlp = length(idxSet);
    % 重叠轨迹在trackConfirm中的索引
    iTrackOvlp = find(ismember(iPplOfConfirm, idxSet));
    % 重叠点云簇在clusterIn中的索引
    iClusterOvlp = assocRsltIn.assignments(ismember(assocRsltIn.assignments(:, 1), iTrackOvlp), 2);
    % 提取重叠点云
    pcOvlp = cell2mat(clusterIn.pc((iClusterOvlp)));
    % 原iClusterOvlp簇在当前clusterOut中的索引
    iClusterOvlpNew = clusterIdx(iClusterOvlp);
    % 以重叠轨迹的坐标为初始位置
    initPos = vertcat(trackConfirm(iTrackOvlp).centroid);
    % GMM聚类
    [clusterOut, clusterIdx] = gmmOverlapProcess(clusterOut, clusterIdx, iClusterOvlpNew, pcOvlp, nPplOvlp, initPos);
end

%% 重新关联
assocRsltOut = trackAssociation(trackConfirm, vertcat(clusterOut.centroid), clusterOut.ghostLabel, p.costConfirm);
