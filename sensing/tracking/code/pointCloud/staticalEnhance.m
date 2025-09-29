function [clusterNew, assocRsltOut] = staticalEnhance(tracks, assocRsltIn, costTh)
% 静态目标增强
% 注意frame属性在本函数中部更新, 而是更新于renewTrajectory
% 因为关联失败的轨迹也会有卡尔曼预测, 所以关联成功与否都会进行帧更新
% 输入:
% 1. tracks: 轨迹区
% 2. assocRsltIn: 输入的关联结果
%     - .cost: 航迹关联cost
%     - .assignments: 关联成功的连线
%     - .unassignedTracks: 关联失败的轨迹
%     - .unassignedDetections: 关联失败的点云簇
% 3. costTh: 航迹关联cost阈值
% 输出:
% 1. clusterNew 二次聚类结果
%     - .pc: 簇内点云坐标
%     - .centroid: 簇质心坐标
%     - .ghostLabel: 鬼影标记
% 2. assocRsltOut: 同输入, 重新关联的关联结果
% 作者: 刘涵凯
% 更新: 2023-3-10

%% 参数对象及全局变量
p = trackParamShare.param;
global clusters iFrm

%% 提取原始点云
pcAll = clusters(p.iFrmLoad(iFrm)).pcInput;
pcPw = clusters(p.iFrmLoad(iFrm)).pw;

%% 二次点云生成及聚类
nUnassign = length(assocRsltIn.unassignedTracks);
if nUnassign
    radarData= readBin(p.iFrmLoad(iFrm), 0, 'staticRmvEn', 0); % 提取该帧信号, 不进行静态滤波
    [fftRsltRg, ~] = fftRange(radarData); % Range FFT
    % 载入雷达和CFAR参数
    load('config.mat', 'resR', 'cfarParamRA')
    for iUnassign = 1 : nUnassign
        if tracks(assocRsltIn.unassignedTracks(iUnassign)).statusAge > p.staticEnhanceTh
            % 关联失败的轨迹的坐标、距离、角度
            pos = tracks(assocRsltIn.unassignedTracks(iUnassign)).centroid;
            if pos(1) < p.limitX(1) || pos(1) > p.limitX(2) || pos(2) < p.limitY(1) || pos(2) > p.limitY(2)
                % 由于KF预测, 轨迹坐标有概率出现在设定范围外, 若发生, 则不对此轨迹执行静态目标增强
                continue
            end
            rg = norm(pos);
            ang = round(acotd(pos(2) / pos(1)));
            % 构建二次点云生成范围
            nRg1Side = cfarParamRA.guard(1) + cfarParamRA.train(1);
            nAng1Side = cfarParamRA.guard(2) + cfarParamRA.train(2);
            limitR = [rg - p.bodyRgWidth / 2  - resR * nRg1Side, rg + p.bodyRgWidth / 2  + resR * nRg1Side];
            limitAng = [ang - p.bodyAngWidth / 2 - nAng1Side, ang + p.bodyAngWidth / 2 + nAng1Side];
            % XY点云二次生成
            [~, pcRA] = dbfProc1D(fftRsltRg, 'limitR', limitR,  'limitAng', limitAng, 'pcEn', 1, 'cfarPfa', 0.05);
            % 将二次生成点云加入总点云中
            pcAll = [pcAll; [pcRA.x, pcRA.y]];
            pcPw = [pcPw; pcRA.power];
        end
    end
end
% 删除重复点云
[pcAll, uniqIdx, ~] = unique(pcAll, 'rows');
pcPw = pcPw(uniqIdx);
% 二次聚类
clustersReborn = pcCluster2D(pcAll, 'pw', pcPw, 'epsilon', p.epsilonStatic, 'minpts', p.minptsStatic, 'limitX', p.limitX, 'limitY', p.limitY); % XYV点云聚类
% 鬼影判定
clustersReborn = ghostLabeling(ghostInit(clustersReborn));
% 重新构建聚类结果
clusterNew = struct('centroid', vertcat(clustersReborn.cluster.centroid), ... % 该帧的所有点云簇的中心坐标
    'ghostLabel', vertcat(clustersReborn.cluster.ghostLabel), ... % 该帧的所有点云簇的中心坐标
    'pc', {{clustersReborn.cluster.pc}'}); % clusterNew.centroid中各坐标在clustersReborn.cluster中的索引

%% 重新关联
assocRsltOut = trackAssociation(tracks, vertcat(clusterNew.centroid), clusterNew.ghostLabel, costTh);
