% 多目标追踪
% 含IMU身份识别辅助
% 作者: 刘涵凯
% 更新: 2023-3-9

clear; close all
config2243

for iSetup = [100] % 场景序号
    for iCount = [1] % 次数序号
        %% 清空工作区
        clearvars -except iSetup iCount traj; clear global; close all
        disp([num2str(iSetup), '-', num2str(iCount), '开始'])

        %% 参数对象及全局变量
        trackPathInit(iSetup, iCount); % 设置文件路径
        p = trackParamConfig; % 设置参数
        global clusters iFrm trackCand trackConfirm trajectory sepRec

        %% 点云生成及聚类
        pcMode = 'load'; % 'generate' 'load'
        clusters = clusterGeneration(pcMode);
        %     end
        % end

        %% 追踪
        trackStructInit % 初始化
        for iFrm = 2 : p.nFrmLoad
            disp(['追踪: 第', num2str(p.iFrmLoad(iFrm)), '帧']);

            % KF预测
            if strcmp(p.trackAlgo, 'KF'); kfPredict; end

            % 鬼影标记
            clusters(p.iFrmLoad(iFrm)) = ghostLabeling(clusters(p.iFrmLoad(iFrm)));

            % 提取新一帧聚类结果
            clusterNew1 = struct('centroid', vertcat(clusters(iFrm).cluster.centroid), ... % 该帧的所有点云簇的中心坐标
                'ghostLabel', vertcat(clusters(iFrm).cluster.ghostLabel), ... % 该帧的所有点云簇的中心坐标
                'pc', {{clusters(iFrm).cluster.pc}'}); % clusterNew1.centroid中各坐标在clusters(iFrm).cluster中的索引

            % 细粒度聚类
            if p.fgClusterEn; clusterNew1 = clusterFinegrained(clusterNew1); end

            % 等待区处理
            if p.waitZoneEn; waitZoneProcess(clusterNew1); end

            % 关联1: 点云簇与确立区
            assocRslt1 = trackAssociation(trackConfirm, vertcat(clusterNew1.centroid), ...
                clusterNew1.ghostLabel, p.costConfirm);

            % 根据关联1的结果更新确立区
            [clusterNew1, assocRslt1] =  confirmZoneProcess(clusterNew1, assocRslt1);

            % 若关联1有剩余点云簇
            if ~isempty(assocRslt1.unassignedDetections)

                % 提取剩余点云簇
                iUnassign = assocRslt1.unassignedDetections;
                clusterNew2 = struct('centroid', clusterNew1.centroid(iUnassign, :), 'ghostLabel', ...
                    clusterNew1.ghostLabel(iUnassign), 'pc', {clusterNew1.pc(iUnassign)});

                % 关联2: 点云簇与候选区
                assocRslt2 = trackAssociation(trackCand, vertcat(clusterNew2.centroid), ...
                    clusterNew2.ghostLabel, p.costCand);

                % 根据关联2的结果更新候选区和确立区
                candidateZoneProcess(clusterNew2, assocRslt2);

            else % 若关联1无剩余点云簇

                % 候选区更新
                renewCandidate(0, []);

            end

            % 轨迹更新
            renewTrajectory

            % 更新轨迹分离记录
            renewSeperateRecord(iFrm, p.backtrackFlag);

            % 更新平行宇宙
            renewMultiverse(trajectory(iFrm).track, iFrm, sepRec(iFrm).sep, p.backtrackFlag, p.sepDelFlag);

            % 每identIntvl帧进行一次身份识别
            % if p.identifyEn && mod(iFrm, p.identIntvl) == 0; identification; end

        end

        %% 轨迹补全
        if p.paddingEn; padTrajectory; end
 
        %% 保存结果
        save(p.handleTrkRslt, '-append', 'clusters', 'trajectory')
        disp([num2str(iSetup), '-', num2str(iCount), '完成'])

        %% 绘图
        drawTracking
    end
end
