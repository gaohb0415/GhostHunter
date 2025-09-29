function waitZoneProcess(cluster)
% 等待区处理
% 输入:
% cluster: 聚类结果
%  - .pc: 簇内点云坐标
%  - .centroid: 簇质心坐标
%  - .ghostLabel: 鬼影标记
% 作者: 刘涵凯
% 更新: 2023-3-14

%% 参数对象及全局变量
p = trackParamShare.param;
global iFrm trackConfirm trackWait ovlpRec

%% 用原本的确立区进行一次关联, 用于判断是否接受等待区轨迹
assocRslt1 = trackAssociation(trackConfirm, vertcat(cluster.centroid), cluster.ghostLabel, p.costConfirm);

%% 将等待区轨迹替换入确立区, 再进行一次航迹关联
trackConfirmCopy = trackConfirm;
idxInConfirm = []; % 等待区轨迹在确立区中的索引
for iTrack = 1 : structLength(trackWait, 'iPeople')
    idxTrack = find(arrayfun(@(x) ismember(trackWait(iTrack).iPeople, x.iPeople), trackConfirmCopy));
    trackConfirmCopy(idxTrack).centroid = trackWait(iTrack).centroid;
    idxInConfirm = [idxInConfirm, idxTrack];
end
assocRslt2 = trackAssociation(trackConfirmCopy, vertcat(cluster.centroid), cluster.ghostLabel, p.costConfirm);
% 对关联失败的等待区轨迹进行静态目标增强
if p.staticEnhEn && sum(ismember(assocRslt2.unassignedTracks, idxInConfirm)) && mod(iFrm, p.staticEnhIntvl) == 0
    [cluster, assocRslt2] = staticalEnhance(trackConfirmCopy, assocRslt2, p.costCand); % 这里使用比costConfirm小的costCand
    % 静态目标增强后点云簇(可能)发生变化, 所以重新执行关联1
    assocRslt1 = trackAssociation(trackConfirm, vertcat(cluster.centroid), cluster.ghostLabel, p.costConfirm);
end

%% 用满足条件的等待区轨迹覆盖确立区相应轨迹
iWait2Confirm = [];
for iTrack = 1 : structLength(trackWait, 'iPeople')
    % 该轨迹在确立区中的索引
    idxTrack = idxInConfirm(iTrack);
    % 关联2中该轨迹的关联结果索引
    idxAssign = find(ismember(assocRslt2.assignments(:, 1), idxTrack));
    if idxAssign
        % 若关联2关联成功(即等待区轨迹关联成功)
        idxDet = assocRslt2.assignments(idxAssign, 2);
        % if ~cluster.ghostLabel(idxDet)
        % 这里原本想要加入与鬼影标记有关的条件, 加了之后发现效果不理想, 遂暂时作罢
        if ~cluster.ghostLabel(idxDet) && ismember(idxDet, assocRslt1.unassignedDetections)
            % 若关联到的簇未在正常的确立区关联(关联1)中关联成功
            %% 第三次关联: 从正常确立区中删除该轨迹后, 关联1中该轨迹对应的簇(若存在)是否与其他轨迹关联
            % 23.4.18注: 该逻辑在处理鬼影造成的偏航时应该是有问题的, 所以先删除
            idxRslt1Assign = find(assocRslt1.assignments(:, 1) == idxTrack);
            % 若关联1中该轨迹关联成功
            if ~isempty(idxRslt1Assign)
                % 该轨迹在关联1中的对应的簇的索引
                idxDetConfirm = assocRslt1.assignments(idxRslt1Assign, 2);
                % 删除该轨迹
                trackConfirmTemp = structRowDelete(trackConfirm, idxTrack);
                % 执行关联3
                assocRslt3 = trackAssociation(trackConfirmTemp, vertcat(cluster.centroid), cluster.ghostLabel, p.costConfirm);
                if ismember(idxDetConfirm, assocRslt3.unassignedDetections)
                    % 若该簇在关联3中关联失败, 则终止此次等待区覆盖
                    continue
                end
            end
            % 执行到了这里, 说明可以将等待区轨迹覆盖到确立区了
            iWait2Confirm = [iWait2Confirm, iTrack];
            %% 轨迹覆盖
            % 下面应该可以暂时不改statusAge
            % 注意更新的时间为上一帧
            trackConfirm(idxTrack).centroid = trackWait(iTrack).centroid;
            % 重新创建KF
            trackConfirm(idxTrack).kalmanFilter = createNewKF(trackWait(iTrack).centroid, 'motionType', p.motionType);
            % 暂时将状态设为失迹, 后续的确立区关联和处理将更改此状态
            trackConfirm(idxTrack).status = "miss";
            % 删除"等待时间"里的该轨迹的记录
            anchorFrame = iFrm - trackWait(iTrack).age; % 等待起始帧
            iFrmDelete = trackConfirm(idxTrack).frame >= anchorFrame;
            trackConfirm(idxTrack).trajectory(iFrmDelete, :) = [];
            trackConfirm(idxTrack).frame(iFrmDelete) = [];
            % 用等待区轨迹覆盖确立区记录
            trackConfirm(idxTrack).frame = [trackConfirm(idxTrack).frame; (anchorFrame : iFrm - 1)'];
            trackConfirm(idxTrack).trajectory = [trackConfirm(idxTrack).trajectory; ...
                repmat(trackWait(iTrack).centroid, iFrm - anchorFrame, 1)];

            %% 删除"等待时间"里, 该轨迹的重叠记录
            for iFrmWait = anchorFrame : iFrm - 1
                iRecDel = [];
                for iRec = 1 : structLength(ovlpRec(iFrmWait).ovlp, 'idxSet')
                    ovlpRec(iFrmWait).ovlp(iRec).idxSet(ovlpRec(iFrmWait).ovlp(iRec).idxSet == trackWait(iTrack).iPeople) = [];
                    if length(ovlpRec(iFrmWait).ovlp(iRec).idxSet) < 2
                        % 若删除此轨迹后该条重叠记录仅剩一条轨迹, 则删除该重叠记录
                        iRecDel = [iRecDel, iRec];
                    end
                end
                ovlpRec(iFrmWait).ovlp = structRowDelete(ovlpRec(iFrmWait).ovlp, iRecDel);
            end
            % 记录新更新的重叠记录的开头位置
            if ~p.backtrackFlag || p.backtrackFlag > anchorFrame
                p.backtrackFlag = anchorFrame;
            end
        end
    end
end

if p.backtrackEn && p.backtrackFlag
    backtrack(p.backtrackFlag, 'trajectoryFrame', 0);
end

%% 删除覆盖到确立区的或年龄达到阈值的等待区轨迹
trackWait = structRowDelete(trackWait, iWait2Confirm);
iDel = [];
for iTrack = 1 : structLength(trackWait, 'iPeople')
    trackWait(iTrack).age = trackWait(iTrack).age + 1;
    if trackWait(iTrack).age > p.waitAgeLmt
        iDel = [iDel, iTrack];
    end
end
trackWait = structRowDelete(trackWait, iDel);
