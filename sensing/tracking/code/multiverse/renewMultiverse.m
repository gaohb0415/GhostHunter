function renewMultiverse(trajectory, frame, sepRecIn, backtrackFlag, sepDelFlag)
% 平行宇宙更新
% 输入:
% 1. trajectory: 主宇宙轨迹记录
%  - .iPeople: 轨迹计数ID
%  - .name: 人员姓名ID
%  - .trajectory: 轨迹记录
%  - .frame: 帧记录
%  - .status: 轨迹状态
%  - .pcLast: 最后帧的点云
%  - .kalmanFilter: 卡尔曼滤波器
%  - .statusAge: 状态年龄
% 2. frame: 待更新的帧
% 3. sepRecIn: 该帧的轨迹分离记录
%  - .idxSet: 分离轨迹的iPeople对
%  - .nSeperate: 分离轨迹数
% 4. backtrackFlag: 回溯标记. 0-不回溯; 非0-回溯起点帧
% 5. sepDelFlag: 分离记录删除标记. 0-未删除; 非0-删除起点帧
% 作者: 刘涵凯
% 更新: 2023-3-15

%% 参数对象及全局变量
p = trackParamShare.param;
global multivRec sepRec

%% 平行宇宙回溯
if p.backtrackEn && any(backtrackFlag) || any(sepDelFlag)
    if ~backtrackFlag
        % backtrackFlag的优先级高于sepDelFlag
        backtrackFlag = sepDelFlag;
    end
    % 对有关轨迹ID的变量进行ID回溯
    backtrack(backtrackFlag)
    % 回溯
    multiverseBacktrack(backtrackFlag);
    % 重新记录平行宇宙
    for iFrm = backtrackFlag : frame
        renewMultiverse(extractTrajectory(trajectory, iFrm), iFrm, sepRec(iFrm).sep, 0, 0)
    end
    return
end

%% 将现有平行宇宙的轨迹更新到最新帧
if p.nMultiv == 1
    % 从未发生轨迹分离时, 直接复制追踪结果
    multivRec(frame).multiv.track = trajectory;
    multivRec(frame).multiv.association = [vertcat(trajectory.iPeople), vertcat(trajectory.iPeople)];
else
    % 曾经发生过轨迹分离, 则更新平行宇宙记录中的轨迹
    % 提取上一帧的平行宇宙
    trackMultiv = multivRec(frame - 1).multiv;
    % 更新已有轨迹
    for iMultivOld = 1 : structLength(trackMultiv, 'iMultiverse')
        for iTraj = 1 : structLength(trackMultiv(iMultivOld).track, 'iPeople')
            % 确认身份索引
            % 平行宇宙中的iPeople
            iPplNew = trackMultiv(iMultivOld).track(iTraj).iPeople;
            % 对应主宇宙中的iPeople
            iPplOld = trackMultiv(iMultivOld).association(trackMultiv(iMultivOld).association(:, 2) == iPplNew, 1);
            % 轨迹在主宇宙中的索引
            idx = isMemberOfStruct(trajectory, 'iPeople', iPplOld);
            % 更新轨迹
            % 主宇宙轨迹待写入平行宇宙的部分的索引
            idxNew = find(trajectory(idx).frame > p.anchorFrmMultiv);
            % 主宇宙轨迹待写入平行宇宙的轨迹记录
            trajectoryNew = trajectory(idx).trajectory(idxNew, :);
            % 主宇宙轨迹待写入平行宇宙的帧记录
            frameNew = trajectory(idx).frame(idxNew);
            % 写入平行宇宙
            trackMultiv(iMultivOld).track(iTraj).trajectory = [trackMultiv(iMultivOld).track(iTraj).trajectory; trajectoryNew];
            trackMultiv(iMultivOld).track(iTraj).frame = [trackMultiv(iMultivOld).track(iTraj).frame; frameNew];
            trackMultiv(iMultivOld).track(iTraj).kalmanFilter = trajectory(idx).kalmanFilter;
            trackMultiv(iMultivOld).track(iTraj).statusAge = trajectory(idx).statusAge;
            trackMultiv(iMultivOld).track(iTraj).status = trajectory(idx).status;
            trackMultiv(iMultivOld).track(iTraj).pcLast = trajectory(idx).pcLast;
        end
    end
    % 若有新轨迹出现, 将其加入平行宇宙中
    for iTraj = 1 : structLength(trajectory, 'iPeople')
        if ~isMemberOfStruct(trackMultiv(1).track, 'iPeople', trajectory(iTraj).iPeople)
            nTrajOld = structLength(trackMultiv(1).track, 'iPeople');
            for iMultivOld = 1 : structLength(trackMultiv, 'iMultiverse')
                trackMultiv(iMultivOld).track(nTrajOld + 1) = trajectory(iTraj);
                % 更新平行宇宙轨迹与主宇宙轨迹的关联关系
                trackMultiv(iMultivOld).association = ...
                    [trackMultiv(iMultivOld).association; [trajectory(iTraj).iPeople, trajectory(iTraj).iPeople]];
            end
        end
    end
    % 平滑. 注意, 为了保证轨迹的快速准确更新, 不对最后位置进行平滑
    for iMultiv = 1 : structLength(trackMultiv, 'iMultiverse')
        for iTraj = 1 : structLength(trackMultiv(iMultiv).track, 'iPeople')
            if length(trackMultiv(iMultiv).track(iTraj).frame) > p.smthLen
                trackMultiv(iMultiv).track(iTraj).trajectory(end - p.smthLen : end - 1, :) = ...
                    smoothdataV3(trackMultiv(iMultiv).track(iTraj).trajectory(end - p.smthLen : end - 1, :), 1, p.smthMeth, p.smthWin);
            end
        end
    end
    % 将最新的平行宇宙记录保存到结构体中
    multivRec(frame).multiv = trackMultiv;
end
% 更新平行宇宙锚点帧
p.anchorFrmMultiv = frame;

%% 若此帧有轨迹分离记录, 则基于现有平行宇宙进行分枝
if p.multiverseEn && structLength(sepRecIn, 'idxSet')
    % 提取更新到当前帧的多元宇宙
    trackMultivOld = multivRec(frame).multiv;
    % 新建平行宇宙结构体用于新的轨迹分离
    trackMultivNew = struct('iMultiverse', 1, 'track', struct('iPeople', [], 'name', [], ...
        'trajectory', [], 'frame', [], 'status', [], 'pcLast', [], 'kalmanFilter', [], 'statusAge', []), ...
        'brother', [], 'parent', [], 'association', []);
    % 平行宇宙分枝
    for iSep = 1 : structLength(sepRecIn, 'idxSet')
        % 提取分离记录
        iPplSep = sepRecIn(iSep).idxSet;
        nPplSep = sepRecIn(iSep).nSeperate;
        % 这里将分离过程简化了, 如果重叠的3人变为1+2, 则直接按照1+1+1处理
        if length(iPplSep) > nPplSep
            nPplSep = length(iPplSep);
        end
        % 计算所有可能的分离情况, 生成关联表
        assocTable = perms(1 : nPplSep);
        % 对每个现有平行宇宙进行分枝
        for iMultivOld = 1 : structLength(trackMultivOld, 'iMultiverse')
            % 分枝数量, 即关联表的所有可能性
            nMultivNew = size(assocTable, 2);
            % 分枝的平行宇宙ID
            idxMultivNew = p.nMultiv + (1 : nMultivNew);
            % 更新平行宇宙数量
            p.nMultiv = p.nMultiv + nMultivNew;
            % 主宇宙中分离的轨迹的在此平行宇宙中的iPeople
            iPplSepOld = zeros(1, nPplSep);
            for iTrajSep = 1 : nPplSep
                idx = find(trackMultivOld(iMultivOld).association(:, 1) == iPplSep(iTrajSep));
                iPplSepOld(iTrajSep) = trackMultivOld(iMultivOld).association(idx, 2);
            end
            % 对该平行宇宙进行分枝
            for iNew = 1 : nMultivNew
                % 该分枝在新平行宇宙(即全部平行宇宙的分枝)中的索引
                iMultivNew = nMultivNew * (iMultivOld - 1) + iNew;
                % 记录平行宇宙ID
                trackMultivNew(iMultivNew).iMultiverse = idxMultivNew(iNew);
                % 记录上级宇宙和同级宇宙(代码中暂时没用到)
                trackMultivNew(iMultivNew).parent = trackMultivOld(iMultivOld).iMultiverse;
                trackMultivNew(iMultivNew).brother = idxMultivNew(idxMultivNew ~= idxMultivNew(iNew));
                % 分离的轨迹的在此平行宇宙中的iPeople
                iPplSepNew = iPplSepOld(assocTable(iNew, :));
                % 初始化关联表. 分枝将继承上级宇宙的关联表的第一列
                trackMultivNew(iMultivNew).association = trackMultivOld(iMultivOld).association;
                % 更新分枝的关联表
                for iTrajSep = 1 : nPplSep
                    idx = find(trackMultivOld(iMultivOld).association(:, 2) == iPplSepOld(iTrajSep));
                    trackMultivNew(iMultivNew).association(idx, 2) = iPplSepNew(iTrajSep);
                end
                % 初始化track
                trackMultivNew(iMultivNew).track = trackMultivOld(iMultivOld).track;

                %% 对分离轨迹进行内容交换, 即"分枝"
                for iTrajSep = 1 : nPplSep
                    % 主宇宙中分离的轨迹ID在上级宇宙中的ID
                    iPplOld = trackMultivOld(iMultivOld).association(...
                        find(trackMultivOld(iMultivOld).association(:, 1) == iPplSep(iTrajSep)), 2);
                    % 主宇宙中分离的轨迹ID在分枝宇宙中的ID
                    iPplNew = trackMultivNew(iMultivNew).association(...
                        find(trackMultivNew(iMultivNew).association(:, 1) == iPplSep(iTrajSep)), 2);
                    if iPplOld ~= iPplNew
                        % 若分枝宇宙改变了上级宇宙中该轨迹与主宇宙的关联关系, 则执行内容覆盖
                        % 这里默认只对最后一帧进行覆盖, 因为主程序每帧都会执行该函数
                        % 用于覆盖的轨迹在上级宇宙中的索引
                        idxOld = isMemberOfStruct(trackMultivOld(iMultivOld).track, 'iPeople', iPplOld);
                        % 被覆盖轨迹在分枝宇宙中的索引
                        idxNew = isMemberOfStruct(trackMultivNew(iMultivNew).track, 'iPeople', iPplNew);
                        if trackMultivNew(iMultivNew).track(idxNew).frame(end) == frame
                            % 若被覆盖的轨迹的最后一帧存在内容, 则删去
                            trackMultivNew(iMultivNew).track(idxNew).trajectory(end, :) = [];
                            trackMultivNew(iMultivNew).track(idxNew).frame(end) = [];
                            trackMultivNew(iMultivNew).track(idxNew).pcLast = [];
                        end
                        if trackMultivOld(iMultivOld).track(idxOld).frame(end) == frame
                            % 若用于覆盖的轨迹的最后一帧存在内容, 则写入被覆盖的轨迹
                            trackMultivNew(iMultivNew).track(idxNew).trajectory(end + 1, :) = ...
                                trackMultivOld(iMultivOld).track(idxOld).trajectory(end, :);
                            trackMultivNew(iMultivNew).track(idxNew).frame(end + 1) = ...
                                trackMultivOld(iMultivOld).track(idxOld).frame(end);
                            trackMultivNew(iMultivNew).track(idxNew).status = ...
                                trackMultivOld(iMultivOld).track(idxOld).status;
                            trackMultivNew(iMultivNew).track(idxNew).pcLast = ...
                                trackMultivOld(iMultivOld).track(idxOld).pcLast;
                        end
                    end
                end
            end
        end
        % 将最新的平行宇宙记录保存到结构体中
        multivRec(frame).multiv = trackMultivNew;
    end
end
