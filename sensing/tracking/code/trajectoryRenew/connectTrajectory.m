function connectFlag = connectTrajectory(idxCand, pc)
% 轨迹续接
% 输入: 
% 1. idxCand: 待续接的候选区轨迹索引
% 2. pc: 待续接轨迹的关联点云簇的点云
% 输出: 
% connectFlag: 续接标记. 0-未进行续接; 1-已进行续接
% 作者: 刘涵凯
% 更新: 2023-3-9

%% 参数对象及全局变量
p = trackParamShare.param;
global iFrm trackCand trackConfirm trackWait trajectory

%% 轨迹续接
connectFlag = 0;
if structLength(trackConfirm, 'centroid')
    % 首先筛选出失踪时间差小于retrvFrmDif的轨迹

    % 下方代码的失踪定义为"非active"
    % idxMiss = find(arrayfun(@(x) ~strcmp("active", x.status) & ...
    % x.statusAge - trackCand(idxCand).age > -5 & ... % 这里用-5而不是0是为了应对一些特殊情况
    % x.statusAge - trackCand(idxCand).age < p.retrvFrmDif, trackConfirm));
    % 暂时修改一下 不考虑时间差
    % idxMiss = find(arrayfun(@(x) ~strcmp("active", x.status), trackConfirm));
    % 暂时修改为只对失迹轨迹
    idxMiss = find(arrayfun(@(x) strcmp("miss", x.status), trackConfirm));
    
    if ~isempty(idxMiss)
        % 然后通过"确立区轨迹失踪时的位置"和"候选区轨迹出现时的位置"的距离差判断是否续接
        posDif = vertcat(trackConfirm(idxMiss).centroid) - trackCand(idxCand).trajectory(1, :);
        d = sqrt(posDif(:, 1) .^ 2 + posDif(:, 2) .^ 2);
        while ~isempty(d)
            [dMin, idxMin] = min(d);
            % 通过"确立区轨迹失踪时的帧"和"候选区轨迹出现时的帧"的帧差和失踪轨迹的状态设定距离阈值
            frmDif = abs(trackConfirm(idxMiss(idxMin)).statusAge - trackCand(idxCand).age);
            % 对miss和非miss的失踪轨迹采取不同的阈值基准. 忘记这样做的原因了
            if strcmp(trackConfirm(idxMiss(idxMin)).status, "miss")
                distDifTh = p.retrvDistDif(1);
            else
                distDifTh = p.retrvDistDif(2);
            end
            if dMin < distDifTh + frmDif * p.retrvDistDif1Frm
                %% 续接
                connectFlag = 1; % 续接标记
                % 将续接的候选区轨迹起始帧作为回溯标记
                if ~p.backtrackFlag || p.backtrackFlag > trackCand(idxCand).frame(1)
                    p.backtrackFlag = trackCand(idxCand).frame(1);
                end
                % 续接trackConfirm
                trackConfirm(idxMiss(idxMin)).pc = pc;
                trackConfirm(idxMiss(idxMin)).centroid = trackCand(idxCand).centroid;
                trackConfirm(idxMiss(idxMin)).kalmanFilter = trackCand(idxCand).kalmanFilter;
                trackConfirm(idxMiss(idxMin)).status = 'active';
                trackConfirm(idxMiss(idxMin)).statusAge = 1; % 对active而言, age并不重要, 这里索性设为1
                % 续接轨迹记录
                iTraj = find(arrayfun(@(x) ismember(trackConfirm(idxMiss(idxMin)).iPeople, x.iPeople), trajectory(iFrm - 1).track));
                iFrmDel = find(trajectory(iFrm - 1).track(iTraj).frame >= trackCand(idxCand).frame(1));
                % 删除重复的旧轨迹记录
                trajectory(iFrm - 1).track(iTraj).trajectory(iFrmDel, :) = [];
                trajectory(iFrm - 1).track(iTraj).frame(iFrmDel) = [];
                trackConfirm(idxMiss(idxMin)).trajectory(iFrmDel, :) = [];
                trackConfirm(idxMiss(idxMin)).frame(iFrmDel) = [];
                % 新增轨迹记录
                trajectory(iFrm - 1).track(iTraj).trajectory = [trajectory(iFrm - 1).track(iTraj).trajectory; ...
                    trackCand(idxCand).trajectory(1 : end - 1, :)];
                trajectory(iFrm - 1).track(iTraj).frame = [trajectory(iFrm - 1).track(iTraj).frame; ...
                    trackCand(idxCand).frame(1 : end - 1)];
                trackConfirm(idxMiss(idxMin)).trajectory = [trackConfirm(idxMiss(idxMin)).trajectory; ...
                    trackCand(idxCand).trajectory(1 : end - 1, :)];
                trackConfirm(idxMiss(idxMin)).frame = [trackConfirm(idxMiss(idxMin)).frame; ...
                    trackCand(idxCand).frame(1 : end - 1)];
                % 在等待区中删除相应轨迹
                iDel = find(arrayfun(@(x) ismember(trackConfirm(idxMiss(idxMin)).iPeople, x.iPeople), trackWait));
                trackWait = structRowDelete(trackWait, iDel);
                break
            else
                d(idxMin) = [];
            end
        end
    end
end
