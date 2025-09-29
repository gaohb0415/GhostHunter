% SMPL行走数据扩充及累积位移
% 作者: 刘涵凯
% 更新: 2024-3-28

clear; close all

%% 参数对象
p = simParamConfig;
load('smplSeg.mat', 'vertInSeg') % 面-部位从属关系

%% 参数设置
nFrm = 250; % 应大于雷达采集帧数
nData = 2; % 筛选后仅存2条数据
freqMod = 120 ./ (80 : 20 : 160); % 调整不同速度

%% 累积位移
distCum = zeros(nData, nFrm + 1);
for iFreqMod = 1 : length(freqMod)
    freqModFac = freqMod(iFreqMod);
    for iModel = 1 : nData
        %% 载入SMPL数据
        iSmpl = (iFreqMod - 1) * nData + iModel;
        load([p.handleSMPL, 'walking\CMU\mesh_walking_CMU_', num2str(iModel), '.mat'])
        mocap_framerate = round(mocap_framerate / freqModFac); % 调整频率
        smplInfo(iSmpl).iModel = iModel;
        smplInfo(iSmpl).freqModFac = freqModFac;

        %% 维度转换
        trans = permute(trans, [2, 1]); % 将位移维度转换为[坐标, 帧索引]
        joints = permute(joints, [2, 3, 1]); % 将关节点维度转换为[关节索引, 坐标, 帧索引]

        %% 消除位移和旋转
        joints = joints - permute(repmat(trans, [1, 1, 52]), [3, 1, 2]);
        joints = joints([1 : 23, 38], :, :);
        for iFrm = 1 : mocap_frame_sum
            joints(:, :, iFrm) = coordTransform2D(joints(:, :, iFrm), -rad2deg(orientation(iFrm, 2)) + oriShift, transShift);
        end

        %% 数据扩充
        idx = 1 : (mocap_framerate / p.fFrm) : mocap_frame_sum; % 符合雷达帧频的SMPL帧索引
        transTemp = trans(:, idx);
        idxLink = []; % 连接处的索引
        while length(idx) < nFrm + 2
            jointEnd = joints(:, :, idx(end)); % 当前最后的关节点坐标
            jointDist = squeeze(mean(vecnorm(joints - jointEnd, 2, 2), 1)); % 欧氏距离(可能不严谨, 意思对了就行)
            jointDist = movmean(jointDist(1 : 2 * mocap_framerate), ceil(0.05 * mocap_framerate)); % 取前2秒的数据
            [~, idxSame] = min(jointDist); % 最相像的帧的索引
            idxNew = idxSame - (mocap_framerate / p.fFrm) : (mocap_framerate / p.fFrm) : mocap_frame_sum - 1; % 注意这里是从idxSame的"上一帧"开始的
            idxLink = [idxLink, length(idx) + 1];
            idxNew(1) = max(1, idxNew(1));
            idx = [idx, idxNew(2 : end)];
            % 位移校正
            transNew = trans(:, idxNew(2 : end)) + transTemp(:, end) - trans(:, idxNew(1));
            transNew(:, 1) = (transTemp(:, end) + transNew(:, 2)) / 2;
            transTemp = [transTemp, transNew]; 
        end
        smplInfo(iSmpl).iExtr = idx; % 雷达数据的每帧对应的SMPL数据索引
        smplInfo(iSmpl).link = idxLink;
        trans = transTemp;

        %% 计算累积位移
        dist = vecnorm(trans(:, 2 : end) - trans(:, 1 : end - 1), 2, 1);
        dist = smoothdataV3(dist', 1, 'movmean', 5)'; % 平滑
        distCumTemp = [0, cumsum(dist)];
        distCum(iSmpl, :) = distCumTemp(1 : nFrm + 1);
    end
end

%% 保存数据
save('.\sensing\simulation\SMPL\distCum.mat', 'distCum', 'smplInfo');
