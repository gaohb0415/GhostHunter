function [vert, joint] = smplForMove(smplDataHandle, smplInfo)
% 用于行走的SMPL生成
% 输入:
% 1. smplDataHandle: SMPL数据地址
% 2. smplInfo: SMPL数据与雷达的适配信息
%     - .iExtr: 每帧雷达数据对应的SMPL数据的索引
%     - .freqModFac: 速度调整系数
%     - .link: 连接处的索引
% 输出:
% 1. vert: 顶点坐标, [顶点索引, 坐标, 帧索引]
% 2. joint: 关节点坐标, [关节索引, 坐标, 帧索引]
% 作者: 刘涵凯
% 更新: 2024-3-28

%% 参数对象
p = simParamShare.param;

%% 载入SMPL数据
load(smplDataHandle)
load('smplSeg.mat', 'vertInSeg') % 面-部位从属关系
idx = smplInfo.iExtr; % 扩充数据

%% 维度转换
trans = permute(trans, [2, 1]); % 将位移维度转换为[坐标, 帧索引]
vertices = permute(vertices, [2, 3, 1]); % 将顶点维度转换为[顶点索引, 坐标, 帧索引]
joints = permute(joints, [2, 3, 1]); % 将关节点维度转换为[关节索引, 坐标, 帧索引]

%% 消除位移和旋转
vertices = vertices - permute(repmat(trans, [1, 1, 6890]), [3, 1, 2]);
joints = joints - permute(repmat(trans, [1, 1, 52]), [3, 1, 2]);
for iFrm = 1 : mocap_frame_sum
    vertices(:, :, iFrm) = coordTransform2D(vertices(:, :, iFrm), -rad2deg(orientation(iFrm, 2)) + oriShift, transShift);
    joints(:, :, iFrm) = coordTransform2D(joints(:, :, iFrm), -rad2deg(orientation(iFrm, 2)) + oriShift, transShift);
end

%% 网格平滑
mocap_framerate = mocap_framerate / smplInfo.freqModFac; % 速度调整, 与smplDistCum一致
smoothWin = ceil(p.smthTimeWin * mocap_framerate);
joints = smoothdata(joints, 3, 'movmean', smoothWin);
% idxVertSmooth = find(ismember(vertInSeg, [7, 8, 9, 10, 12, 19, 21, 24])); % 仅平滑躯干
idxVertSmooth = find(ismember(vertInSeg, [3, 4, 13, 15, 7, 8, 9, 10, 12, 17, 18, 19, 21, 24])); % 微多普勒实验所需
vertices(idxVertSmooth, :, :) = smoothdata(vertices(idxVertSmooth, :, :), 3, 'movmean', smoothWin);

%% 加入身高
% height = mean(trans(3, :));
height = trans(3, :);
vertices(:, 3, :) = vertices(:, 3, :) + permute(repmat(height, [1, 1, 6890]), [3, 1, 2]);
joints(:, 3, :) = joints(:, 3, :) + permute(repmat(height, [1, 1, 52]), [3, 1, 2]);

%% 数据提取
vert = vertices(:, :, idx);
joint = joints(:, :, idx);

%% 对连接的时隙进行平滑
smthTimeWinLink = floor(p.smthTimeWinLink * p.fFrm);
if mod(smthTimeWinLink, 2) == 0; smthTimeWinLink = smthTimeWinLink + 1; end % 将窗口帧数设定为奇数
smthFrmLink = ceil(p.smthTimeLink * p.fFrm);
for iLink = 1 : length(smplInfo.link)
    idxLinkFrm = smplInfo.link(iLink) + (-smthFrmLink : smthFrmLink); % 被平滑的帧
    idxLinkWin = idxLinkFrm(1) - floor(smthTimeWinLink / 2) : idxLinkFrm(end) + floor(smthTimeWinLink / 2); % 平滑窗内的帧
    % 网格点处理
    vertSeg = movmean(vert(:, :, idxLinkWin), smthTimeWinLink, 3, 'Endpoints', 'discard');
    vertTail = vert(:, :, idxLinkFrm(end) : end);
    vert(:, :, idxLinkFrm) = vertSeg - vertSeg(:, :, 1) + vert(:, :, idxLinkFrm(1)); % 起始处校正
    vert(:, :, idxLinkFrm(end) : end) = vertTail - vertTail(:, :, 1) + vert(:, :, idxLinkFrm(end)); % 结尾处校正
    % 关节点处理
    jointSeg = movmean(joint(:, :, idxLinkWin), smthTimeWinLink, 3, 'Endpoints', 'discard');
    jointTail = joint(:, :, idxLinkFrm(end) : end);
    joint(:, :, idxLinkFrm) = jointSeg - jointSeg(:, :, 1) + joint(:, :, idxLinkFrm(1));
    joint(:, :, idxLinkFrm(end) : end) = jointTail - jointTail(:, :, 1) + joint(:, :, idxLinkFrm(end));
end

%% 提取雷达帧数范围内的SMPL数据
vert = vert(:, :, 1 : p.nFrm + 1);
joint = joint(:, :, 1 : p.nFrm + 1);
