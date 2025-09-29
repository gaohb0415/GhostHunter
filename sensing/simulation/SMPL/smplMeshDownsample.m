% 对SMPL网格进行基于三维坐标的降采样
% 作者: 刘涵凯
% 更新: 2024-3-26

%% 载入SMPL模型
load mesh0.mat % 标准姿势
load('smplSeg.mat', 'segContainVert') % 部位-顶点从属关系

%% 设置模式
type = 'normal'; % 'normal' 'ghost'

%% 代表躯干的部位
switch type
    case 'normal'
        segTorso = ["spine1", "spine"]; % 由上至下 "spine2", "spine1", "spine", "hips"
        iVertTorso = [];
        for iSeg = 1 : length(segTorso)
            iVertTorso = [iVertTorso; segContainVert.(segTorso(iSeg))];
        end
    case 'ghost'
        segGhost = ["head", "neck", "leftShoulder", "rightShoulder", "spine2", "spine1", "spine", "hips", "leftArm", "rightArm", "leftUpLeg", "rightUpLeg"];
        iVertGhost = [];
        for iSeg = 1 : length(segGhost)
            iVertGhost = [iVertGhost; segContainVert.(segGhost(iSeg))];
        end
end

%% 降采样
switch type
    case 'normal'
        % % 躯干部位
        vertTorsoDs = pcDsFast(vertices(iVertTorso, :), 0.05); % 正常0.08 多普勒0.05 还原度0.02 论文画图0.1
        % 其他部位
        vertOtherDs = pcDsFast(vertices(~ismember(1 : size(vertices, 1), iVertTorso), :), 0.05); % 正常0.08 多普勒0.05 还原度0.02 论文画图0.1
        vertDs = [vertTorsoDs; vertOtherDs];
    case 'ghost'
        vertDs = pcDsFast(vertices(iVertGhost, :), 0.1); % 0.2 0.4 多普勒0.1  论文画图0.02
end

%% 离降采样结果最近的顶点和面
% 顶点预处理
verticAll = vertices;
idxVertAll = 1 : size(vertices, 1);
idxVertDs = zeros(size(vertDs, 1), 1);
% 反射面预处理
centroid = mean(permute(reshape(vertices(faces(:), :), [size(faces, 1), 3, 3]), [1, 3, 2]), 3);
centroidAll = centroid;
idxFaceAll = 1 : size(faces, 1);
idxFaceDs = zeros(size(vertDs, 1), 1);
% 寻找最近邻
for iVert = 1 : size(vertDs)
    % 最近顶点
    [~, idxVertMin] = min(vecnorm(verticAll - vertDs(iVert, :), 2, 2));
    idxVertDs(iVert) = idxVertAll(idxVertMin);
    % 最近反射面
    [~, idxFaceMin] = min(vecnorm(centroidAll - vertDs(iVert, :), 2, 2));
    idxFaceDs(iVert) = idxFaceAll(idxFaceMin);
    % 防止点重复
    verticAll(idxVertMin, :) = [];
    idxVertAll(idxVertMin) = [];
    centroidAll(idxFaceMin, :) = [];
    idxFaceAll(idxFaceMin) = [];
end

%% 图示降采样结果
% y和z交换了一下, 在figure里旋转方便
close all; plot3(vertices(:, 1), vertices(:, 3), vertices(:, 2), '.', 'markersize', 1, 'color', 'b'); drawnow; axis equal
hold on; trisurf(faces(idxFaceDs, :), vertices(:, 1), vertices(:, 3), vertices(:, 2), 'FaceColor', 'r'); axis equal; drawnow;

%% 保存数据
switch type
    case 'normal'
        save('.\sensing\simulation\SMPL\meshDs.mat', 'idxVertDs', 'idxFaceDs')
    case 'ghost'
        save('.\sensing\simulation\multipath\meshDsGhost.mat', 'idxVertDs', 'idxFaceDs')
end
