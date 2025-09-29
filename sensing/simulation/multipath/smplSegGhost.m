% 对SMPL网格进行粗略部位划分, 用于鬼影计算
% 作者: 刘涵凯
% 更新: 2023-8-25

%% 载入SMPL面-全部位关系
load('smplSeg.mat')

%% 部位粗略划分
ghostSegName = ["head", "neck", "leftShoulder", "rightShoulder", "spine2", "spine1", "spine", "hips", "leftArm", "rightArm", "leftUpLeg", "rightUpLeg"];
for iSeg = 1 : length(ghostSegName)
    ghostSeg.(ghostSegName(iSeg)) = ghostSegName(iSeg);
end

%% 记录部位-面/顶点包含关系
for iSeg = 1 : length(ghostSegName)
    seg = ghostSegName(iSeg);
    subSegs = ghostSeg.(seg);
    ghostSegContainFace.(seg) = [];
    ghostSegContainVert.(seg) = [];
    for iSubSeg = 1 : length(subSegs)
        ghostSegContainFace.(seg) = [ghostSegContainFace.(seg); segContainFace.(subSegs(iSubSeg))]; 
        ghostSegContainVert.(seg) = [ghostSegContainVert.(seg); segContainVert.(subSegs(iSubSeg))]; 
    end
end

%% 记录面/顶点-部位从属关系
ghostFaceInSeg = zeros(size(faceInSeg));
ghostVertInSeg = zeros(size(vertInSeg));
for iSeg = 1 : length(ghostSegName)
    seg = ghostSegName(iSeg);
    ghostFaceInSeg(ghostSegContainFace.(seg)) = iSeg;
    ghostVertInSeg(ghostSegContainVert.(seg)) = iSeg;
end

%% 保存数据
segName = ghostSegName;
segContainFace = ghostSegContainFace;
segContainVert = ghostSegContainVert;
faceInSeg = ghostFaceInSeg;
vertInSeg = ghostVertInSeg;
save('.\simulation\multipath\smplSegGhost.mat', 'segName', 'segContainFace', 'segContainVert', 'faceInSeg', 'vertInSeg')
