% 读取并存储SMPL模型顶点与身体部位的对应关系
% 作者: 刘涵凯
% 更新: 2023-6-27

%% 读取json文件
jsonFileHandle = '.\sensing\simulation\SMPL\smpl_vert_segmentation.json';
fid = fopen(jsonFileHandle, 'r');
str = fread(fid, '*char').';
fclose(fid);
segContainVert = jsondecode(str);

%% 载入SMPL模型
load mesh0.mat % 标准姿势

%% 部位-顶点关系
segName = string(fieldnames(segContainVert)); % 部位-顶点包含关系
% 使顶点索引从1开始(原为0)
for iSeg = 1 : length(segName)
    segContainVert.(segName(iSeg)) = segContainVert.(segName(iSeg)) + 1;
end
% 顶点-部位从属关系
vertInSeg = zeros(size(vertices, 1), 1);
for iSeg = 1 : length(segName)
    vertInSeg(segContainVert.(segName(iSeg))) = iSeg;
end

%% 部位-面关系
for iSeg = 1 : length(segName)
    segContainFace.(segName(iSeg)) = [];
end
nFace = size(faces, 1);
nPt = nFace * 3;
facePts = faces(:); % 所有面的顶点
ptInSeg = zeros(nPt, 1);
for iPt = 1 : nPt
    for iSeg = 1 : length(segName)
        if ismember(faces(iPt), segContainVert.(segName(iSeg)))
            ptInSeg(iPt) = iSeg;
            break
        end
    end
end
ptInSeg = reshape(ptInSeg, nFace, 3);
faceInSeg = median(ptInSeg, 2); % 面-部位从属关系. 当面的三个顶点属于两个部位时, 该面属于包含两个顶点的部位
for iFace = 1 : nFace % 部位-面包含关系
    segContainFace.(segName(faceInSeg(iFace))) = [segContainFace.(segName(faceInSeg(iFace))); iFace];
end

%% 保存数据
save('.\sensing\simulation\SMPL\smplSeg.mat', 'segContainVert', 'segContainFace', 'vertInSeg', 'faceInSeg', 'segName')
