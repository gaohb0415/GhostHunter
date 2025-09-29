function [faceDs, rcsDs] = faceDownsample(faceIdx, rcs, smplType)
% 根据网格降采样
% 输入:
% 1. faceIdx: 待降采样面在全部面中的索引
% 2. rcs: 各待降采样面的反射强度
% 3. smplType: 采取何种SMPL数据. 'normal'; 'ghost'
% 输出:
% 1. faceDs: 降采样面在全部面中的索引
% 2. rcsDs: 各降采样面的反射强度
% 作者: 刘涵凯
% 更新: 2023-6-28

%% 参数对象
p = simParamShare.param;

%% 载入降采样池
switch smplType % 本体和鬼影使用不同的降采样池
    case 'normal'
        load('meshDs.mat', 'idxFaceDs') % 降采样池中的面在全部面中的索引
        load('smplSeg.mat', 'faceInSeg') % 面-部位从属关系
        segPw = p.segPw;
    case 'ghost'
        load('meshDsGhost.mat', 'idxFaceDs') % 降采样池中的面在全部面中的索引
        load('smplSegGhost.mat', 'faceInSeg') % 面-部位从属关系
        segPw = p.segPwGhost;
end

%% 降采样面补充
faceDs = faceIdx(ismember(faceIdx, idxFaceDs)); % 降采样面在全部面中的索引
faceInSegRaw = faceInSeg(faceIdx); % 待降采样面-部位从属关系
faceInSegDs = faceInSeg(faceDs); % 降采样面-部位从属关系
segRaw = unique(faceInSegRaw); % 待降采样面所涉部位
segDs = unique(faceInSegDs); % 降采样面所涉部位
segRawNotInDs = segRaw(~ismember(segRaw, segDs)); % 存在可见面, 但可见面未在降采样池中的部位
for iMiss = 1 : length(segRawNotInDs)
    % 将待降采样面中该部位的第一个可见面添加到降采样面中
    % 若部位序号为0, 则略过
    if segRawNotInDs(iMiss) == 0
        continue
    end
    [~, faceMiss] = ismember(segRawNotInDs(iMiss), faceInSegRaw);
    faceDs(end + 1) = faceIdx(faceMiss);
    faceInSegDs(end + 1) = segRawNotInDs(iMiss);
end
% 当faceDs只有一个元素时, 在上面代码中的end+1会使其成为行向量. 下面代码作用就是将行向量转为列向量
if size(faceDs, 2) ~= 1
    faceDs = faceDs';
    faceInSegDs = faceInSegDs';
end

%% 删除未在有效部位的面, 主要用于鬼影模型
iInvalid = faceInSegDs == 0;
faceDs(iInvalid) = [];
faceInSegDs(iInvalid) = [];
segDs = unique(faceInSegDs); % 重新统计降采样面所涉部位

%% 提取降采样面的RCS
rcsDs = zeros(size(faceDs)); % 降采样面在待降采样面中的索引
for iSeg = 1 : length(segDs)
    idxInRaw = faceInSegRaw == segDs(iSeg);
    idxInDs = faceInSegDs == segDs(iSeg);
    [~, idxDsInRaw] = ismember(faceDs(faceInSegDs == segDs(iSeg)), faceIdx);
    rcsDs(idxInDs) = rcs(idxDsInRaw) / sum(rcs(idxDsInRaw)) * sum(rcs(idxInRaw)) * segPw(segDs(iSeg));
end
% 删除RCS为0的面
faceDs(rcsDs == 0) = [];
rcsDs(rcsDs == 0) = [];

