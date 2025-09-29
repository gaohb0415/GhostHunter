function matOut = matExtract(matIn, dimTarget, binSelect)
% 将矩阵进行提取、平均, 最终获得目标维度的矩阵或向量
% 输入: 
% 1. mat: 矩阵
% 2. dimTarget: 目标向量维度. 如: 3-第3维; [1, 2]-第1、2维
% 3. binSelect: 其他维度的提取索引. 0-沿该维度平均; 非0-提取该索引下的数据
% 输出: 
% matOut: 目标维度的矩阵或向量
% 作者: 刘涵凯
% 更新: 2022-6-19

dims = 1 : ndims(matIn);
dims(dimTarget) = [];

nDims = length(dims);
nBinSelect = length(binSelect);

if nBinSelect < nDims
    binSelect = [binSelect, nDims - nBinSelect];
elseif nBinSelect > nDims
    binSelect = binSelect(1 : nDims);
end

for iDim = 1 : nDims
    if binSelect(iDim) == 0
        matIn = mean(matIn, dims(iDim));
    else
        binIndex = {':', ':', ':', ':', ':'};
        binIndex(dims(iDim)) = {binSelect(iDim)};
        matIn = matIn(binIndex{:});
    end
end

matOut = squeeze(matIn);
