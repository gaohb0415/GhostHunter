function structOut = structRowDelete(structIn, idxDelete)
% 删除结构体的某些行
% 当结构体删除到[]时, 将其各字段赋空值, 如('a', [], 'b', [])
% 输入: 
% 1. structIn: 目标结构体
% 2. idxDelete: 要删除的行的索引
% 输出: 
% structOut: 删除某些行后的结构体
% 作者: 刘涵凯
% 更新: 2022-8-28

structIn(idxDelete) = [];
if size(structIn, 2) == 0
    % 若结构体已删除到[]
    fieldNames = fieldnames(structIn);
    for iName = 1 : length(fieldNames)
        structOut.(fieldNames{iName}) = [];
    end
else
    structOut = structIn;
end
