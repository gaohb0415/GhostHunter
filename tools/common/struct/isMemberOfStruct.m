function idx = isMemberOfStruct(structIn, filed, value)
% 判断结构体某filed的值是否为value
% 当结构体为空、目标字段不存在或为[]时, 输出0
% 输入: 
% 1. structIn: 目标结构体
% 2. filed:
% 输出: 
% idx: 指定filed的值等于value的结构体的索引
% 作者: 刘涵凯
% 更新: 2023-3-9

idx = 0;
if isempty(structIn)
    % 若结构体为空
    return
elseif isfield(structIn, filed)
    % 若目标字段存在
    if isempty(structIn(1).(filed)) % 不要删这个(1)
        % 若目标字段为[]
        return
    else
        len = length(structIn);
        for iStruct = 1 : len
            if isequal(structIn(iStruct).(filed), value)
                if ~idx
                    idx = iStruct;
                else
                    idx = [idx, iStruct];
                end
            end
        end
    end
else
    % 若目标字段不存在
    return
end
