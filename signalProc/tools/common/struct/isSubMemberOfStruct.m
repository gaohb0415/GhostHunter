function idx = isSubMemberOfStruct(structIn, filed, value)
% 判断value是否为结构体某filed的值的一部分
% 当结构体为空、目标字段不存在或为[]时, 输出0
% 输入:
% 1. structIn: 目标结构体
% 2. filed:
% 输出:
% idx: 指定filed的值中包含value的结构体的索引
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
            findRslt =  ismember(value, structIn(iStruct).(filed));
            if prod(findRslt)
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
