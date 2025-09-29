function len = structFieldLength(structIn, filed)
% 获得结构体的某filed的第一维度的尺寸
% 当结构体为空、目标字段不存在或为[]时, 输出其长度为0
% 输入: 
% 1. structIn: 目标结构体
% 2. filed: 以该字段的长度作为结构体长度
% 输出: 
% len: 结构体长度
% 作者: 刘涵凯
% 更新: 2022-8-28

if isempty(structIn)
    % 若结构体为空
    len = 0;
elseif isfield(structIn, filed)
    % 若目标字段存在
    if isempty(structIn(1).(filed)) % 不要删这个(1)
        % 若目标字段为[]
        len = 0;
    else
        len = size(structIn.(filed), 1);
    end
else
    % 若目标字段不存在
    len = 0;
end
