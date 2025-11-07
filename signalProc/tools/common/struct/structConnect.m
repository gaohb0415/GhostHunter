function structOut = structConnect(structIn)
% 将几个相同结构的struct的字段沿第一维度进行连接
% 输入:
% structIn: 待连接的结构体数组
% 输出:
% structOut: 连接后的结构体
% 作者: 刘涵凯
% 更新: 2024-3-15

%% 初始化输出
fieldNames = fieldnames(structIn);
nField = length(fieldNames);
for iField = 1 : nField
    structOut.(fieldNames{iField}) = [];  % 将字段添加到结构体, 并将初始值设为 []
end

%% 连接
for iField = 1 : length(fieldNames)
    structOut.(fieldNames{iField}) = vertcat(structIn.(fieldNames{iField}));
end
