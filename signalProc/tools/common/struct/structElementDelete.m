function struct = structElementDelete(struct, idxDelete, varargin)
% 删除结构体的某些元素
% 输入:
% 1. struct: 目标结构体
% 2. idxDelete: 待删除元素的索引
% 3. varargin:
%     - fieldNames: 参与元素删除的字段
% 输出:
% struct: 删除元素后的结构体
% 作者: 刘涵凯
% 更新: 2023-6-30

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('fieldNames', fieldnames(struct));
p.parse(varargin{:});
fieldNames = p.Results.fieldNames;

%% 删除元素
if ~isempty(idxDelete)
    for iField = 1 : length(fieldNames)
        struct.(fieldNames{iField})(idxDelete) = []; % 若数据有多维度, 则删除第一维的idxDelete行
    end
end
