function [ptIdx, th] = cfar1D(value, cfarParam)
% 调用MATLAB phased库进行1D CFAR检测
% 输入:
% 1. value: 数值向量
% 2. cfarParam:
%     - .train: 训练单元, 单边
%     - .guard: 保护单元, 单边
%     - .pfa: 虚警概率
%     - .extraTh: 额外阈值
%     - .method (可选): 指定CFAR方法, 'CA' (默认) 或 'OS'
%     - .rank (可选): OS-CFAR的排序值k (仅当 method 为 'OS' 时需要)
% 输出:
% 1. ptIdx: 点云索引
% 2. th: CFAR阈值
% 作者: 刘涵凯
% 更新: 2023-7-3
% 修改: 2025-9-28 (增加了对OS-CFAR的支持)

% --- 第一步: 建立通用的CFAR检测器对象 ---
% 'NumTrainingCells', 2 * cfarParam.train: 设置训练单元，cfarParam.train是单边的，所以双边需要 * 2
% 'NumGuardCells', 2 * cfarParam.guard: 设置保护单元
% 'ProbabilityFalseAlarm', cfarParam.pfa: 设置虚警概率,设定报警器的“敏感度”,值越低误报的概率越低
% 'ThresholdOutputPort', true: 返回计算出的门限值
cfar = phased.CFARDetector('NumTrainingCells', 2 * cfarParam.train, 'NumGuardCells', ...
    2 * cfarParam.guard, 'ProbabilityFalseAlarm', cfarParam.pfa, 'ThresholdOutputPort', true);

% --- 第二步: (这是新增的核心逻辑) 根据参数选择具体的CFAR算法 ---
% 特别注意：这里所谓的切换到OS-CFAR是对于OS-CFAR使用增加了变量rank
% CFAR整体算法逻辑的修改已经封装到了phased.CFARDetector中了

% 检查cfarParam中是否有method字段, 并且其值是否为'OS'
if isfield(cfarParam, 'method') && strcmpi(cfarParam.method, 'OS')
    % 如果用户指定使用 OS-CFAR
    cfar.Method = 'OS';
    
    % 检查用户是否指定了 'rank' (k值), 如果没有, 则设置一个常用的默认值
    if isfield(cfarParam, 'rank')
        cfar.Rank = cfarParam.rank;
    else
        % 如果未指定rank, 推荐使用总训练单元数(双边)的 3/4 或 75% 作为默认值
        cfar.Rank = round(2 * cfarParam.train * 0.75);
    end
else
    % 如果没有指定method, 或者method不是'OS', 则默认使用CA-CFAR
    cfar.Method = 'CA';
end

% --- 第三步: (这部分与你的原始代码完全相同) 执行CFAR并获得结果 ---
% 执行CFAR, 获得检测阈值
% 让 cfar 检测器去分析 value 这个信号
[~, th] = cfar(value, 1 : length(value)); 

% 加入额外阈值
% 施加一个固定的“安全余量”
th = th + cfarParam.extraTh; 

% 获得检测结果
ptIdx = find(value - th > 0);

end