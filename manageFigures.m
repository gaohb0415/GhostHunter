function manageFigures(maxFigures)
% manageFigures - 检查并强制关闭超出数量限制的窗口 (v3 - 稳健版)
%
% 用法: manageFigures(maxFigures)
%
% 输入:
%   maxFigures - 允许打开的最大窗口数量

    % 使用 get(groot, 'Children') 是查找顶层figure最直接的方式
    all_figs = get(groot, 'Children');
    
    % 如果窗口数量小于等于上限，则什么也不做
    if numel(all_figs) <= maxFigures
        return;
    end
    
    % Children列表默认是按创建顺序（从新到旧）排列的，我们想关掉最旧的。
    % figure对象有一个Number属性，我们可以根据它来排序，确保关闭的是编号最小的。
    fig_numbers = [all_figs.Number];
    
    % 对编号进行升序排序，这样第一个就是最旧的
    sorted_numbers = sort(fig_numbers);
    
    % 计算需要关闭的窗口数量
    num_to_close = numel(sorted_numbers) - maxFigures;
    
    % 关闭所有多余的旧窗口
    for i = 1:num_to_close
        fig_to_close = sorted_numbers(i);
        close(fig_to_close);
        fprintf('窗口数量超限，已关闭最早的窗口 (Figure %d)。\n', fig_to_close);
    end
end