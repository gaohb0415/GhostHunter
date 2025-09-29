function array = movmeanV2(array, win)
% 在普通movmean后, 对未被计算的边界进行线性插值
% 输入: 
% 1. array: 输入数组, 第一维度为movmean计算维度
% 2. win: movmean窗大小
% 输出: 
% array: 输出数组
% 作者: 刘涵凯
% 更新: 2024-3-14

[nRow, nCol] = size(array);

halfWin = ceil(win / 2);
idxEdge = [1 : halfWin, nRow - halfWin + 1 : nRow];
idxInner = halfWin + 1 : nRow - halfWin;

array = movmean(array, win, 1);
for iCol = 1 : nCol
    array(idxEdge, iCol) = interp1(idxInner, array(idxInner, iCol), idxEdge, 'linear', 'extrap');
end

