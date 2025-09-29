function arrayOut = smoothdataV3(arrayIn, dim, method, win)
% 在普通smoothdata后, 对边界继续执行窗口递减的smoothdata
% 输入: 
% 1. arrayIn: 输入数组, 第一维度为movmean计算维度
% 2. dim: 平滑计算维度, 现只支持1
% 3. method: 平滑方法, 同smoothdata函数
% 4. win: 平滑窗大小
% 输出: 
% arrayOut: 输出数组
% 作者: 刘涵凯
% 更新: 2024-3-14

arrayOut = zeros(size(arrayIn));
arrayOut([1, end], :, :, :, :) = arrayIn([1, end], :, :, :, :);

halfWin = floor(win / 2);
lenArray = size(arrayIn, 1);
iLeft = halfWin + 1;
iRight = lenArray - halfWin;
arrayOut(iLeft : iRight, :, :, :, :) = smoothdataV2(arrayIn, dim, method, win);

while iLeft > 2
    iLeft = iLeft - 1;
    iRight = iRight + 1;
    win = 2 * iLeft - 1;
    arrayOut(iLeft, :, :, :, :) = smoothdataV2(arrayIn(1 : win, :, :, :, :), dim, method, win);
    arrayOut(iRight, :, :, :, :) = smoothdataV2(arrayIn(end - win + 1 : end, :, :, :, :), dim, method, win);
end

%% 对首位进行一次平滑, 有需要时开启
% head = smoothdata(arrayIn(1 : 4, :, :, :, :), dim, method, 4);
% tail = smoothdata(arrayIn(end - (3 : -1 : 0), :, :, :, :), dim, method, 4);
% arrayOut([1, 2], :, :, :, :) = head([1, 2], :, :, :, :);
% arrayOut([end - 1, end], :, :, :, :) = tail([end - 1, end], :, :, :, :);
