function arrayOut = smoothdataV2(arrayIn, dim, method, win)
% 在普通smoothdata后, 仅保留被平滑的部分, 删除边界
% 输入: 
% 1. arrayIn: 输入数组, 第一维度为movmean计算维度
% 2. dim: 平滑计算维度, 现只支持1
% 3. method: 平滑方法, 同smoothdata函数
% 4. win: 平滑窗大小
% 输出: 
% arrayOut: 输出数组
% 作者: 刘涵凯
% 更新: 2024-3-14

halfWin = floor(win / 2);
lenArray = size(arrayIn, 1);
iLeft = halfWin + 1;
iRight = lenArray - halfWin;

arraySmth = smoothdata(arrayIn, dim, method, win);
arrayOut = arraySmth(iLeft : iRight, :, :, :, :);

