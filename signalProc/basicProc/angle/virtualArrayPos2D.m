function virtualArrayPos2D
% 计算二维虚拟阵列的相对位置
% 最新版本仅cascade.half测试过
% 保存数据: 
% array2D
% - .pos: 虚拟阵列中各阵元的相对位置按位置顺序排列
% - .iSort: pos由Tx-Rx Channel顺序转换为位置顺序(含删除重叠阵元)的索引
% - .iUnit: pos由位置顺序转换为Tx-Rx Channel顺序的索引
% - .iGlobal: 将补零后的阵列矩阵拉伸为向量后, 阵元在向量中的索引
% 作者: 刘涵凯
% 更新: 2022-7-3

%% 根据天线排布生成虚拟阵列位置
% cascade.half
baselineRxX = [0, 1, 2, 3, -11, -10, -9, -8];
baselineRxZ = [0, 0, 0, 0, 0, 0, 0, 0];
baselineTxX = [0, -1, -2, -3, -7, -11];
baselineTxZ = [0, 2, 5, 6, 6, 6];
x = baselineRxX' + baselineTxX;
x = x(:);
z = baselineRxZ' + baselineTxZ;
z = z(:);
array2D.pos.cascade.half = [x, z];

%% 删除位置重叠的阵元, 将位置序列重新排序
[array2D.pos.cascade.half, array2D.iSort.cascade.half, array2D.iUnit.cascade.half] = unique(array2D.pos.cascade.half, 'rows', 'stable');

%% 阵元在补零后的矩阵中的索引
% 将阵列矩阵拉伸为向量后, 阵元在向量中的索引
% cascade.half
nX = 26;
x = array2D.pos.cascade.half(:, 1) - min(array2D.pos.cascade.half(:, 1)) + 1;
z = array2D.pos.cascade.half(:, 2) - min(array2D.pos.cascade.half(:, 2)) + 1;
array2D.iGlobal.cascade.half = (z - 1) * nX + x;

%% 删除无关变量并保存位置信息
clear x z nX
save .\signalProc\basicProc\angle\array2D.mat
