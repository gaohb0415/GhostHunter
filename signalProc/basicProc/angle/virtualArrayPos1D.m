function virtualArrayPos1D
% 计算一维虚拟阵列的相对位置
% 保存数据: 
% array1D
% - .pos: 虚拟阵列中各阵元的相对位置按位置顺序排列
% - .iSort: pos由Tx-Rx Channel顺序转换为位置顺序(含删除重叠阵元)的索引
% - .iUnit: pos由位置顺序转换为Tx-Rx Channel顺序的索引
% - .iGlobal: 阵元在补零后的阵列中的索引
% 作者: 刘涵凯
% 更新: 2024-3-1

%% 根据天线排布生成虚拟阵列位置
% 注意此时的array1D.pos是以Tx-Rx Channel顺序排列的
%% cascade
% cascade.half指启用2芯片(master + slave3)
% cascade.half.Tx1(仅使能master的Tx1)
array1D.pos.cascade.half.txSet1 = [0, 1, 2, 3, -11, -10, -9, -8]';
% cascade.half.Tx10~12(仅使能slave3的3个Tx)
array1D.pos.cascade.half.txSet2 = [0, 1, 2, 3, -11, -10, -9, -8]' + (-4) * (0 : 2);
array1D.pos.cascade.half.txSet2 = array1D.pos.cascade.half.txSet2(:);
% cascade.half.Tx1~3+10~12(使能全部6个Tx), 与case 3相同, 因为一维阵列不考虑master芯片的Tx
array1D.pos.cascade.half.txSet3 = [0, 1, 2, 3, -11, -10, -9, -8]' + (-4) * (0 : 2);
array1D.pos.cascade.half.txSet3 = array1D.pos.cascade.half.txSet3(:);
%% xWR1843
% 1T4R
array1D.pos.xwr1843.array1.txSet1 = [0, -1, -2, -3]';

%% 删除位置重叠的阵元, 将位置序列按位置顺序排序
%% cascade
% cascade.half
[array1D.pos.cascade.half.txSet1, array1D.iSort.cascade.half.txSet1, array1D.iUnit.cascade.half.txSet1] = unique(array1D.pos.cascade.half.txSet1);
[array1D.pos.cascade.half.txSet2, array1D.iSort.cascade.half.txSet2, array1D.iUnit.cascade.half.txSet2] = unique(array1D.pos.cascade.half.txSet2);
[array1D.pos.cascade.half.txSet3, array1D.iSort.cascade.half.txSet3, array1D.iUnit.cascade.half.txSet3] = unique(array1D.pos.cascade.half.txSet3);
% xWR1843
% 1T4R
[array1D.pos.xwr1843.array1.txSet1, array1D.iSort.xwr1843.array1.txSet1, array1D.iUnit.xwr1843.array1.txSet1] = unique(array1D.pos.xwr1843.array1.txSet1);

%% 真实阵元及虚拟阵元在虚拟阵列中的索引
%% cascade
% cascade.half. 仅txSet1需插值
array1D.iGlobal.cascade.half.txSet1.real = [1 : 4, 12 : 15];
array1D.iGlobal.cascade.half.txSet1.virtual = 1 : 15;

%% 保存位置信息
save .\signalProc\basicProc\angle\array1D.mat
