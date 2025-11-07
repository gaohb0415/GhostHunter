function idx = arrayInIntvl(array, intvl)
% 判断数组中有哪些元素的大小在设定范围内
% 输入: 
% 1. array: 一维数组
% 2. intvl: 设定范围, 可同时设定多组范围, 组成N*2的矩阵
% 输出: 
% idx: 在设定范围内的元素的索引
% 作者: 刘涵凯
% 更新: 2024-3-14

idx = zeros(size(array));
for iIntvl = 1 : size(intvl, 1)
    idx = idx | (array >= intvl(iIntvl, 1) & array <= intvl(iIntvl, 2));
end
