function ptDs = pcDsFast(pt, gridStep)
% 调用pcdownsample, 以gridAverage模式进行降采样
% 输入:
% 1. pt: 待降采样点
% 2. gridStep: 降采样网格大小
% 输出:
% ptDs: 降采样后的点
% 作者: 刘涵凯
% 更新: 2024-3-26

pc = pointCloud(pt);
pcDs = pcdownsample(pc, 'gridAverage', gridStep);
ptDs = pcDs.Location;
