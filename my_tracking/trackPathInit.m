function trackPathInit(iSetup, iCount)
% 路径设置
% 23年11月的目录格式跟之前不一样
% 输入:
% 1. iSetup: 实验场景索引
% 2. iCount: 实验次数索引
% 作者: 刘涵凯
% 更新: 2023-3-6

%% 参数对象
p = trackParamShare.param;

%% 雷达原始数据路径
binFilePath = ['G:\radarData\23.4.24\Track', num2str(iSetup), '\', num2str(iCount)]; %  G:\radarData\23.11.20\track\track
binFileNameMaster = 'master_0000_data.bin';
binFileNameSlave3 = 'slave3_0000_data.bin';
binFileHandleMaster = [binFilePath, '\', binFileNameMaster];
binFileHandleSlave3 = [binFilePath, '\', binFileNameSlave3];
binFileHandles = [binFileHandleMaster; binFileHandleSlave3];
% 在config.mat中更新路径
save('config.mat', '-append', 'binFileHandles');

%% 数据存储路径
% 根据实验次数索引设置数据存储路径
iExp = [num2str(iSetup), '-', num2str(iCount)];
% 雷达聚类数据
p.handleData = ['G:\radarData\23.4.24\clusterResults\track', iExp, '.mat'];
% IMU数据
p.handleDataImu = ['G:\radarData\23.4.24\clusterResults\IMU', iExp, '.mat'];
% 追踪结果
p.handleTrkRslt = '.\sensing\tracking\data\trackingResults\trackingResults.mat';
