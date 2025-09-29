% IMU数据载入及预处理
% 作者: 刘涵凯
% 更新: 2023-5-22

%% 时间校准
timeShift = 0; % IMU时间比雷达时间快timeShift秒
rateImu = 100; % Hz

%% 数据载入及处理
for iTrack = 17
    for iCount = 9
        close all
        clear dataPerDev
        % 数据地址
        handleData = ['G:\radarData\23.4.24\IMU\Track-', int2str(iTrack), '-Count-', int2str(iCount), '.csv'];
        % 数据载入
        dataRaw = readtable(handleData, 'ReadVariableNames',false);
        % 提取设备索引
        dataDevIdx = table2array(dataRaw(:, 1));
        % 提取各设备的数据
        devUnique = unique(dataDevIdx);
        for iDev = 1 : length(devUnique)
            dataPerDev(iDev).id = devUnique(iDev);
            dataPerDev(iDev).data = table2array(dataRaw(dataDevIdx == devUnique(iDev), 2 : end));
            time = dataPerDev(iDev).data(:, 1);
        end
        % 保存处理结果
        save(['G:\radarData\23.4.24\clusterResults\IMU', int2str(iTrack), '-', int2str(iCount), '.mat'], 'dataPerDev', 'rateImu')
    end
end
