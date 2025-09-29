% 将IMU数据进行重命名和地址转移
% 作者: 刘涵凯
% 更新: 2022-8-28

pathDesk = "C:\Users\liuha\Desktop\"; % 原地址
pathIMU = "C:\Users\liuha\Desktop\IMU\"; % 目标地址
handleData = "1.csv"; % 原文件名

iTrack = 1;
for iCount = 1 : 10
    source = fullfile(pathDesk, handleData);
    destination = fullfile(pathIMU, ['Track-', int2str(iTrack), '-Count-', int2str(iCount), '.csv']);
    copyfile(source, destination);
    disp([int2str(iTrack), '-', int2str(iCount), '记录完成']);
    pause(2)
end
