% 存储追踪绘图所需的Marker
% 作者: 刘涵凯
% 更新: 2023-7-25

%% 数字 
dirListNum=dir('./draw/tracking/picture/number'); % 数字图片文件夹
for iMarker = 1 : (length(dirListNum) - 2) / 3 % 注意有两个空文件
    [targetMarker.(strcat('num', num2str(iMarker))).confirm.marker, ~, targetMarker.(strcat('num', num2str(iMarker))).confirm.alpha] = imread([num2str(iMarker), '.png']);
    [targetMarker.(strcat('num', num2str(iMarker))).miss.marker, ~, targetMarker.(strcat('num', num2str(iMarker))).miss.alpha] = imread([num2str(iMarker), 'Pink.png']);
    [targetMarker.(strcat('num', num2str(iMarker))).lost.marker, ~, targetMarker.(strcat('num', num2str(iMarker))).lost.alpha] = imread([num2str(iMarker), 'Gray.png']);
end

%% 字母
dirListLtr=dir('./draw/tracking/picture/letter'); % 数字图片文件夹
letters = ["A"; "B"; "C"; "D"; "E"; "N"];
for iMarker = 1 : (length(dirListLtr) - 2) / 3 % 注意有两个空文件
    [targetMarker.(letters(iMarker)).confirm.marker, ~, targetMarker.(letters(iMarker)).confirm.alpha] = imread([char(letters(iMarker)), '.png']);
    [targetMarker.(letters(iMarker)).miss.marker, ~, targetMarker.(letters(iMarker)).miss.alpha] = imread([char(letters(iMarker)), 'Pink.png']);
    [targetMarker.(letters(iMarker)).lost.marker, ~, targetMarker.(letters(iMarker)).lost.alpha] = imread([char(letters(iMarker)), 'Gray.png']);
end

save('./draw/tracking/trackingMarker.mat', 'targetMarker')
