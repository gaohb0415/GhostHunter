% 绘制IMU的加速度、角速度、四元数、欧拉角
% 作者: 刘涵凯
% 更新: 2023-5-22

clear; close all

%% 绘图参数设置
lineWidth = 2;
fontSize = 10;
basePos = [1.4 1.2 12 1.4];
height = 1.8;
gcfPos = [1, 5, 13.6, 6.35];

%% 载入IMU数据
load colorLib.mat
handleData = 'G:\radarData\23.4.24\IMU\Track-13-Count-3.csv';
data = readtable(handleData, 'ReadVariableNames',false);

%% IMU数据提取
data = table2array(data);
data = data(data(:, 1) == 1, :); % 选择IMU编号
time = data(:, 2);
g = 9.80; % 重力加速度
accX =data(:, 4) * g;
accY = data(:, 5) * g;
accZ = data(:, 6) * g;
gyroX = data(:, 7);
gyroY = data(:, 8);
gyroZ = data(:, 9);
eulX = data(:, 13);
eulY = data(:, 14);
eulZ = data(:, 15);
qW = data(:, 16);
qX = data(:, 17);
qY = data(:, 18);
qZ = data(:, 19);

%% 加速度
figure
% X
h1 = subplot(3, 1, 1); 
hx = plot(time, accX);
ylabel('X (m^2/s)')
% Y
h2 = subplot(3, 1, 2); 
hy = plot(time, accY);
ylabel('Y (m^2/s)')
% Z
h3 = subplot(3, 1, 3); 
hz = plot(time, accZ);
ylabel('Z (m^2/s)')
% 设置线条
set(hx, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorBlue)
set(hy, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorOrange)
set(hz, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorYellow)
% 整体
xlabel('Time (s)')
set(gcf, 'color', 'w')
set(gca, 'Fontsize', fontSize)
% 设置子图位置
set(h1, 'Units', 'centimeters', 'position', basePos + [0, 2 * height, 0, 0], 'Xlim', [0, time(end)])
set(h2, 'Units', 'centimeters', 'position', basePos + [0, height, 0, 0], 'Xlim', [0, time(end)])
set(h3, 'Units', 'centimeters', 'position', basePos, 'Xlim', [0, time(end)])
set(h1,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h2,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h3, 'Box', 'off', 'Fontsize', fontSize)
h1.XAxis.Visible = 'off';   % 默认属性 on 表明可见
h2.XAxis.Visible = 'off';   % 默认属性 on 表明可见
set(gcf, 'Units', 'centimeters', 'position', gcfPos)

%% 角速度
figure
% X
h1 = subplot(3, 1, 1); 
hx = plot(time, gyroX);
ylabel('X (°/s)')
% Y
h2 = subplot(3, 1, 2); 
hy = plot(time, gyroY);
ylabel('Y (°/s)')
% Z
h3 = subplot(3, 1, 3); 
hz = plot(time, gyroZ);
ylabel('Z (°/s)')
% 设置线条
set(hx, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorBlue)
set(hy, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorOrange)
set(hz, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorYellow)
% 整体
xlabel('Time (s)')
set(gcf, 'color', 'w')
set(gca, 'Fontsize', fontSize)
% 设置子图位置
set(h1, 'Units', 'centimeters', 'position', basePos + [0, 2 * height, 0, 0], 'Xlim', [0, time(end)])
set(h2, 'Units', 'centimeters', 'position', basePos + [0, height, 0, 0], 'Xlim', [0, time(end)])
set(h3, 'Units', 'centimeters', 'position', basePos, 'Xlim', [0, time(end)])
set(h1,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h2,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h3, 'Box', 'off', 'Fontsize', fontSize)
h1.XAxis.Visible = 'off';   % 默认属性 on 表明可见
h2.XAxis.Visible = 'off';   % 默认属性 on 表明可见
set(gcf, 'Units', 'centimeters', 'position', gcfPos)

%% 欧拉角
figure
% X
h1 = subplot(3, 1, 1); 
hx = plot(time, eulX);
ylabel('Roll (°)')
% Y
h2 = subplot(3, 1, 2); 
hy = plot(time, eulY);
ylabel('Pitch (°)')
% Z
h3 = subplot(3, 1, 3); 
hz = plot(time, eulZ);
ylabel('Yaw (°)')
% 设置线条
set(hx, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorBlue)
set(hy, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorOrange)
set(hz, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorYellow)
% 整体
xlabel('Time (s)')
set(gcf, 'color', 'w')
set(gca, 'Fontsize', fontSize)
% 设置子图位置
set(h1, 'Units', 'centimeters', 'position', basePos + [0, 2 * height, 0, 0], 'Xlim', [0, time(end)])
set(h2, 'Units', 'centimeters', 'position', basePos + [0, height, 0, 0], 'Xlim', [0, time(end)])
set(h3, 'Units', 'centimeters', 'position', basePos, 'Xlim', [0, time(end)])
set(h1,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h2,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h3, 'Box', 'off', 'Fontsize', fontSize)
h1.XAxis.Visible = 'off';   % 默认属性 on 表明可见
h2.XAxis.Visible = 'off';   % 默认属性 on 表明可见
set(gcf, 'Units', 'centimeters', 'position', gcfPos)

%% 四元数
figure
% W
h1 = subplot(4, 1, 1); 
hw = plot(time, qW);
ylabel('W')
% X
h2 = subplot(4, 1, 2); 
hx = plot(time, qX);
ylabel('X')
% Y
h3 = subplot(4, 1, 3); 
hy = plot(time, qY);
ylabel('Y')
% Z
h4 = subplot(4, 1, 4); 
hz = plot(time, qZ);
ylabel('Y')
% 设置线条
set(hw, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorBlue)
set(hx, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorOrange)
set(hy, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorYellow)
set(hz, 'LineStyle', '-', 'LineWidth', lineWidth, 'Color', colorGreen)
% 整体
xlabel('Time (s)')
set(gcf, 'color', 'w')
set(gca, 'Fontsize', fontSize)
% 设置子图位置
basePos = [1.4 1.2 12 1.4/4*3];
height = 1.8/4*3;
gcfPos = [1, 5, 13.6, 6.35];
set(h1, 'Units', 'centimeters', 'position', basePos + [0, 3 * height, 0, 0], 'Xlim', [0, time(end)])
set(h2, 'Units', 'centimeters', 'position', basePos + [0, 2 * height, 0, 0], 'Xlim', [0, time(end)])
set(h3, 'Units', 'centimeters', 'position', basePos + [0, height, 0, 0], 'Xlim', [0, time(end)])
set(h4, 'Units', 'centimeters', 'position', basePos, 'Xlim', [0, time(end)])
set(h1,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h2,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h3,'xticklabel',[], 'Box', 'off', 'Fontsize', fontSize)
set(h4, 'Box', 'off', 'Fontsize', fontSize)
h1.XAxis.Visible = 'off';   % 默认属性 on 表明可见
h2.XAxis.Visible = 'off';   % 默认属性 on 表明可见
h3.XAxis.Visible = 'off';   % 默认属性 on 表明可见
set(gcf, 'Units', 'centimeters', 'position', gcfPos)
