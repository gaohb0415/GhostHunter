% mmWaveMatlab主函数
% 作者: 刘涵凯
% 更新: 2024-6-22

%% 写入路径
close all; clear
addpath(genpath(pwd))

%% 载入雷达配置
config2243          % 载入雷达相关配置，并且载入雷达采集到的信号文件位置

%% 读取数据
iFrm = 115;
radarData= readBin(iFrm, 0); % 提取某帧，获得的radarData数据就是后面所有数据处理的起点


cfg.showRangeFFT        = 0;   % 是否显示 距离-FFT 图
cfg.showRangeDoppler    = 1;   % 是否显示 距离-多普勒 图
cfg.showRangeAngle      = 0;   % 是否显示 距离-角度 热力图
cfg.show2DTopDown       = 0;   % 是否显示 2D俯视图 (含真值)
cfg.show3DPointCloud    = 0;   % 是否显示 3D点云 图
cfg.show3DClusteredPC   = 0;   % 是否显示 3D点云聚类 图

radar_yaw_angle_deg = -30;  % 雷达旋转角度控制

% ROI设置
roi.range = [0, 8];        % 单位: 米
roi.angle = [-90, +90];    % 单位: 度

% 场景化真值设置
% 车辆
ground_truth_world.car.x = [ 1.65, 3.44, 3.44, 1.65, 1.65];
ground_truth_world.car.y = [ 0.63, 0.63, 3.48, 3.48, 0.63];
% 行人路径
ground_truth_world.path.x = [ 4.44, 0];
ground_truth_world.path.y = [ 4.98, 4.98];


%           定义并模拟来自LiDAR的先验信息 (ROI)
disp('--- 启用基于ROI的增强处理 ---');
fprintf('ROI范围: 距离 [%.1f, %.1f] m, 角度 [%d, %d] deg\n', ...
        roi.range(1), roi.range(2), roi.angle(1), roi.angle(2));



% =======================【场景化真值】=======================
disp('--- 加载场景化真值 (Ground Truth) ---');

% --- 遮挡车辆的边界框 (红框) ---
% --- 同学行走的路径 (红色虚线段) ---
% ====================================================================


fprintf('雷达水平旋转角度: %d deg\n', radar_yaw_angle_deg);
rotated_ground_truth = transform_ground_truth(ground_truth_world, radar_yaw_angle_deg);



%% 基本信号处理

% 测距（目标有多远？）
% drawEn：1.生成FFT图 0. 不生成
% pcEn：1. 在生成的FFT上面标注出点云信息 0.不标注
% 距离点云
[fftRsltRg, pcRg] = fftRange(radarData, 'pcEn', 1, 'drawEn', cfg.showRangeFFT); % Range FFT ，画出RangeFFT的波形图
% RD速度点云
[fftRsltRD, pcRD] = fftDoppler(fftRsltRg, 'pcEn', 1, 'drawEn', cfg.showRangeDoppler); % Doppler FFT

% 一维数字波束形成（DBF），用于测算物体的方位角（目标在那个方向？）
% 生成的是 Range-Angle Map 图像
% [pwRA, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', [0, 8], 'resAng', 1, 'drawEn', 1); % 1D DBF，画出的是物体的极坐标雷达图

% pcRA 角度、强度点云
[pwRA, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', roi.range, 'limitAng', roi.angle, 'resAng', 0.05, 'drawEn', cfg.showRangeAngle);
% [pwRAE, heatmapAE] = dbfProc2D(fftRsltRg, 'limitR', [2, 4], 'limitAz', [-30, 30], 'limitEl', [-30, 20], 'resAz', 0.25, 'resEl', 0.25); % 2D DBF
% [fftRsltAng1D, pcRA] = fftAngle1D(fftRsltRg, 'limitR', [0, 8], 'pcEn', 0, 'drawEn', 1); % 1D Angle FFT
% [fftRsltAng2D, heatmapAE] = fftAngle2D(fftRsltRg, 'limitR', [3.8, 4.6], 'drawEn', 1); % 2D Angle FFT

%% 整合信号处理
% 点云生成

% 2D点云俯视图生成模块

if cfg.show2DTopDown
    generate2DTopDownView(5, pcRA, rotated_ground_truth, roi, iFrm);
    drawnow;
end

% ========================================================================

% 4D-FFT生成三维点云（距离、速度、水平角度、垂直角）
pc3D = pcFrom4DFFT(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakAz', 1, 'nPeakEl', 3, 'drawEn', cfg.show3DPointCloud); % 4D FFT

% pc3D = pcFrom2PassDBF(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakEl', 1, 'drawEn', 1); % 2-Pass DBF

% 点云聚类
clusterXY = pcCluster2D([pcRA.x, pcRA.y], 'pw', [], 'limitXV', [-3.2, 3.2], 'limitY', [1.6, 6.4], 'drawEn', 0); % XY点云聚类
% clusterVY = pcCluster2D([pcRD.velocity, pcRD.range], 'pcType', 'VY', 'pw', [], 'limitY', [1.6, 6.4], 'drawEn', 1); % XV点云聚类

clusterXYZ= pcCluster3D([pc3D.x, pc3D.y, pc3D.z], 'pw', [], 'vel', pc3D.vel, 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZV', [0, 2], 'drawEn', cfg.show3DClusteredPC); % XYZ点云聚类
% clusterXYV = pcCluster3D([pc3D.x, pc3D.y, pc3D.vel], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZV', [-2, 2], 'pcType', 'XYV'); % XYV点云聚类


% 微多普勒
% mdRslt = microDoppler;

%% 感知
% armInfo = armMotionDetection('limitT', [0.7, 10.3]); % 手臂运动探测
% stm = voiceSpectrum('limitT', [0.5, 6], 'nFrmWindow', 4); % 声谱生成
% [rateHb, rateResp] = vitalSignsDetection('locMode', 'range', 'limitT', [0, 20]); % 生命体征监测
% tracking2D % 追踪
