% mmWaveMatlab主函数
% 作者: 刘涵凯
% 更新: 2024-6-22

%% 写入路径
close all; clear
addpath(genpath(pwd))

%% 载入雷达配置
config2243          % 载入雷达相关配置，并且载入雷达采集到的信号文件位置

%% 读取数据
iFrm = 46;
radarData= readBin(iFrm, 0); % 提取某帧，获得的radarData数据就是后面所有数据处理的起点

% ========================================================================
%           定义并模拟来自LiDAR的先验信息 (ROI)
% ========================================================================
% 假设LiDAR模块告诉我们，"鬼探头"最可能发生在车头左前方
% 距离雷达 2.5米 到 3.5米，角度在 -30度 到 -10度 的区域内。
disp('--- 启用基于ROI的增强处理 ---');
roi.range = [0, 8]; % 单位: 米
roi.angle = [-90, +90];    % 单位: 度
fprintf('ROI范围: 距离 [%.1f, %.1f] m, 角度 [%d, %d] deg\n', ...
        roi.range(1), roi.range(2), roi.angle(1), roi.angle(2));
% ========================================================================

% =======================【场景化真值】=======================
disp('--- 加载场景化真值 (Ground Truth) ---');

% --- 遮挡车辆的边界框 (红框) ---
% 注意：为了画一个闭合的矩形，我们需要5个点（最后一个点和第一个点重合）
ground_truth.car.x = [ 1.0, 2.5, 2.5, 1.0, 1.0];
ground_truth.car.y = [ 2.0, 2.0, 4.5, 4.5, 2.0];

% --- 同学行走的路径 (红色虚线段) ---
ground_truth.path.x = [ 1.0, 2.5];
ground_truth.path.y = [ 4.75, 4.75];
% ====================================================================

%% 基本信号处理

% 测距（目标有多远？）
% drawEn：1.生成FFT图 0. 不生成
% pcEn：1. 在生成的FFT上面标注出点云信息 0.不标注
% 距离点云
[fftRsltRg, pcRg] = fftRange(radarData, 'pcEn', 1, 'drawEn', 1); % Range FFT ，画出RangeFFT的波形图
% RD速度点云
[fftRsltRD, pcRD] = fftDoppler(fftRsltRg, 'pcEn', 1, 'drawEn', 1); % Doppler FFT

% 一维数字波束形成（DBF），用于测算物体的方位角（目标在那个方向？）
% 生成的是 Range-Angle Map 图像
% [pwRA, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', [0, 8], 'resAng', 1, 'drawEn', 1); % 1D DBF，画出的是物体的极坐标雷达图

% pcRA 角度、强度点云
[pwRA, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', roi.range, 'limitAng', roi.angle, 'resAng', 0.05, 'drawEn', 1,'cfarPfa',0.25);
% [pwRAE, heatmapAE] = dbfProc2D(fftRsltRg, 'limitR', [2, 4], 'limitAz', [-30, 30], 'limitEl', [-30, 20], 'resAz', 0.25, 'resEl', 0.25); % 2D DBF
% [fftRsltAng1D, pcRA] = fftAngle1D(fftRsltRg, 'limitR', [0, 8], 'pcEn', 0, 'drawEn', 1); % 1D Angle FFT
% [fftRsltAng2D, heatmapAE] = fftAngle2D(fftRsltRg, 'limitR', [3.8, 4.6], 'drawEn', 1); % 2D Angle FFT

%% 整合信号处理
% 点云生成



% ========================================================================
%           2D俯视图生成模块 (适配V1.0绘图函数的版本)
% ========================================================================

figure(5); % 1. 明确操作5号窗口
clf;       % 2. 清空窗口内容
hold on;   % 3. 准备叠加绘制所有元素

% --- 步骤 A: 绘制真值作为静态背景 ---
% (确保 ground_truth 结构体已在脚本前面定义)
plot(ground_truth.car.x, ground_truth.car.y, 'r-', 'LineWidth', 2);
plot(ground_truth.path.x, ground_truth.path.y, 'r--', 'LineWidth', 2);

% --- 步骤 B: 在真值背景上，调用V1.0版本的绘图函数 ---
% (确保 pcRA 和 roi 变量已经存在)
if exist('pcRA', 'var') && ~isempty(pcRA.x)
    clusterRslt2D = pcCluster2D([pcRA.x, pcRA.y], 'pw', pcRA.power, 'drawEn', 0);
    
    % [核心修改] 调用函数时不接收任何输出
    drawPointsOnExistingAxes(clusterRslt2D.pcInput, ...
                 'clusterID', clusterRslt2D.clusterIdx, ...
                 'power', clusterRslt2D.pw, ...
                 'roi', roi); 
else 
    % [核心修改] 调用函数时不接收任何输出
    drawPointsOnExistingAxes([], 'roi', roi); 
    disp('当前帧未检测到2D点云。');
end

% --- 步骤 C: 美化图像并创建静态图例 ---
plot(0, 0, 'kv', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % 绘制雷达
hold off;
grid on;
axis equal; 
xlim([-4, 4]);
ylim([0, 8]);
xlabel('X (m)');
ylabel('Y (m)');

% [核心修改] 使用一个固定的静态图例，因为我们没有从函数获得动态信息
legend('车辆边界', '行人路径', '雷达', 'ROI', '检测点', 'Location', 'best');
       
title(['二维俯视点云图 (帧: ', num2str(iFrm), ')']);
drawnow;
% ========================================================================

% 4D-FFT生成三维点云（距离、速度、水平角度、垂直角）
pc3D = pcFrom4DFFT(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakAz', 1, 'nPeakEl', 3, 'drawEn', 0); % 4D FFT

% pc3D = pcFrom2PassDBF(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakEl', 1, 'drawEn', 1); % 2-Pass DBF

% 点云聚类
% clusterXY = pcCluster2D([pcRA.x, pcRA.y], 'pw', [], 'limitXV', [-3.2, 3.2], 'limitY', [1.6, 6.4], 'drawEn', 1); % XY点云聚类
% clusterVY = pcCluster2D([pcRD.velocity, pcRD.range], 'pcType', 'VY', 'pw', [], 'limitY', [1.6, 6.4], 'drawEn', 1); % XV点云聚类

clusterXYZ= pcCluster3D([pc3D.x, pc3D.y, pc3D.z], 'pw', [], 'vel', pc3D.vel, 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZV', [0, 2], 'drawEn', 0); % XYZ点云聚类
% clusterXYV = pcCluster3D([pc3D.x, pc3D.y, pc3D.vel], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZV', [-2, 2], 'pcType', 'XYV'); % XYV点云聚类


% 微多普勒
% mdRslt = microDoppler;

%% 感知
% armInfo = armMotionDetection('limitT', [0.7, 10.3]); % 手臂运动探测
% stm = voiceSpectrum('limitT', [0.5, 6], 'nFrmWindow', 4); % 声谱生成
% [rateHb, rateResp] = vitalSignsDetection('locMode', 'range', 'limitT', [0, 20]); % 生命体征监测
% tracking2D % 追踪
