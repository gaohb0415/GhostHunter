% mmWaveMatlab主函数
% 作者: 刘涵凯
% 更新: 2024-6-22

%% 写入路径
close all; clear
addpath(genpath(pwd))

%% 载入雷达配置
config2243          % 载入雷达相关配置，并且载入雷达采集到的信号文件位置

%% 读取数据
iFrm = 50;
radarData= readBin(iFrm, 0); % 提取某帧，获得的radarData数据就是后面所有数据处理的起点

%% 基本信号处理

% 测距（目标有多远？）
% drawEn：1.生成FFT图 0. 不生成
% pcEn：1. 在生成的FFT上面标注出点云信息 0.不标注
[fftRsltRg, pcRg] = fftRange(radarData, 'pcEn', 1, 'drawEn', 1); % Range FFT ，画出RangeFFT的波形图
% [fftRsltRD, pcRD] = fftDoppler(fftRsltRg, 'pcEn', 0, 'drawEn', 0); % Doppler FFT

% 一维数字波束形成（DBF），用于测算物体的方位角（目标在那个方向？）
% 生成的是 Range-Angle Map 图像
[pwRA, pcRA] = dbfProc1D(fftRsltRg, 'pcEn', 1, 'limitR', [0, 8], 'resAng', 1, 'drawEn', 1); % 1D DBF，画出的是物体的极坐标雷达图
% [pwRAE, heatmapAE] = dbfProc2D(fftRsltRg, 'limitR', [2, 4], 'limitAz', [-30, 30], 'limitEl', [-30, 20], 'resAz', 0.25, 'resEl', 0.25); % 2D DBF
% [fftRsltAng1D, pcRA] = fftAngle1D(fftRsltRg, 'limitR', [0, 8], 'pcEn', 0, 'drawEn', 1); % 1D Angle FFT
% [fftRsltAng2D, heatmapAE] = fftAngle2D(fftRsltRg, 'limitR', [3.8, 4.6], 'drawEn', 1); % 2D Angle FFT

%% 整合信号处理
% 点云生成

% ========================================================================
%           2D俯视图生成模块 (调用入口 - 保证每帧都绘图)
% ========================================================================

% --- 步骤 1: 检查是否存在由dbfProc1D生成的原始2D点云(pcRA) ---
if exist('pcRA', 'var') && ~isempty(pcRA.x)
    
    % --- 步骤 2: 如果存在点云, 则进行聚类 ---
    % 'drawEn', 0 表示只进行计算，不画默认的图
    clusterRslt2D = pcCluster2D([pcRA.x, pcRA.y], 'pw', pcRA.power, 'drawEn', 0);

    % --- 步骤 3: 调用绘图函数, 传入聚类结果 ---
    % 即使聚类失败, clusterRslt2D.pcInput 也会包含原始点云
    drawPc2DPlus(clusterRslt2D.pcInput, ...
                 'clusterID', clusterRslt2D.clusterIdx, ...
                 'power', clusterRslt2D.pw, ...
                 'limitX', [-4, 4], ...
                 'limitY', [0, 8]);
else
    % --- 步骤 4: 如果连原始2D点云都没有, 则调用绘图函数并传入空数据 ---
    disp('当前帧未检测到原始2D点云，生成空白图。');
    drawPc2DPlus([], ...
                 'limitX', [-4, 4], ...
                 'limitY', [0, 8]);
end



% 4D-FFT生成三维点云（距离、速度、水平角度、垂直角）
pc3D = pcFrom4DFFT(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakAz', 1, 'nPeakEl', 3, 'drawEn', 1); % 4D FFT

% pc3D = pcFrom2PassDBF(radarData, 'limitR', [0.8, 7.2], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZ', [0, 2], 'nPeakEl', 1, 'drawEn', 1); % 2-Pass DBF

% 点云聚类
% clusterXY = pcCluster2D([pcRA.x, pcRA.y], 'pw', [], 'limitXV', [-3.2, 3.2], 'limitY', [1.6, 6.4], 'drawEn', 1); % XY点云聚类
% clusterVY = pcCluster2D([pcRD.velocity, pcRD.range], 'pcType', 'VY', 'pw', [], 'limitY', [1.6, 6.4], 'drawEn', 1); % XV点云聚类

clusterXYZ= pcCluster3D([pc3D.x, pc3D.y, pc3D.z], 'pw', [], 'vel', pc3D.vel, 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZV', [0, 2], 'drawEn', 1); % XYZ点云聚类
% clusterXYV = pcCluster3D([pc3D.x, pc3D.y, pc3D.vel], 'limitX', [-3, 3], 'limitY', [1.2, 6.8], 'limitZV', [-2, 2], 'pcType', 'XYV'); % XYV点云聚类


% 微多普勒
% mdRslt = microDoppler;

%% 感知
% armInfo = armMotionDetection('limitT', [0.7, 10.3]); % 手臂运动探测
% stm = voiceSpectrum('limitT', [0.5, 6], 'nFrmWindow', 4); % 声谱生成
% [rateHb, rateResp] = vitalSignsDetection('locMode', 'range', 'limitT', [0, 20]); % 生命体征监测
% tracking2D % 追踪
