function p = simParamConfig
% 仿真参数设置
% 输出:
% p: 参数对象
% 作者: 刘涵凯
% 更新: 2024-11-29

%% 参数对象
p = simParamShare.param;

%% 路径设置
p.handleSMPL = 'G:\radarData\simulation\SMPL\';                                              % SMPL模型路径
p.handleTrajectory = 'G:\radarData\simulation\trajectory\trajRecord.mat';         % 轨迹路径
p.handleRemakeTrajectory = 'G:\radarData\simulation\trajRemake\remakedTrajs1.mat';  % 重组轨迹路径
p.handleResp = 'G:\radarData\simulation\resp\curveLib\segLib\measured\segLib.mat';  % 呼吸曲线路径
p.handleSimSignal = 'G:\radarData\simulation\test\';                                          % 仿真信号存储路径

%% 人体模型
p.smthTimeWin = 0.4;         % 网格平滑的时间窗尺寸, 双边 s
p.smthTimeLink = 0.1;         % 连接处被平滑范围, 单边 s
p.smthTimeWinLink = 0.2;   % 连接处平滑窗尺寸, 双边 s
% 人体部位反射强度调整
% p.segPw = ones(24, 1);
p.segPw([5, 20]) = 0; % 脚趾 0
p.segPw([6, 11]) = 0.2; % 脚 0.2 
p.segPw([14, 16]) = 0.2; % 手掌 0.2
p.segPw([1, 23]) = 0.2; % 手腕 0.2
p.segPw([7, 8, 12, 19, 21, 24]) = 1; % 躯干 1
p.segPw([9, 10]) = 0.8; % 肩 0.8
p.segPw([2, 22]) = 0.4; % 大腿 0.8
p.segPw([3, 13]) = 0.4; % 大臂 0.4
p.segPw([4, 15]) = 0.4; % 小腿 0.4
p.segPw([17, 18]) = 0.4; % 小臂 0.4

%% 雷达参数
% 采样
p.s = 99e12;                 % 斜率 Hz/s
p.tAdcStart = 3e-6;      % 开始升频但还未采样的时间
p.fStart = 77e9 + p.tAdcStart * p.s; % 实际起始频率
p.nAdc1Chirp = 256;    % 一个chirp的ADC采样次数
p.adcRate = 7.04e6;     % ADC采样频率
p.nChirp1Frm = 128;    % 一帧中各Tx发射的chrip的个数
p.tChirp = 50e-6;         % 一个chirp的占用时间
p.tFrm = 50e-3;            % 帧周期
p.nFrm = 201;              % 帧数
% 属性
p.tRamp = p.nAdc1Chirp / p.adcRate;                                 % 有效扫频时间
p.fCenter = p.fStart + p.nAdc1Chirp / 2 /  p.adcRate * p.s; % 实际中心频率
p.lambda = physconst('LightSpeed') / p.fCenter;                % 天线阵列中的lambda是固定值, 而此处可变
p.bw = p.tRamp * p.s;                                                          % 有效带宽
p.adcSlot = (0 : p.nAdc1Chirp - 1) / p.adcRate;                   % ADC采样时隙
p.fFrm = 1 / p.tFrm;                                                             % 帧频率
% 功率
p.friisFactor = sqrt(3.5);  % 括号内为Friis公式的路径衰减系数
p.txAmpl = 100e6;          % 用于调整幅度 自行调整 无单位 全1-30e6 调整-35e6 骨骼默认-14 调整-18/16 RAM: smplAll: 100e6 smplRaw: 3e6 skelRaw: 1.5e6 skelAll: 90e6

%% 天线阵列
p.posRadar = [0, 0, 1.15]; % 雷达位置
arrayWaveLen = physconst('LightSpeed') / 77e9; % 板子的阵列是按77GHz的波长设计的
% 以半波长为单位的阵列排布
% cascade.half
txX = 15.518 + [0, -1, -2, -3, -7, -11]';
txZ = 32.144 + [0, 2, 5, 6, 6, 6]';
rxX = [0, 1, 2, 3, -11, -10, -9, -8]';
rxZ = zeros(8, 1);
% 转换单位为m
p.posArrayTx = arrayWaveLen / 2 * [txX, txZ];
p.posArrayRx = arrayWaveLen / 2 * [rxX, rxZ];
% 启用顺序
p.orderTx = 1 : 6;
p.orderRx = 1 : 8;
p.nTx = length(p.orderTx);                 % 启用的Tx数
p.nRx = length(p.orderRx);                % 启用的Rx数
p.nTransmit = p.nTx * p.nChirp1Frm; % 一帧中发射的chirp数
% 计算中心位置并做差分
center = mean([p.posArrayTx(p.orderTx, :); p.posArrayRx(p.orderRx, :)]);
p.posTx = p.posArrayTx(p.orderTx, :) - center;    % Tx坐标
p.posRx = p.posArrayTx(p.orderTx, :) - center;    % Rx坐标

%% 轨迹重组
p.nFrmRmk = p.nFrm + 6;  % 重组轨迹时间长度 帧
p.k = 10;                              % 每k帧执行一次重组判定
p.NRp = 500;                      % 最大连续复制次数

%% 信道失配
p.channelMismatchEn = 1; % 开关
% 频率偏差
rangeError = 0.2 * randn(p.nTx + p.nRx, 1);
freqShift = 2 * pi * rangeError * (1 : p.nAdc1Chirp) ./ p.nAdc1Chirp;
% 信道幅度偏差
amplShift = 1 + 0.1 * randn(p.nTx + p.nRx, 1);
% 信道相位偏差
phaseShift = 0.15 * randn(p.nTx + p.nRx, 1);
% 整体信道失配
channelMismatch = repmat(amplShift .* exp(1i * phaseShift), 1, p.nAdc1Chirp) .* exp(1i * freqShift);
channelMismatchRx = channelMismatch(1 : p.nRx, :);
channelMismatchTx = channelMismatch(p.nRx + 1 : end, :);
channelMismatchRx = repmat(channelMismatchRx, p.nTx, 1);
channelMismatchTx = repelem(channelMismatchTx, p.nRx, 1);
p.channelMismatch = channelMismatchRx .* channelMismatchTx;
% 参数存储及载入
% channelMismatch = p.channelMismatch; save('.\sensing\simulation\initialization\channelParam.mat', 'channelMismatch');
load('.\sensing\simulation\initialization\channelParam.mat'); p.channelMismatch = channelMismatch;

%% 多径
p.segPwGhost = ones(12, 1); % 鬼影各部位反射强度调整
p.amplGhostFac = 5; % 鬼影反射信号幅度调整
% 反射面
p.reflector = [];
% 地面
% p.reflector(end + 1).vert = [3.6, 0, 0; 3.6, 8, 0; -3.6, 8, 0; -3.6, 0, 0];
% p.reflector(end).atten = 0.2; 
% % 天花板
% p.reflector(end + 1).vert = [3.6, 0, 4; 3.6, 8, 4; -3.6, 8, 4; -3.6, 0, 4];
% p.reflector(end).atten = 0.3;
% %
% p.reflector(end + 1).vert = [-3.6, 5, 4; -3.6, 6, 3; 3.6, 6, 3; 3.6, 5, 4];
% p.reflector(end).atten = 0.2;
% 后墙
% p.reflector(end + 1).vert = [3.6, 8, 0; 3.6, 8, 4; -3.6, 8, 4; -3.6, 8, 0];
% p.reflector(end).atten = 0.6;
% 侧墙
p.reflector(end + 1).vert = [3.6, 0, 0; 3.6, 8, 0; 3.6, 8, 4; 3.6, 0, 4];
p.reflector(end).atten = 0.7; 
% 弱鬼影判定
p.ghostWeakRatio = 0.05;   % 删除幅度低于最强鬼影点幅度*p.ghostWeakRatio的鬼影点
p.ghostNumLmt = 500;       % 每个目标最多生成的鬼影点数

%% 呼吸编码
p.distAmplCorrect = 1.2;       % 位移补偿倍数
p.respDistMean = 0.01;         % 峰峰值 m
p.respDistStd = 0.001;           % m
p.respCycleMean = 3;            % 呼吸周期均值 s
p.respCycleStd = p.respCycleMean / 20; % 呼吸周期标准差 s
p.respCycleGuard = 0.15;      % 呼吸编码保护区 s 单边
p.code = [1, 0, 1, 1];               % 身份码
p.codeCycle = [2.85, 3.35];    % 0和1对应的呼吸周期 s
p.codeRedun = ceil(length(p.code) / 3); % 冗余位个数
p.codePeriod = 20;                % 编码出现周期 单位为周期
p.decodeCycleWin = 0.15;     % 呼吸解码有效区 s 单边
