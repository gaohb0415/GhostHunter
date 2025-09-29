function config2243
% 雷达配置、雷达参数计算、信号处理参数设置
% 适用于MMWCAS
% 作者: 刘涵凯
% 更新: 2023-8-28

%% 雷达型号
device = '2243';

%% 阵列类型
% 'all'为4芯片, 'half'为2芯片(master + slave3)
arrayType = 'half';
% 这里的芯片使用的是master与slave3号芯片，级联模式

%% 雷达数据路径
binFileNameMaster = 'master_0000_data.bin';
binFileNameSlave3 = 'slave3_0000_data.bin';
% binFilePath = 'C:\Users\liuha\Desktop\20250801meeting\data\1user';
% binFilePath = 'C:\Users\liuha\Desktop\20250801meeting\data\2user';
%% 雷达读取数据的文件夹路径
binFilePath = 'E:\20250801meeting\data\20250919\ping';
binFileHandleMaster = [binFilePath, '\', binFileNameMaster];
binFileHandleSlave3 = [binFilePath, '\', binFileNameSlave3];
binFileHandles = [binFileHandleMaster; binFileHandleSlave3];

%% 使能天线
%% 表示雷达架设在距离地面1.15米的位置
%% posRadar 的高度值需要根据你实验时的实际架设高度来修改，这会直接影响目标高度的解算是否准确
posRadar = [0, 0, 0.68]; % 雷达位置. 目前仅高度设置有用, 不要修改x和y. 雷达朝向为y轴正向
% 原1.15

fSpacing = 77e9;           % 天线阵列排布所依据的频率
nTx = 6;                         % 发射天线个数
nRx = 8;                         % 接收天线个数


%% 采样参数
fStart = 77e9;            % 起始频率 77GHz
nAdc1Chirp = 512;    % 一个chirp的ADC采样次数
adcRate = 7.04e6;     % ADC采样频率
nChirp1Frm = 256;    % 一帧中各Tx发射的chrip的个数

% Chirp的调频斜率 (Hz/s)。这是一个极其重要的参数，它和采样率共同决定了最大探测距离
s = 120e12; 

tIdle = 10e-6;            % chirp间的空闲时间
tAdcStart = 3e-6;      % 开始升频但还未采样的时间
tRampEnd = 40e-6;  % 整个扫频时间
tTxStart = 1e-6;         % Tx启动时间
tFrm = 100e-3;            % 帧周期,每隔100毫秒就采集一帧的数据，雷达刷新率为10Hz(1/0.1s)



%% 计算参数
%% 这部分计算代码不是用来更改的，是通过采集完的数据发现了问题之后
%% 通过以下衍生参数的异常反推上面采样参数的异常的，然后更改上面的采样参数
c = physconst('LightSpeed');       % 光速
lambda = c / fStart;                     % 波长
bw = nAdc1Chirp / adcRate * s;  % 有效带宽
bwTotal = s * tRampEnd;             % 总带宽
fCenter = fStart + s * tAdcStart + bw / 2; % 实际中心频率
spacingCal = fCenter / fSpacing; % 天线阵列排布所依据的频率/实际中心频率
fFrm = 1 / tFrm;                           % 帧频率
tChirp= tIdle + tRampEnd;          % 一个chirp的占用时间
tChirpIntvl = tChirp * nTx;           % 每个Tx发射两个chirp的间隔(即chirp周期)
tChirp1Frm = tChirpIntvl * nChirp1Frm; % 一帧中的总chirp时长 
data1Frm = nTx * nRx * nAdc1Chirp * nChirp1Frm * 2 * 2 / 1024 / 1024; % 每帧数据量 MB
nFrm1File = floor(2047 / data1Frm * 2); % 一个文件能容纳的最大帧数 *2指启用2芯片


% 计算帧数
nFrm = 0;
handleTemp = binFileHandleMaster;
while exist(handleTemp, 'file')
    fileinfo = dir(handleTemp); 
    nFrm = nFrm + fileinfo.bytes / 1024 / 1024 / data1Frm * 2;  % *2指启用2芯片
    handleTemp(end - 12 : end - 9) = num2str(str2num(handleTemp(end - 12 : end - 9)) + 1, '%04d');
end
clearvars handleTemp
nFrm = nFrm - 2;       % 留出2帧容错 实际表现为前后各舍弃1帧
tTotal = nFrm * tFrm; % 总采样时间

%% 计算可分辨范围和分辨率
maxR = adcRate * c / (2 * s);          % 最大探测距离
resR = c / (2 * bw);                         % 距离分辨率
maxV = lambda / (4 * tChirpIntvl); % 最大探测速度 m/s
resV = lambda / (2 * tChirp1Frm); % 速度分辨率 m/s



%% Range CFAR参数设置（距离）
%% CFAR：从背景噪声中准确地找出目标信号（动态、智能阈值）
cfarParamRg.train = 8;      % 单边 训练单元数量，用来估计噪声水平
cfarParamRg.guard = 4;    % 单边 保护单元数量，防止目标信号本身泄露到训练单元中

% 虚警概率 (Probability of False Alarm)，即把噪声误判为目标的概率。
% 这个值设得越低，检测门限就越高，越不容易产生误报，但可能会漏掉一些弱小的目标
cfarParamRg.pfa = 0.25;
cfarParamRg.extraTh = 0; % 额外阈值

% 漏报很多：调高pfa、减少extraTh
% 误报很多：降低pfa、增大extraTh
% train和guard的大小也需要根据目标大小的密集程度来进行调整


%% Range-Doppler CFAR参数设置（距离多普勒）
cfarParamRD.train = [8, 4];   % 单边 [距离,  速度]
cfarParamRD.guard = [8, 4]; % 单边 [距离,  速度]
cfarParamRD.pfa = 0.005;
cfarParamRD.extraTh = 0;

%% Range-Azimuth CFAR参数设置（方位角CFAR）
cfarParamRA.train = [12, 6]; % 单边 [距离,  角度]
cfarParamRA.guard = [8, 4]; % 单边 [距离,  角度]
cfarParamRA.pfa = 0.01;
cfarParamRA.extraTh = 3.5e3;

% 在save语句前添加
disp('当前数据文件路径：');
disp(binFileHandles);  % 打印主从芯片数据文件路径

%% 保存配置
save .\config\config\config.mat
