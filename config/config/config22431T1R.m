function config22431T1R
% 雷达配置、雷达参数计算、信号处理参数设置
% 适用于MMWCAS 
% 1T1R
% 作者: 刘涵凯
% 更新: 2023-8-28

%% 雷达型号
device = '2243';

%% 阵列类型
% 'all'为4芯片, 'half'为2芯片(master + slave3), '1T1R'为单发单收
arrayType = '1T1R';

%% 雷达数据路径
binFileNameMaster = 'master_0000_data.bin';
binFilePath = 'G:\radarData\23.11.27\resp1\1';

binFileHandleMaster = [binFilePath, '\', binFileNameMaster];
binFileHandles = binFileHandleMaster;

%% 使能天线
posRadar = [0, 0, 1.05]; % 雷达位置. 目前仅高度设置有用, 不要修改x和y. 雷达朝向为y轴正向
fSpacing = 77e9; % 天线阵列排布所依据的频率
nTx = 1; % 发射天线个数
nRx = 1; % 接收天线个数

%% 采样参数
fStart = 77e9; % 起始频率
nAdc1Chirp = 256; % 一个chirp的ADC采样次数
adcRate = 7.04e6; % ADC采样频率
nChirp1Frm = 199; % 一帧中各Tx发射的chrip的个数
s = 99e12; % 斜率 Hz/s
tIdle = 4960e-6; % chirp间的空闲时间
tAdcStart = 3e-6; % 开始升频但还未采样的时间
tRampEnd = 40e-6; % 整个扫频时间
tTxStart = 1e-6; % Tx启动时间
tFrm = 1000e-3; % 帧周期

%% 计算参数
c = physconst('LightSpeed'); % 光速
lambda = c / fStart; % 波长
bw = nAdc1Chirp / adcRate * s;  % 有效带宽
bwTotal = s * tRampEnd; % 总带宽
fCenter = fStart + s * tAdcStart + bw / 2; % 实际中心频率
spacingCal = fCenter / fSpacing; % 天线阵列排布所依据的频率/实际中心频率
fFrm = 1 / tFrm; % 帧频率
tChirp= tIdle + tRampEnd;  % 一个chirp的占用时间
tChirpIntvl = tChirp * nTx; % 每个Tx发射两个chirp的间隔(即chirp周期)
tChirp1Frm = tChirpIntvl * nChirp1Frm; % 一帧中的总chirp时长 
data1Frm = nTx * nRx * nAdc1Chirp * nChirp1Frm * 2 * 2 / 1024 / 1024; % 每帧数据量 MB
nFrm1File = floor(2047 / data1Frm * 1); % 一个文件能容纳的最大帧数 *1指启用1芯片
% 计算帧数
nFrm = 0;
handleTemp = binFileHandleMaster;
while exist(handleTemp, 'file')
    fileinfo = dir(handleTemp); 
    nFrm = nFrm + fileinfo.bytes / 1024 / 1024 / data1Frm;
    handleTemp(end - 12 : end - 9) = num2str(str2num(handleTemp(end - 12 : end - 9)) + 1, '%04d');
end
clearvars handleTemp
nFrm = nFrm - 2; % 留出2帧容错 实际表现为前后各舍弃1帧
tTotal = nFrm * tFrm; % 总采样时间

%% 计算可分辨范围和分辨率
maxR = adcRate * c / (2 * s); % 最大探测距离
resR = c / (2 * bw); % 距离分辨率
maxV = lambda / (4 * tChirpIntvl); % 最大探测速度 m/s
resV = lambda / (2 * tChirp1Frm); % 速度分辨率 m/s

%% Range CFAR参数设置
cfarParamRg.train = 8; % 单边 8
cfarParamRg.guard = 4; % 单边 4
cfarParamRg.pfa = 0.28; % 0.25
cfarParamRg.extraTh = 0; % 额外阈值 200

%% Range-Doppler CFAR参数设置
cfarParamRD.train = [8, 16]; % 单边 [距离,  速度]
cfarParamRD.guard = [8, 16]; % 单边 [距离,  速度]
cfarParamRD.pfa = 0.01;
cfarParamRD.extraTh = 0;

%% 保存配置
save .\config\config\config.mat
