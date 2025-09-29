function [fftRsltRg, pcRg] = fftRange(radarData, varargin)
% Range FFT, 含1D CFAR
% FFT点数自动选择为一个Chirp的ADC采样数
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC, Chirp/Velocity, Rx, Tx]
%
% 2. varargin: Matlab中的可变参数
%     - windowEn: FFT时是否加窗. 0-否; 1-是
%     - pcEn: 是否计算点云. 0-否; 1-是
%     - drawEn: 是否绘图. 0-否; 1-是
%     - logEn: 是否将纵坐标设为dB. 0-否; 1-是
% 输出:
% 1. fftRsltRg: 雷达数据矩阵, [Range, Chirp/Velocity, Rx, Tx]
% 2. pcRg: 距离点云
%     - .iRange: Range bin索引
%     - .range: 距离
% 作者: 刘涵凯
% 更新: 2022-7-11

% fftRange函数功能：
% 将雷达采集到的原始时域信号，转换为频域信号，从而解算出目标的距离

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('windowEn', 1);  % 加窗处理，是否在FFT前对信号进行加窗处理
p.addOptional('pcEn', 0);      % 点云使能，是否进行CFAR检测生成距离点
p.addOptional('drawEn', 0);    % 绘图使能
p.addOptional('logEn', 1);     % 绘图的时候Y轴是否使用dB作为单位
p.parse(varargin{:});
windowEn = p.Results.windowEn;
pcEn = p.Results.pcEn;
drawEn = p.Results.drawEn;
logEn = p.Results.logEn;

%% 获取数据矩阵尺寸
[nAdc, nChirp, nRx, nTx, nFrm] = size(radarData);

%% Range FFT
if windowEn % 加窗(汉宁窗) 减少频谱泄露，有效抑制旁瓣，让信号的能量更加集中在主瓣
    radarData = radarData .* repmat(hanning(nAdc), [1, nChirp, nRx, nTx, nFrm]);
end
fftRsltRg = fft(radarData, nAdc, 1);  % 使用fft计算出雷达数据矩阵

%% 计算点云信息(CFAR目标检测)
%% 这段代码只有前面的pcEn(点云使能为1的时候才会执行)
pcRg = struct('iRange', [], 'range', []);   %距离点云，检测结果存放在这里(目标索引、真实距离)
cfarThRg = [];
if pcEn
    % CFAR
    % cfarParamRg中的参数含义：

    % train: 训练单元，表示从CUT为中心左右两侧查看多少个紧邻的单元（guard单元之外开始计数）
    % 一般情况下train越大越精准，但是如果测量范围过于复杂，大量的train也可能发生漏检

    % guard: 保护单元，紧邻着CUT的单元，在估计噪声的时候会被忽略掉。CUT左右两边各忽略掉guard个单元

    % pfa: 虚警概率，直接决定了检测门限的“灵敏度”。CFAR算法会根据这个pfa值、训练单元数量以及假设的噪声模型
    % 计算出一个乘法因子（scaling factor, α）。最终的检测门限约等于：估计的局部噪声功率 * α
    % pfa值越低，意味着对于虚警的容忍度就越低，那么最后系统计算出来的门槛就会很高，检测的结果会很干净
    % 现在这个0.25值是非常高的，也就是说系统的门槛会很低，但是这样有助于雷达捕捉信息，然后再将信息做详细的处理
    % 怎么调整后续再看

    % extrath（额外门限）
    % 这是一个附加的、固定的门限值。最终的判决门限 = (CFAR动态计算的门限) + extraTh

    load('config.mat', 'cfarParamRg') %加载预设好的CFAR参数

    fftRsltRgVec = matExtract(abs(fftRsltRg), 1, [0, 0, 0, 0]); % 提取结果向量
    % 向量中每一个元素代表了在特定距离上的信号强度

    % --- 指定使用 OS-CFAR 或者是 CA-CFAR 方法 ---
    % 通过修改method的值可以确定是使用CA还是OS cfar
    cfarParamRg.method = 'OS'; 
    cfarParamRg.rank = 12; 

    [pcRg.iRange, cfarThRg] = cfar1D(fftRsltRgVec, cfarParamRg); % 执行CFAR
    % pcRg.iRange：存储了 cfar1D 函数检测到的所有目标的索引号（向量形式）
    % cfarThRg：也是一个向量，存放了上面 cfar 对于 fftRsltRgVec 每一个点计算出的自适应门限值。
    % 这个参数的长度应该与 fftRsltRgVec 完全相同

    if isempty(pcRg.iRange); warning('未检测到距离点云'); end
    % 计算距离
    load('config.mat', 'resR')
    rg = resR * (0 : nAdc - 1)'; % 计算坐标刻度
    pcRg.range = rg(pcRg.iRange);
end

%% 绘图
if drawEn
    if ~exist('fftRsltRgVec', 'var')
        fftRsltRgVec = matExtract(abs(fftRsltRg), 1, [0, 0, 0, 0]); % 提取结果向量
    end
    drawRangeFFT(fftRsltRgVec, 'pcIdx', pcRg.iRange, 'cfarTh', cfarThRg, 'logEn', logEn)
end
