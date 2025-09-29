function [respDist, code, respCycle] = respCoding(segLib)
% 呼吸编码
% 输入:
% segLib: 呼吸片段库
% - .good: 较好的呼吸片段 有这个就够了
% - .bad: 较差的呼吸片段
% - .all: 全部呼吸片段
% 输出:
% 1. respDist: 呼吸曲线, m
% 2. code: 身份编码
% 3. respCycle: 每个呼吸周期的持续时间
% 作者: 刘涵凯
% 更新: 2024-3-27

%% 参数对象
p = simParamShare.param;

%% 预估所需的编码周期数
tLmt = (p.nFrm + 1) / p.fFrm; % 雷达所需时间
nPeriod = ceil(tLmt / (p.respCycleMean * p.codePeriod));

%% 调整编码
code = zeros(nPeriod * p.codePeriod, 1);
for iPeriod = 1 : nPeriod
    nCode = length(p.code);
    nCodeRedun = nCode + p.codeRedun;
    % 加入冗余位
    % 2指未编码位
    codeWin1 = [p.code(1); 2 * ones(nCodeRedun - 2, 1); p.code(end)]; % 冗余位不出现于编码首尾
    codeWin1(sort(randperm(nCodeRedun - 2, nCode - 2)) + 1) = p.code(2 : end - 1);
    % 置于周期中
    codeWin2 = 3 * ones(p.codePeriod, 1);
    idxStart = randperm(p.codePeriod  - nCodeRedun - 1, 1) + 1;
    codeWin2(idxStart : (idxStart + nCodeRedun - 1)) = codeWin1; % 编码位不出现于周期首尾
    % 将编码位的两侧也设为冗余, 以确保两侧为高质量呼吸片段
    codeWin2([idxStart - 1, idxStart + nCodeRedun]) = 2;
    % 写入该周期的编码
    code((iPeriod - 1) * p.codePeriod + 1 : iPeriod * p.codePeriod) = codeWin2;
end

%% 生成频率
idx0 = find(code == 0);
idx1 = find(code == 1);
idx2 = find(code == 2);
idx3 = find(code == 3);
n0 = length(idx0);
n1 = length(idx1);
n2 = length(idx2);
n3 = length(idx3);
% 2/3码候选频率
cycleCand = p.respCycleMean + p.respCycleStd * randn(10 * (length([idx2; idx3])), 1); % 取10倍数量以防止不够用
uncodeIntvl = p.codeCycle' + p.respCycleGuard * 1.001 * [-1, 1]; % 未编码区间
cycleUncode = cycleCand(~arrayInIntvl(cycleCand, uncodeIntvl)); % 未编码频率
% 当未编码频率的数量还是不足时, 用比较简单粗暴的方式补足数量
if isempty(cycleUncode); cycleUncode = p.codeCycle(2) + p.respCycleGuard; end
while(length(cycleUncode) < n2); cycleUncode = [cycleUncode; cycleUncode]; end
% 编码频率
cycleCode0 = p.codeCycle(1) * ones(n0, 1);
cycleCode1 = p.codeCycle(2) * ones(n1, 1);
cycleCode2 = cycleUncode(randperm(length(cycleUncode), n2));
cycleCode3 = cycleCand(randperm(length(cycleCand), n3));

%% 将频率转换为时间
respCycle = zeros(size(code));
respCycle([idx0; idx1; idx2; idx3]) = [cycleCode0; cycleCode1; cycleCode2; cycleCode3];
while(sum(respCycle) < tLmt)
    code(end + 1) = 3;
    respCycle(end + 1) = cycleCand(randperm(length(cycleCand), 1));
end
nCode = length(code);
idx3 = find(code == 3);
n3 = length(idx3);
respCycle = round(respCycle * p.fFrm) / p.fFrm; % 使respIntvl能整除帧周期, 以确保峰值唯一性 

%% 呼吸波形调制
% 防止片段不够用
while structLength(segLib.good, 'dist') < n0 + n1 + n2; segLib.good = [segLib.good, segLib.good]; end
while structLength(segLib.all, 'dist') < n3; segLib.all = [segLib.all, segLib.all]; end
% 为每个码分配一个片段
idxValidSeg = randperm(structLength(segLib.good, 'dist'), n0 + n1 + n2);
segLib.select([idx0; idx1; idx2]) =  segLib.good(idxValidSeg);
% idxInvalidSeg = randperm(structLength(segLib.all, 'dist'), n3);
% segLib.select(idx3) =  segLib.all(idxInvalidSeg);
idxInvalidSeg = randperm(structLength(segLib.good, 'dist'), n3);
segLib.select(idx3) =  segLib.good(idxInvalidSeg);
% 每个片段的幅度
ampl = p.respDistMean +  0 * p.respDistStd * randn(nCode, 1); % 不需要额外设置标准差
% 初始化波形及时间戳
respDist = ampl(1) * segLib.select(1).dist;
respTime = linspace(0, respCycle(1), length(respDist));
% 片段连接
for iCode = 2 : nCode
    respDist = [respDist(1 : end - 1), respDist(end) + ampl(iCode) * segLib.select(iCode).dist];
    respTime = [respTime(1 : end - 1),  respTime(end) + linspace(0, respCycle(iCode), length(segLib.select(iCode).dist))];
end

%% 将曲线上的各点对齐雷达时间戳并进行去趋势
respDist = interp1(respTime, respDist, (0 : p.nFrm + 1) / p.fFrm);
respDist = respDist - movmean(respDist, round(p.fFrm *  p.respCycleMean));
