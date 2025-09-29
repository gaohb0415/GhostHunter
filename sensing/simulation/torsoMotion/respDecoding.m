function [code, isPseudo] = respDecoding(peakTime)
% 呼吸解码
% 输入:
% peakTime: 波峰(波谷也行)的时间戳, s
% 输出:
% 1. code: 解码结果
% 2. isPseudo: 是否伪造. 0-否; 1-是
% 作者: 刘涵凯
% 更新: 2024-3-27

%% 参数对象
p = simParamShare.param;

%% 呼吸周期
respCycle = peakTime(2 : end) - peakTime(1 : end - 1);
nSeg = length(respCycle);

%% 解码
code = NaN(nSeg, 1);
code(arrayInIntvl(respCycle, p.codeCycle(1) + p.decodeCycleWin * 1.0001 * [-1, 1])) = 0;
code(arrayInIntvl(respCycle, p.codeCycle(2) + p.decodeCycleWin * 1.0001 * [-1, 1])) = 1;

%% 真伪识别
win = length(p.code) + p.codeRedun; % 识别窗
isPseudo = 0;
codeStr = join(string(p.code), '');
for iSeg = 1 : nSeg - win + 1 % 逐比特滑动
    codeTemp = code(iSeg : iSeg + win - 1);
    codeTemp(isnan(codeTemp)) = [];
    if length(codeTemp) ~= length(p.code) % 有效比特数一致性检测
        continue
    end
    if codeTemp(1) ~= p.code(1) || codeTemp(end) ~= p.code(end)  % 首尾比特一致性检测. 忘记了为什么要做这一步检测
        continue
    end
    codeTempStr = join(string(codeTemp), '');
    if contains(codeTempStr, codeStr)
        isPseudo = 1;
        return
    end
end
