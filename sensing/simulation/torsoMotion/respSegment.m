% 呼吸曲线分段
% 已被修改为单人版本, 用于评估的两类此代码需要查看之前的备份
% 作者: 刘涵凯
% 更新: 2024-3-28

clear; close all

%% 路径设置
hData = 'G:\radarData\simulation\resp\respData\allMeasured\rawCurve\';
hRespSeg = 'G:\radarData\simulation\resp\curveLib\segLib\measured\respSeg.mat';
hLib = 'G:\radarData\simulation\resp\curveLib\segLib\measured\segLib.mat';

%% 初始化
nCurve = 200; % 这里设成10也行
respSeg(nCurve).good = [];
respSeg(nCurve).bad = [];
respSeg(nCurve).all = [];

%% 呼吸片段提取
for iCurve = 1 : nCurve
    load([hData, num2str(iCurve), '.mat'])
    idxPeak = peak.idx;
    nPeak = length(idxPeak);
    % 片段处理
    for iPeak = 1 : nPeak - 1
        curveSeg = dist(idxPeak(iPeak) : idxPeak(iPeak + 1));
        nSample = length(curveSeg);
        % 平滑
        curveSeg = smoothdata(curveSeg, 10);
        % 记录幅度和时间
        amplSeg = max(curveSeg) - min(curveSeg);
        tSeg = peak.time(iPeak + 1) - peak.time(iPeak);
        % 将首尾尖峰进行锐化
        % 这里使用简单粗暴的方式, 直接删去首尾的斜率较低的部分
        nDel = ceil(nSample / 40); % 
        curveSeg([1 : nDel, end - nDel + 1 : end]) = [];
        % 对幅度进行两步归一化
        % 首先以第一个值做差分
        curveSeg = curveSeg - curveSeg(1);
        % 然后对幅度做归一化(不必严格)
        curveSeg = curveSeg / abs(max(curveSeg));
        % 评价呼吸波形好坏
        % 评价标准1: 首尾差值比较
        isGoodDiff = (curveSeg(end) > -0.25) && (curveSeg(end) < 0.25);
        % 评价标准2: 曲线平滑性
        headSeg = curveSeg(1 : ceil(nSample / 5));
        tailSeg = curveSeg(end - ceil(nSample / 5) + 1 : end);
        headSeg = headSeg(2 : end) - headSeg(1 : end - 1);
        tailSeg = tailSeg(1 : end - 1) - tailSeg(2 : end);
        thD = 0.015;
        isSmoooth = (min(headSeg) > thD) && (min(tailSeg) > thD); % 10
        % 评价标准3: 边缘锐度. 此标准仅在编码评估中使用. 不要在意量纲
        d1 = curveSeg(2) - curveSeg(1);
        d2 = curveSeg(end - 1) - curveSeg(end);
        thD = 0.021;
        isSharp = (d1 > thD) && (d2 > thD); % 10
        % 写入
        if isGoodDiff && isSmoooth && isSharp
            plot(curveSeg)
            respSeg(iCurve).good(end + 1).dist = curveSeg;
            respSeg(iCurve).good(end).time = tSeg;
            respSeg(iCurve).good(end).ampl = amplSeg;
        else
            respSeg(iCurve).bad(end + 1).dist = curveSeg;
            respSeg(iCurve).bad(end).time = tSeg;
            respSeg(iCurve).bad(end).ampl = amplSeg;
        end
    end
    respSeg(iCurve).all = [respSeg(iCurve).good, respSeg(iCurve).bad];
    respSeg(iCurve).amplMean = mean([respSeg(iCurve).good.ampl]);
    respSeg(iCurve).amplStd = std([respSeg(iCurve).good.ampl]);
    respSeg(iCurve).timeMean = mean([respSeg(iCurve).good.time]);
    respSeg(iCurve).timeStd = std([respSeg(iCurve).good.time]);
end
save(hRespSeg, 'respSeg');

%% 只选取第一个人的正常呼吸数据
for iPpl = 1 : 10
    segLib = libLink(respSeg(1 : 10));
end
save(hLib, 'segLib');

%% 将各呼吸曲线的片段整合到同一结构体
function libOut = libLink(libIn)
    nLib = length(libIn);
    libOut = libIn(1);
    for iLib = 2 : nLib
        libOut.good = [libOut.good, libIn(iLib).good];
        libOut.bad = [libOut.bad, libIn(iLib).bad];
        libOut.all = [libOut.all, libIn(iLib).all];
        libOut.amplMean = [libOut.amplMean, libIn(iLib).amplMean];
        libOut.amplStd = [libOut.amplStd, libIn(iLib).amplStd];
        libOut.timeMean = [libOut.timeMean, libIn(iLib).timeMean];
        libOut.timeStd = [libOut.timeStd, libIn(iLib).timeStd];
    end
end
