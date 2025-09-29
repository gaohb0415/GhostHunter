function [iRgDet, iAngDet] = bodyDetection(pwRslt, varargin)
% 探测人体距离、角度范围和手臂距离范围
% 输入: 
% 1. pwRslt: 点云坐标
% 2. varargin:
%     - detMode: 探测维度. 'range'-仅距离; 'angle'-仅角度; 'RA'-距离+角度
%     - armDetEn: 是否探测手臂. 0-否; 1-是
%     - thFactorRg: 距离阈值参数, 详见代码
%     - thFactorAng: 角度阈值参数, 详见代码
%     - lmtNumDetRg: 人体所占最大距离bin数(单侧)
%     - lmtNumDetAng: 人体所占最大角度bin数(单侧), 此数要根据角度分辨率而设定
%     - extRg: 向外扩展的距离bin数
% 输出: 
% iRgDet: 人体距离探测结果(均为索引)
% - .stongest: 反射强度最大的距离
% - .body: 人体距离范围
% - .center: 人体所在距离(人体距离范围的中位数)
% - .arm: 手臂距离范围
% iAngDet: 人体角度探测结果(均为索引)
% - .stongest: 反射强度最大的角度
% - .body: 人体角度范围
% - .center: 人体所在角度(人体角度范围的中位数)
% 作者: 刘涵凯
% 更新: 2022-7-26

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('detMode', 'range');
p.addOptional('armDetEn', 0);
p.addOptional('thFactorRg', [0.05, 1.05]); % [0.05, 1.02]
p.addOptional('thFactorAng', [0.05, 1.02]);
p.addOptional('lmtNumDetRg', 15);
p.addOptional('lmtNumDetAng', 15);
p.addOptional('extRg', [0, 0]);
p.parse(varargin{:});
detMode = p.Results.detMode;
armDetEn = p.Results.armDetEn;
thFactorRg = p.Results.thFactorRg;
thFactorAng = p.Results.thFactorAng;
lmtNumDetRg = p.Results.lmtNumDetRg;
lmtNumDetAng = p.Results.lmtNumDetAng;
extRg = p.Results.extRg;

%% 距离探测
iRgDet = struct('center', [], 'body', [], 'arm', []);
if strcmp(detMode, 'range') || strcmp(detMode, 'RA')
    switch detMode
        case 'range'
            pwRg = pwRslt;
        case 'RA'
            pwRg = mean(pwRslt, 2); % 若输入为RA矩阵, 则在角度维平均
    end

    [~, iRgDet.strongest] = max(pwRg); % 反射强度最大的距离bin的索引
    % 综合确定阈值: max("最大强度乘系数thFactor(1)", "频谱强度的中位数乘系数thFactor(2)")
    thPw = max([thFactorRg(1) * pwRg(iRgDet.strongest), thFactorRg(2) * median(pwRg)]);
    % 候选距离索引, 即反射强度最大的距离bin的索引+-人体单侧所占最大距离bin数
    iCand = max(1, iRgDet.strongest - lmtNumDetRg) : min(length(pwRg), iRgDet.strongest + lmtNumDetRg);
    % 将候选bin的强度与阈值进行比较
    pwCmp = find(pwRg(iCand) > thPw);
    % 获得人体距离范围
    iRgDet.body = iCand(pwCmp(1) : pwCmp(end));
    iRgDet.body = [iRgDet.body(1) - (extRg(1) : -1 : -1), iRgDet.body, iRgDet.body(end) + (1 : extRg(2))];
    % 用中位数代表人体所在距离(也可以用平均数、众数等)
    iRgDet.center = round(median(iRgDet.body));

    % 手臂探测过于粗暴, 之后需要改进
    if armDetEn
        [~, iMiddle] = findpeaks(1 ./ pwRg(iRgDet.body), 'NPeaks', 1); % 找到手臂和身体中间的强度谷点
        if any(iMiddle)
            iRgDet.arm = iRgDet.body(1 : iMiddle);
        end
    end
end

%% 角度探测
iAngDet = struct('center', [], 'body', []);
if strcmp(detMode, 'angle') || strcmp(detMode, 'RA')
    switch detMode
        case 'angle'
            pwAng = pwRslt;
        case 'RA'
            pwAng = mean(pwRslt, 1); % 若输入为RA矩阵, 则在距离维平均
    end

    [~, iAngDet.stongest] = max(pwAng);  % 反射强度最大的角度bin的索引
    % 综合确定阈值: max("最大强度乘系数thFactor(1)", "频谱强度的中位数乘系数thFactor(2)")
    thPw = max([thFactorAng(1) * pwAng(iAngDet.stongest), thFactorAng(2) * median(pwAng)]);
    % 候选角度索引, 即反射强度最大的角度bin的索引+-人体单侧所占最大角度bin数
    iCand = max(1, iAngDet.stongest - lmtNumDetAng) : min(length(pwAng), iAngDet.stongest + lmtNumDetAng);
    % 将候选bin的强度与阈值进行比较
    pwCmp = find(pwAng(iCand) > thPw);
    % 获得人体角度范围
    iAngDet.body = iCand(pwCmp(1) : pwCmp(end));
    % 用中位数代表人体所在角度(也可以用平均数、众数等)
    iAngDet.center = round(median(iAngDet.body));
end

