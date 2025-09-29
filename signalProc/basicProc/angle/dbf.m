function [pw, sigRecons] = dbf(angAz, angEl, X, arrayPosX, arrayPosZ, varargin)
% 1D/2D数字波束形成, 含空间谱估计和信号重构
% 支持CBF和Capon算法. 默认用Capon计算反射强度, 用CBF计算权重并重构反射信号
% 反射强度计算代码参考了MATLAB phased库MVDREstimator函数, 但将Cx\修改为pinv(Cx)以增强稳定性
% 输入:
% 1. angAz: 待测水平角, N*1
% 2. angEl: 待测俯仰角, N*1. []-1D
% 3. X: 信号矢量阵
% 4. arrayPosX: 阵元相对水平位置, N*1
% 5. arrayPosZ: 阵元相对垂直位置, N*1. []-1D
% 6. varargin:
%     - spacingCal: 天线阵列排布所依据的频率/实际中心频率
%     - pwAlgo: 计算功率的DoA算法. 'CBF'; 'Capon'
%     - wtAlgo: 计算权重的DoA算法. 'CBF'; 'Capon'
%     - pwEn: 是否计算反射强度. 0-否; 1-是
%     - sigReconsEn: 是否执行信号重构. 0-否; 1-是
% 输出:
% 1. pw: 反射强度
% 2. sigRecons: 重构信号, [Chirp, AnglePair]
% 作者: 刘涵凯
% 更新: 2022-7-26

%% 默认参数
%% 该函数中可以调整的地方就是处理 功率 与 权重 所需要的DoA算法
p = inputParser();
p.CaseSensitive = false;
p.addOptional('spacingCal', 1);
p.addOptional('pwAlgo', 'Capon'); 
%% 追求高刷新率场景的话（处理速度⬆）p.addOptional('pwAlgo', 'CBF'); 
%% 功率DoA算法更换为CBF
p.addOptional('wtAlgo', 'CBF');
p.addOptional('pwEn', 1);
p.addOptional('sigReconsEn', 0);
p.parse(varargin{:});
spacingCal = p.Results.spacingCal;
pwAlgo = p.Results.pwAlgo;
wtAlgo = p.Results.wtAlgo;
pwEn = p.Results.pwEn;
sigReconsEn = p.Results.sigReconsEn;

%% 预处理
if isempty(angEl) % 1D
    A = exp(-1i * pi * spacingCal * arrayPosX * sind(angAz')); % 导向矢量阵
else % 2D
    A = exp(-1i * pi * spacingCal * (arrayPosX * (sind(angAz') .* cosd(angEl')) + ...
        arrayPosZ * sind(angEl'))); % 导向矢量阵
end
[nAnt, nChirp] = size(X); % Chirp数即快拍数
R = X * X' / nChirp; % 协方差(自相关)矩阵

%% 反射强度
pw = [];
if pwEn 
    switch pwAlgo
        case 'CBF'
            pw = real(sum(A' .* (R * A).', 2));
        case 'Capon'
            pw = 1 ./ real(sum(A' .* (pinv(R) * A).', 2));
    end
end

%% 信号重构
sigRecons = [];
if sigReconsEn
    % 计算权重
    switch wtAlgo
        case 'CBF'
            wt = A;
        case 'Capon'
            wt = (pinv(R) * A) ./ repmat(sum(A' .* (pinv(R) * A).', 2).', [nAnt, 1]);
    end
    nAnglePair = length(angAz);
    % 复制信号矩阵并进行维度转换, 使其由nAnt * nChirp变为nChirp * nAnglePair * nAnt
    STrans = permute(repmat(X, [1, 1, nAnglePair]), [2, 3, 1]);
    % 复制权重矩阵并进行维度转换, 使其由nAnt * nAngle变为nChirp * nAngle * nAnt
    wtTrans = permute(repmat(wt, [1, 1, nChirp]), [3, 2, 1]);
    % 信号矩阵点乘权重矩阵后在天线维度累加, 重构信号size = nChirp * nAnglePair
    sigRecons = sum(STrans .* conj(wtTrans), 3);
end
