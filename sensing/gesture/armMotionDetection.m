function armMoRslt = armMotionDetection(varargin)
% 生成手部的range-time map, azimuth-time map, elevation-time map, velocity-time map
% 目前手臂距离探测机制很简陋, 有待改进
% 目前速度计算基于距离维提取, 后续可以改为基于距离-角度维提取
% 目前角度探测范围为手动设定, 可以借助角度估计和bodyDetection函数进行自动设定
% 输入: 
% varargin:
% - rtmEn: 是否生成range-time map. 0-否; 1-是
% - atmEn: 是否生成azimuth-time map. 0-否; 1-是
% - etmEn: 是否生成elevation-time map. 0-否; 1-是
% - vtmEn: 是否生成velocity-time map. 0-否; 1-是
% - limitT: 时间范围, 决定读取哪些帧
% - drawEn: 是否绘图. 0-否; 1-是
% 输出: 
% armMoRslt: 手臂运动信息
% - .map:
%    * .rtm: range-time map
%    * .atm: azimuth-time map
%    * .etm: elevation-time map
%    * .vtm: velocity-time map
% - .axis: 坐标轴
%    * .time: 时间
%    * .rg: 距离
%    * .az: 水平角
%    * .el: 俯仰角
%    * .vel: 速度
% 作者: 刘涵凯
% 更新: 2022-7-26

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('rtmEn', 1);
p.addOptional('atmEn', 1);
p.addOptional('etmEn', 1);
p.addOptional('vtmEn', 1);
p.addOptional('limitT', [0, 10]);
p.addOptional('drawEn', 1);
p.parse(varargin{:});
rtmEn = p.Results.rtmEn;
atmEn = p.Results.atmEn;
etmEn = p.Results.etmEn;
vtmEn = p.Results.vtmEn;
limitT = p.Results.limitT;
drawEn = p.Results.drawEn;
% 参数优先级: etmEn > atmEn > rtmEn
if etmEn
    atmEn = 1;
end
if atmEn
    rtmEn = 1;
end
% 参数优先级: vtmEn > rtmEn
if vtmEn
    rtmEn = 1;
end

%% 帧选取与时间设置
load('config.mat', 'nChirp1Frm', 'tFrm', 'nFrm', 'device', 'resR', 'resV')
iFrmStart = max(1, floor(limitT(1) / tFrm)); % 起始帧
iFrmEnd = min(nFrm, ceil(limitT(2) / tFrm)); % 结束帧
iFrmLoad = iFrmStart : iFrmEnd; % 选取的帧
nFrmLoad = length(iFrmLoad); % 帧数

%% 手臂定位
iRgBody = []; % 人体距离bin索引
iRgArm = []; % 手臂距离bin索引
% 人在做动作时手臂位置一直在变化, 所以在整段时间中进行定位, 最终得到手臂运动距离范围
frmStep = 10; % 每隔frmStep帧进行一次人体和手臂定位
for iFrmStep = 1 : nFrmLoad / frmStep
    radarDataTemp = readBin(iFrmLoad((iFrmStep - 1) * frmStep + 1), 0);
    fftRsltRg = fftRange(radarDataTemp);
    % 输入Range FFT结果进行人体探测
    [iRgDet, ~] = bodyDetection(matExtract(abs(fftRsltRg), 1, [0, 0, 0]), 'armDetEn', 1);
    % 更新整体探测结果
    iRgBody = [iRgBody, iRgDet.body];
    iRgArm = [iRgArm, iRgDet.arm];
end
% 选取所有人体距离bin中的众数代表人体所在距离
iRgBody = mode(iRgBody);
% 选取所有手臂距离bin中的中间绝大部分作为手臂运动距离范围
iRgArm = round(prctile(iRgArm, 10)) : round(prctile(iRgArm, 90));
iRgArm(end) = [];
disp(['身体距离: ', num2str(round((iRgBody - 1) * resR *100) / 100), 'm']) % 保留两位小数
disp(['手部移动距离: ', num2str(round((iRgArm(1) - 1) * resR * 100) / 100), '~', ...
    num2str(round((iRgArm(end) - 1) * resR * 100) / 100), 'm']) % 保留两位小数

%% 计算时变图
% 初始化
rtm = []; atm = []; etm = []; vtm = []; 
axisRg = []; axisAz = []; axisEl = []; axisVel = []; 
if rtmEn
    axisRg = resR * (iRgArm - 1); % 距离轴
    rtm = zeros(length(iRgArm), nFrmLoad);
end
if atmEn
    axisAz = -30 : 1 : 30; % 水平角轴
    nAz = length(axisAz);
    atm = zeros(nAz, nFrmLoad);
end
if etmEn
    axisEl = -30 : 1 : 30; % 俯仰角轴
    nEl = length(axisEl);
    etm = zeros(nEl, nFrmLoad);
end
if vtmEn
    axisVel = resV * (-nChirp1Frm / 2 : nChirp1Frm / 2 - 1); % 速度轴
    vtm = zeros(nChirp1Frm, nFrmLoad);
end

% 计算
for iFrm = 1 : nFrmLoad
    radarDataTemp = readBin(iFrmLoad(iFrm), 0);
    fftRsltRg = fftRange(radarDataTemp);
    fftRsltRgArm = fftRsltRg(iRgArm, :, :, :); % 取出手臂范围的Range FFT结果

    % 距离-时间图
    if rtmEn
        rtm(:, iFrm) = matExtract(abs(fftRsltRg(iRgArm, :, :, :)), 1, [0, 0, 0]);
    end
    [~, iRgHand] = max(rtm(:, iFrm)); % 将反射强度最大的距离bin视为手部所在的距离

    % 水平角-时间图
    if atmEn
        % 提取手部所在距离的信号
        antArray = virtualArray1D(fftRsltRgArm(iRgHand, :, :, :), 'DBF');
        [atm(:, iFrm), ~] = dbf(axisAz, [], antArray.signal, antArray.arrayPos, []);
        [~, iAzHand] = max(atm(:, iFrm));  % 将反射强度最大的水平角视为手部所在的水平角
    end

    % 俯仰角-时间图
    if etmEn
        % 提取手部所在距离的信号
        antArray = virtualArray2D(fftRsltRgArm(iRgHand, :, :, :), 'DBF');
        % 将输入DBF的水平角设定为手部所在水平角
        [etm(:, iFrm), ~] = dbf(repmat(axisAz(iAzHand), 1, nEl), axisEl, antArray.signal, ...
            antArray.arrayPosX, antArray.arrayPosZ);
    end

    % 速度-时间图
    if vtmEn
        % 提取手部所在距离的信号
        vtm(:, iFrm) = matExtract(abs(fftDoppler(fftRsltRgArm(iRgHand, :, :, :))), 2, [0, 0, 0]);
    end
end

%% 保存结果
armMoRslt = struct('map', [], 'axis', []);
armMoRslt.map = struct('rtm', rtm, 'atm', atm, 'etm', etm, 'vtm', vtm);
armMoRslt.axis = struct('time', tFrm * (iFrmStart + (-1 : nFrmLoad - 2)), ...
    'rg', axisRg, 'az', axisAz, 'el', axisEl, 'vel', axisVel);
save('.\postProc\gesture\armMotionResults.mat', 'armMoRslt')

%% 绘图
if drawEn
    drawArmMotion
end
