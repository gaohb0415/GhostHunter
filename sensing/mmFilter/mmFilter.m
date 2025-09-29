function dataTampered = mmFilter(dataRaw, varargin)
% mmFilter主程序
% 支持MMWCAS master+slave3
% 输入:
% 1. dataRaw: 雷达数据矩阵
% 2. varargin:
%     - .mode: mmFilter执行模式. 'none' 'pPosLock' 'vertSigCl' 'spInfoErasure'
%                                                   'chirpDisorg' 'lsbSuppr' 'dynBSF' 'interFrmPhFlctn'
%     - .lockRg: 锁定距离, m
%     - .lockAng: 锁定角度, °
%     - .angLockEn: 是否进行角度锁定. 0-否; 1-是
%     - .interpEn: 是否进行面阵插值. 0-否; 1-是
% 输出:
% dataTampered: 隐私过滤后的雷达数据矩阵
% 作者: 刘涵凯
% 更新: 2022-12-1

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('mode', 'none');
p.addOptional('lockRg', 3);
p.addOptional('lockAng', 20);
p.addOptional('angLockEn', 1);
p.addOptional('interpEn', 0);
p.parse(varargin{:});
mode = p.Results.mode;
lockRg = p.Results.lockRg;
lockAng = p.Results.lockAng;
angLockEn = p.Results.angLockEn;
interpEn = p.Results.interpEn;

%% 若未设定模式, 则直接输出原始数据
if strcmp(mode, 'none')
    dataTampered = dataRaw;
    return
end

%% 隐私过滤
dataSCF = dataRaw - mean(dataRaw, 2); % 静态杂波滤除
switch mode
    case 'pPosLock'
        %% Pseudo-position lock
        load('config.mat', 'resR')
        iRgLock = round(lockRg / resR); % 锁定距离的距离bin索引

        [fftRsltRg, ~] = fftRange(dataSCF); % Range FFT
        [fftRsltRgRaw, ~] = fftRange(dataRaw, 'windowEn', 0); % 不静态滤波、不加窗Range FFT
        % 加载/生成距离bin范围索引
        if exist('.\postProc\mmFilter\pPosLockRg.mat', 'file')
            load .\postProc\mmFilter\pPosLockRg.mat
        else
            nExtraRange = 5;
            fftRsltRgVec = matExtract(abs(fftRsltRg), 1, [0, 0, 0]); % 绝对值向量
            [iRgDet, ~] = bodyDetection(fftRsltRgVec); % 人体距离探测
            iRgDet.body = iRgDet.body(1) - nExtraRange : iRgDet.body(end) + nExtraRange;
            iRgTgt = iRgLock + iRgDet.body - iRgDet.center; % 锁定距离范围
            save('.\postProc\mmFilter\pPosLockRg.mat', 'iRgDet', 'iRgTgt')
        end
        fftRsltRgBody = fftRsltRg(iRgDet.body, :, :, :); % 在距离维度提取人体信号
        fftRsltRgBodyRaw = fftRsltRgRaw(iRgDet.body, :, :, :); % 在距离维度提取人体信号

        if angLockEn
            % 进行角度搬移
            switch interpEn
                case 0
                    % 只对一维阵列做处理
                    nRgBody = length(iRgDet.body); % 人体所占距离bin数
                    nRx = 23; % 阵元数
                    [fftRsltAng, ~] = fftAngle1D(fftRsltRgBodyRaw, 'nAngle', nRx); % Angle FFT
                    axisAngFFT = asind((0 : nRx - 1) / (nRx / 2) - 1); % FFT角度轴

                    aoiAngDBF = [-90, 90]; % 人可能存在的角度范围 °
                    resAngDBF = 1; % DBF角度步进间隔
                    axisAngDBF = aoiAngDBF(1) : resAngDBF : aoiAngDBF(2); % DBF角度轴
                    nAngDBF = length(axisAngDBF); % 进行DBF的角度数

                    % 加载/生成距离bin范围索引
                    if exist('.\postProc\mmFilter\pPosLockAng.mat', 'file')
                        load .\postProc\mmFilter\pPosLockAng.mat
                    else
                        antArrayDBF = virtualArray1D(fftRsltRgBody, 'DBF'); % 生成虚拟阵列

                        pwRA = zeros(nAngDBF, nRgBody); % 初始化RAM
                        for iRg = 1 : nRgBody
                            % 对各距离执行DBF
                            [pwRA(:, iRg), ~] = dbf(axisAngDBF, [], antArrayDBF.signal(:, :, iRg), antArrayDBF.arrayPos, []);
                        end
                        pwAng = mean(pwRA, 2); % 将RAM沿距离维度平均
                        [~, iAngDet] = bodyDetection(pwAng, 'detMode', 'angle'); % 人体角度探测
                        % 中心角度
                        iBinAngCenter = find(axisAngFFT < axisAngDBF(iAngDet.center));
                        iBinAngCenter = iBinAngCenter(end); % 人体中心在FFT角度轴中的索引
                        iBinAngLock = find(axisAngFFT < lockAng);
                        iBinAngLock = iBinAngLock(end); % 锁定角度在FFT角度轴中的索引
                        % 人体角度范围
                        iBinAngBodyLeft = find(axisAngFFT < axisAngDBF(iAngDet.body(1)));
                        iBinAngBodyLeft = iBinAngBodyLeft(end); % 左边界在FFT角度轴中的索引
                        iBinAngBodyRight = find(axisAngFFT < axisAngDBF(iAngDet.body(end)));
                        iBinAngBodyRight = iBinAngBodyRight(end); % 右边界在FFT角度轴中的索引
                        iBinAngBody = iBinAngBodyLeft : iBinAngBodyRight; % 人体在FFT角度轴中的索引范围
                        % 锁定角度范围
                        iBinAngTgt = iBinAngLock + iBinAngBody - iBinAngCenter; % 角度搬移后在FFT角度轴中的索引范围
                        save('.\postProc\mmFilter\pPosLockAng.mat', 'iBinAngBody', 'iBinAngTgt')
                    end
                    fftRsltAngBody = fftRsltAng(:, :, iBinAngBody); % 在角度维度提取人体信号

                    % 进行角度搬移
                    if iBinAngBody(1) > 1 && iBinAngBody(end) < nRx % 人体角度范围未接触两边界
                        fftRsltAngAmb = fftRsltAng(:, :, [1 : iBinAngBody(1) - 1, iBinAngBody(end) + 1 : end]);
                        fftRsltAngMove = cat(3, fftRsltAngAmb(:, :, 1 : iBinAngTgt(1) - 1), ...
                            fftRsltAngBody, fftRsltAngAmb(:, :, iBinAngTgt(1) : end));
                    elseif iBinAngBody(1) == 1 && iBinAngBody(end) < nRx % 人体角度范围接触左边界
                        fftRsltAngAmb = fftRsltAng(:, :, iBinAngBody(end) + 1 : end);
                        fftRsltAngMove = cat(3, fftRsltAngBody, fftRsltAngAmb);
                    elseif iBinAngBody(1) > 1 && iBinAngBody(end) == nRx % 人体角度范围接触右边界
                        fftRsltAngAmb = fftRsltAng(:, :, 1 : iBinAngBody(1) - 1);
                        fftRsltAngMove = cat(3, fftRsltAngAmb, fftRsltAngBody);
                    end

                    % 试验更改相位
                    % fftRsltAngMove(5:6, :, 12) = repmat(mean(fftRsltAngMove(5:6, :, 12), 2), 1, 128);

                    % Angle IFFT, 阵元数为单数时务必先shift, 后flip
                    ifftRsltAng = ifft(flip(fftshift(fftRsltAngMove, 3), 3), nRx, 3);
                    % 由虚拟阵列恢复原始阵列
                    load array1D.mat
                    ifftRsltAng = ifftRsltAng(:, :, array1D.iUnit.cascade.half.txSet3);
                    ifftRsltAng = reshape(ifftRsltAng, [nRgBody, size(dataRaw, 2), 8, 3]);
                    % 只对水平阵列进行了角度搬移, 所以这里要把垂直阵元补上
                    fftRsltRgBodyRaw = cat(4, fftRsltRgBodyRaw(:, :, :, 1 : 3), ifftRsltAng);

                case 1
                    % 对插值后的均匀方阵做处理
                    nRgBody = length(iRgDet.body); % 人体所占距离bin数
                    nRx = 26; % 阵元数
                    % Angle FFT
                    antArray = virtualArray2D(fftRsltRgBodyRaw, 'FFT');
                    fftRsltAng = fftshift(fft(antArray.signal, nRx, 3), 3);
                    fftRsltAng = flip(fftRsltAng, 3); % 角度翻转

                    axisAngFFT = asind((0 : nRx - 1) / (nRx / 2) - 1); % FFT角度轴

                    aoiAngDBF = [-80, 80]; % 人可能存在的角度范围 °
                    resAngDBF = 1; % DBF角度步进间隔
                    axisAngDBF = aoiAngDBF(1) : resAngDBF : aoiAngDBF(2); % DBF角度轴
                    nAngDBF = length(axisAngDBF); % 进行DBF的角度数

                    % 加载/生成距离bin范围索引
                    if exist('.\postProc\mmFilter\pPosLockAng.mat', 'file')
                        load .\postProc\mmFilter\pPosLockAng.mat
                    else
                        antArrayDBF = virtualArray1D(fftRsltRgBody, 'DBF'); % 生成虚拟阵列

                        pwRA = zeros(nAngDBF, nRgBody); % 初始化RAM
                        for iRg = 1 : nRgBody
                            % 对各距离执行DBF
                            [pwRA(:, iRg), ~] = dbf(axisAngDBF, [], antArrayDBF.signal(:, :, iRg), antArrayDBF.arrayPos, []);
                        end
                        pwAng = mean(pwRA, 2); % 将RAM沿距离维度平均
                        % [~, iAngDet] = bodyDetection(pwAng, 'detMode', 'angle'); % 人体角度探测
                        [~, iAngDet] = bodyDetection(pwAng, 'detMode', 'angle', 'lmtNumDetAng', 25); % 人体角度探测
                        % 中心角度
                        iBinAngCenter = find(axisAngFFT < axisAngDBF(iAngDet.center));
                        iBinAngCenter = iBinAngCenter(end); % 人体中心在FFT角度轴中的索引
                        iBinAngLock = find(axisAngFFT < lockAng);
                        iBinAngLock = iBinAngLock(end); % 锁定角度在FFT角度轴中的索引
                        % 人体角度范围
                        iBinAngBodyLeft = find(axisAngFFT < axisAngDBF(iAngDet.body(1)));
                        iBinAngBodyLeft = iBinAngBodyLeft(end); % 左边界在FFT角度轴中的索引
                        iBinAngBodyRight = find(axisAngFFT < axisAngDBF(iAngDet.body(end)));
                        iBinAngBodyRight = iBinAngBodyRight(end); % 右边界在FFT角度轴中的索引
                        iBinAngBody = iBinAngBodyLeft : iBinAngBodyRight; % 人体在FFT角度轴中的索引范围
                        % 锁定角度范围
                        iBinAngTgt = iBinAngLock + iBinAngBody - iBinAngCenter; % 角度搬移后在FFT角度轴中的索引范围
                        save('.\postProc\mmFilter\pPosLockAng.mat', 'iBinAngBody', 'iBinAngTgt')
                    end
                    fftRsltAngBody = fftRsltAng(:, :, iBinAngBody, :); % 在角度维度提取人体信号

                    % 进行角度搬移
                    if iBinAngBody(1) > 1 && iBinAngBody(end) < nRx % 人体角度范围未接触两边界
                        fftRsltAngAmb = fftRsltAng(:, :, [1 : iBinAngBody(1) - 1, iBinAngBody(end) + 1 : end], :);
                        fftRsltAngMove = cat(3, fftRsltAngAmb(:, :, 1 : iBinAngTgt(1) - 1, :), ...
                            fftRsltAngBody, fftRsltAngAmb(:, :, iBinAngTgt(1) : end, :));
                    elseif iBinAngBody(1) == 1 && iBinAngBody(end) < nRx % 人体角度范围接触左边界
                        fftRsltAngAmb = fftRsltAng(:, :, iBinAngBody(end) + 1 : end, :);
                        fftRsltAngMove = cat(3, fftRsltAngBody, fftRsltAngAmb);
                    elseif iBinAngBody(1) > 1 && iBinAngBody(end) == nRx % 人体角度范围接触右边界
                        fftRsltAngAmb = fftRsltAng(:, :, 1 : iBinAngBody(1) - 1, :);
                        fftRsltAngMove = cat(3, fftRsltAngAmb, fftRsltAngBody);
                    end

                    % Angle IFFT, 阵元数为单数时务必先shift, 后flip
                    ifftRsltAng = ifft(flip(fftshift(fftRsltAngMove, 3), 3), nRx, 3);
                    % 由虚拟阵列恢复原始阵列
                    fftRsltRgBodyRaw = zeros(nRgBody, size(dataRaw, 2), 8, 6);
                    fftRsltRgBodyRaw(:, :, :, 1) = ifftRsltAng(:, :, [23 : 26, 12 : 15], 1);
                    fftRsltRgBodyRaw(:, :, :, 2) = ifftRsltAng(:, :, [22 : 25, 11 : 14], 3);
                    fftRsltRgBodyRaw(:, :, :, 3) = ifftRsltAng(:, :, [21 : 24, 10 : 13], 6);
                    fftRsltRgBodyRaw(:, :, :, 4) = ifftRsltAng(:, :, [20 : 23, 9 : 12], 7);
                    fftRsltRgBodyRaw(:, :, :, 5) = ifftRsltAng(:, :, [16 : 19, 5 : 8], 7);
                    fftRsltRgBodyRaw(:, :, :, 6) = ifftRsltAng(:, :, [12 : 15, 1 : 4], 7);
            end
        end % if angLockEn

        % 进行距离搬移
        nAdc = size(dataRaw, 1);
        if iRgDet.body(1) > 1 && iRgDet.body(end) < nAdc % 人体距离范围未接触两边界
            fftRsltRgAmb = fftRsltRgRaw([1 : iRgDet.body(1) - 1, iRgDet.body(end) + 1 : end], :, :, :);
            fftRsltRgMove = [fftRsltRgAmb(1 : iRgTgt(1) - 1, :, :, :); fftRsltRgBodyRaw; fftRsltRgAmb(iRgTgt(1) : end, :, :, :)];
        elseif iRgDet.body(1) == 1 && iRgDet.body(end) < nAdc % 人体距离范围接触近边界
            fftRsltRgAmb = fftRsltRgRaw(iRgDet.body(end) + 1 : end, :, :, :);
            fftRsltRgMove = [fftRsltRgBodyRaw; fftRsltRgAmb];
        elseif iRgDet.body(1) > 1 && iRgDet.body(end) == nAdc % 人体距离范围接触远边界
            fftRsltRgAmb = fftRsltRgRaw(1 : iRgDet.body(1) - 1, :, :, :);
            fftRsltRgMove = [fftRsltRgAmb; fftRsltRgBodyRaw];
        end

        dataTampered = ifft(fftRsltRgMove, nAdc, 1); % Range IFFT

    case 'vertSigCl'
        %% Vertical signal clone
        nAdc = size(dataRaw, 1);
        [fftRsltRg, ~] = fftRange(dataSCF); % Range FFT
        [fftRsltRgRaw, ~] = fftRange(dataRaw, 'windowEn', 0); % 不静态滤波、不加窗Range FFT
        % 用水平天线的数据覆盖垂直天线
        fftRsltRgCl = cat(4, repmat(fftRsltRgRaw(:, :, :, 4, :), [1, 1, 1, 3, 1]),  fftRsltRgRaw(:, :, :, 4 : 6, :));

        fftRsltRgVec = matExtract(abs(fftRsltRg), 1, [0, 0, 0]); % 绝对值向量
        [iRgDet, ~] = bodyDetection(fftRsltRgVec, 'armDetEn', 1); % 获得手臂距离范围
        if ~isempty(iRgDet.arm)
            % 保留手臂范围内的垂直信息
            fftRsltRgCl(iRgDet.arm, :, :, 1 : 3, :) = fftRsltRgRaw(iRgDet.arm, :, :, 1 : 3, :);
        end

        dataTampered = ifft(fftRsltRgCl, nAdc, 1); % Range IFFT

    case 'spInfoErasure'
        %% Spatial information erasure
        dataRaw = dataRaw(:, :, :, 4 : 6); % 只考虑水平阵列
        nRx =23; % 阵元数
        [fftRsltRgRaw, ~] = fftRange(dataRaw, 'windowEn', 0); % 不静态滤波、不加窗Range FFT

        if exist('.\postProc\mmFilter\spInfoErasureConfig.mat','file')
            load .\postProc\mmFilter\spInfoErasureConfig.mat

            fftRsltRgBodyRaw = fftRsltRgRaw(iRgDet.body, :, :, :); % 在距离维度提取人体信号
            [fftRsltAng, ~] = fftAngle1D(fftRsltRgBodyRaw, 'nAngle', nRx); % Angle FFT

        else
            % 加载/生成距离bin范围索引
            [fftRsltRg, ~] = fftRange(dataSCF); % Range FFT
            fftRsltRgVec = matExtract(abs(fftRsltRg), 1, [0, 0, 0]); % 绝对值向量
            [iRgDet, ~] = bodyDetection(fftRsltRgVec); % 人体距离探测
            nRgBody = length(iRgDet.body); % 人体所占距离bin数
            fftRsltRgBody = fftRsltRg(iRgDet.body, :, :, :); % 在距离维度提取人体信号
            fftRsltRgBodyRaw = fftRsltRgRaw(iRgDet.body, :, :, :); % 在距离维度提取人体信号

            % Angle FFT
            [fftRsltAng, ~] = fftAngle1D(fftRsltRgBodyRaw, 'nAngle', nRx); % Angle FFT

            axisAngFFT = asind((0 : nRx - 1) / (nRx / 2) - 1); % FFT角度轴

            % DBF
            aoiAngDBF = [-80, 80]; % 人可能存在的角度范围 °
            resAngDBF = 1; % DBF角度步进间隔
            axisAngDBF = aoiAngDBF(1) : resAngDBF : aoiAngDBF(2); % DBF角度轴
            nAngDBF = length(axisAngDBF); % 进行DBF的角度数
            antArrayDBF = virtualArray1D(fftRsltRgBody, 'DBF'); % 生成虚拟阵列
            pwRA = zeros(nAngDBF, nRgBody); % 初始化RAM
            for iRg = 1 : nRgBody
                % 对各距离执行DBF
                [pwRA(:, iRg), ~] = dbf(axisAngDBF, [], antArrayDBF.signal(:, :, iRg), antArrayDBF.arrayPos, []);
            end
            pwAng = mean(pwRA, 2); % 将RAM沿距离维度平均
            [~, iAngDet] = bodyDetection(pwAng, 'detMode', 'angle'); % 人体角度探测
            % 人体角度范围
            iBinAngBodyLeft = find(axisAngFFT < axisAngDBF(iAngDet.body(1)));
            iBinAngBodyLeft = iBinAngBodyLeft(end); % 左边界在FFT角度轴中的索引
            iBinAngBodyRight = find(axisAngFFT < axisAngDBF(iAngDet.body(end)));
            iBinAngBodyRight = iBinAngBodyRight(end); % 右边界在FFT角度轴中的索引
            iBinAngBody = iBinAngBodyLeft : iBinAngBodyRight; % 人体在FFT角度轴中的索引范围

            rgDisorgOrder = randperm(nRgBody); % 距离乱序的顺序
            angDisorgOrder = randperm(length(iBinAngBody)); % 角度乱序的顺序

            save('.\postProc\mmFilter\spInfoErasureConfig.mat', 'iRgDet', 'nRgBody', 'iBinAngBody', 'rgDisorgOrder', 'angDisorgOrder')
        end

        % 在角度维度提取人体信号
        fftRsltAngBody = fftRsltAng(:, :, iBinAngBody);
        % 角度乱序
        fftRsltAng(:, :, iBinAngBody) = fftRsltAngBody(:, :, angDisorgOrder);
        % Angle IFFT, 阵元数为单数时务必先shift, 后flip
        ifftRsltAng = ifft(flip(fftshift(fftRsltAng, 3), 3), nRx, 3);
        % 由虚拟阵列恢复原始阵列
        load array1D.mat
        ifftRsltAng = ifftRsltAng(:, :, array1D.iUnit.cascade.half.txSet3);
        ifftRsltAng = reshape(ifftRsltAng, [nRgBody, size(dataRaw, 2), 8, 3]);
        % 距离乱序
        fftRsltRgBodyRaw = ifftRsltAng(rgDisorgOrder, :, :, :);
        % 恢复完整距离
        fftRsltRgRaw(iRgDet.body, :, :, :) = fftRsltRgBodyRaw;
        % Range IFFT
        dataTampered = ifft(fftRsltRgRaw, size(dataRaw, 1), 1);
        % 用水平天线的数据覆盖垂直天线
        dataTampered = cat(4, repmat(dataTampered(:, :, :, 1), [1, 1, 1, 3]),  dataTampered);

    case 'chirpDisorg'
        %% Chirp disorganization
        % 载入/生成随机顺序
        if exist('.\postProc\mmFilter\chirpDisorgOrder.mat','file')
            load .\postProc\mmFilter\chirpDisorgOrder.mat
        else
            nChirp = size(dataRaw, 2);
            chirpDisorgOrder = randperm(nChirp);
            save('.\postProc\mmFilter\chirpDisorgOrder.mat', 'chirpDisorgOrder')
        end

        dataTampered = dataRaw(:, chirpDisorgOrder, :, :, :);

    case 'lsbSuppr'
        %% Lower sideband suppression
        [nAdc, nChirp] = size(dataRaw, [1, 2]);
        [fftRsltRg, ~] = fftRange(dataSCF); % Range FFT
        [fftRsltRgRaw, ~] = fftRange(dataRaw, 'windowEn', 0); % 不静态滤波、不加窗Range FFT
        [fftRsltRD, ~] = fftDoppler(fftRsltRg); % Doppler FFT
        [fftRsltRDRaw, ~] = fftDoppler(fftRsltRgRaw, 'windowEn', 0); % 不加窗Doppler FFT
        fftRsltRDAbs = matExtract(abs(fftRsltRD), [1, 2], [0, 0]); % RDM, 用于计算抑制区间

        if mod(nChirp, 2) == 0 % 若nChirp为偶数
            iMiddle = nChirp / 2 + 1;
        else % 若nChirp为奇数
            iMiddle = (nChirp + 1) / 2;
        end

        % 若想将上下边带替换, 则启用下面两行
        fftRsltRDAbs = flip(fftRsltRDAbs, 2);
        fftRsltRDRaw = flip(fftRsltRDRaw, 2);

        thPct = 80; % 强度百分比阈值
        for iRange = 1 : nAdc
            % 抑制区间的速度bin索引
            iVelocityDetect = fftRsltRDAbs(iRange, 1 : iMiddle - 3) > prctile(fftRsltRDAbs(iRange, 1 : iMiddle - 3), thPct);
            % 将抑制区间置零
            fftRsltRDRaw(iRange, iVelocityDetect, :, :) = 0;
        end

        % 打乱顺序, 破坏轮廓
        if exist('.\postProc\mmFilter\velDisorgOrder.mat','file')
            load .\postProc\mmFilter\velDisorgOrder.mat
        else
            nChirp = size(dataRaw, 2);
            velDisorgOrder = randperm(iMiddle - 3);
            save('.\postProc\mmFilter\viDisorgOrder.mat', 'velDisorgOrder')
        end
        fftRsltRDRawTemp = fftRsltRDRaw(:, 1 : iMiddle - 3, :, :);
        fftRsltRDRaw(:, 1 : iMiddle - 3, :, :) = fftRsltRDRawTemp(:, velDisorgOrder, :, :);

        % 若想将上下边带替换, 则启用下行
        fftRsltRDRaw = flip(fftRsltRDRaw, 2);

        dataTampered = ifft(fftshift(fftRsltRDRaw, 2), nChirp, 2); % Doppler IFFT
        dataTampered = ifft(dataTampered, nAdc, 1); % Range IFFT

    case 'dynBSF'
        %% Dynamic bandstop filter
        [nAdc, nChirp, nRx, nTx] = size(dataRaw);
        [fftRsltRg, ~] = fftRange(dataSCF); % Range FFT
        [fftRsltRgRaw, ~] = fftRange(dataRaw, 'windowEn', 0); % 不静态滤波、不加窗Range FFT
        [fftRsltRD, ~] = fftDoppler(fftRsltRg); % Doppler FFT
        [fftRsltRDRaw, ~] = fftDoppler(fftRsltRgRaw, 'windowEn', 0); % 不加窗Doppler FFT
        fftRsltRDAbs = matExtract(abs(fftRsltRD), [1, 2], [0, 0]); % RDM, 用于计算滤波区间

        thPct = 60; % % 将强度较小的一大部分保留时的强度百分比阈值
        nAround = 2; % 将最大强度的速度bin及其周围2*nAround个bin保留
        iRDRetain = zeros(nAdc, nChirp); % 保留的RD bin的索引在iRDRetain中设为1
        for iRange = 1 : nAdc
            % 保留强度较小的速度
            iRDRetain(iRange, :) = fftRsltRDAbs(iRange, :) < prctile(fftRsltRDAbs(iRange, :), thPct);
            % 保留强度最大的速度
            [~, iVelocityStr] = max(fftRsltRDAbs(iRange, :));
            iVelocityStr = max(1, iVelocityStr - nAround) : min(nChirp, iVelocityStr + nAround);
            iRDRetain(iRange, iVelocityStr) = 1;
        end
        % 保留所选部分, 即滤除中等强度部分
        fftRsltRDRaw = fftRsltRDRaw .* repmat(iRDRetain, [1, 1, nRx, nTx]);

        dataTampered = ifft(fftshift(fftRsltRDRaw, 2), nChirp, 2); % Doppler IFFT
        dataTampered = ifft(dataTampered, nAdc, 1); % Range IFFT

    case 'interFrmPhFlctn'
        %% Inter-frame phase fluctuation
        % 在命令行运行下句进行帧序号重置
        % load phFlctnInterFrm.mat; iFrm = 1; save('.\postProc\mmFilter\phFlctnInterFrm.mat', 'phFlctnInterFrm', 'iFrm')
        % 载入/生成随机相位
        if exist('.\postProc\mmFilter\phFlctnInterFrm.mat','file')
            load .\postProc\mmFilter\phFlctnInterFrm.mat
        else
            phFlctnInterFrm = rand(1000);
            iFrm = 1;
            save('.\postProc\mmFilter\phFlctnInterFrm.mat', 'phFlctnInterFrm', 'iFrm')
        end
        % 相位叠加
        dataTampered = dataRaw * exp(-1i * 2 * pi * phFlctnInterFrm(iFrm));
        % 更新帧序号
        iFrm = iFrm + 1;
        save('.\postProc\mmFilter\phFlctnInterFrm.mat', 'phFlctnInterFrm', 'iFrm')

end
