function radarData = readTDA2(iFrmLoad, iChirpLoad)
% 读取MMWCAS-DSP-EVM采集的2243数据
% 基于TI read_ADC_bin_TDA2_separateFiles函数开发
% 输入:
% 1. iFrm: 0-提取所有帧; 单独数字-提取某帧; 向量-提取多帧
% 2. iChirp: 同理提取相应Chirp
% 输出:
% radarData: 雷达数据矩阵, [ADC, Chirp, Rx, Tx, (Frame)]
% 作者: 刘涵凯
% 更新: 2023-8-28

%% 计算数据参数
load('config.mat', 'binFileHandles', 'nAdc1Chirp', 'nChirp1Frm', 'nRx', 'nTx', 'nFrm', 'nFrm1File')
nDev = size(binFileHandles, 1); % 启用的芯片数
nRx1Dev = nRx / nDev; % 每个芯片的Rx数
if ~iChirpLoad; iChirpLoad = 1 : nChirp1Frm; end % 读取Chirp的索引. iChirp为0指取全部chirp
nChirpLoad = length(iChirpLoad); % 每帧读取的Chirp数
nIQ1Chirp = nAdc1Chirp * nTx * nRx1Dev * 2; % 每个芯片每个Chirp的IQ信号数. 1ADC=2IQ
nIQ1Frm = nIQ1Chirp * nChirp1Frm; % 每个芯片每帧的IQ信号数
if ~iFrmLoad; iFrmLoad = 1 : nFrm; end % 读取帧的索引 iFrm为0指取全部帧
iFrmLoad = iFrmLoad + 1; % 硬件本身已删去一帧, 此处在其基础上再删一帧, 即, iFrm=1对应实际的第3帧
nFrmLoad = length(iFrmLoad); % 读取的帧数
iFile = ceil(iFrmLoad / nFrm1File); % 各帧数据位于第几个文件
iFrmInFile = iFrmLoad - nFrm1File * (iFile - 1); % 各帧数据位于对应文件第几帧

%% 数据读取
radarData = zeros(nRx, nAdc1Chirp, nTx, nChirpLoad, nFrmLoad); % 输出初始化. 这里依据的是二进制文件的格式
for iFrm = 1 : nFrmLoad
    disp(['雷达数据读取: 第', num2str(iFrmLoad(iFrm) - 1), '帧']);
    binFileHandles(:, end - 12 : end - 9) = repmat(num2str(iFile(iFrm) - 1, '%04d'), nDev, 1);
    for iDev = 1 : nDev
        radarData(nRx1Dev * (iDev - 1) + (1 : nRx1Dev), :, :, :, iFrm) = readBinFile(binFileHandles(iDev, :), iFrmInFile(iFrm), iChirpLoad);
    end
end
radarData = permute(radarData, [2, 4, 1, 3, 5]); % 重组数据为[ADC, Chirp, Rx, Tx, Frame]

    function [adcDataComplex] = readBinFile(fileFullPath, iFrm, iChirp)
        %% 读取MMWCAS-DSP-EVM采集的2243数据
        % 输入:
        % 1. fileFullPath: 数据文件路径
        % 2. iFrm: 读取第iFrm帧(仅一帧)
        % 3. iChirp: 读取第iChirp帧(单Chirp、多Chirp皆可)
        % 输出:
        % adcDataComplex: 格式为[ADC, Chirp, Rx, Tx]的四维复数矩阵

        adcDataComplex = zeros(nRx1Dev, nAdc1Chirp, nTx, nChirpLoad);
        fp = fopen(fileFullPath, 'r');
        % 若取1chirp或取连续chirp, 则直接取值
        if nChirpLoad == 1 || ~any(diff(nChirpLoad) - 1)
            fseek(fp, ((iFrm - 1) * nIQ1Frm + (iChirp(1) - 1) * nIQ1Chirp) * 2, 'bof');
            adcData = fread(fp, nIQ1Chirp * nChirpLoad, 'uint16');
            neg = logical(bitget(adcData, 16));
            adcData(neg) = adcData(neg) - 2 ^16;
            adcData = complex(adcData(1 : 2 : end), adcData(2 : 2 : end));
            % 重组数据为[Rx, ADC, Tx, Chirp]
            adcDataComplex = reshape(adcData, nRx1Dev, nAdc1Chirp, nTx, nChirpLoad);
        else % 若ichirp不连续, 则依次取
            for iiChirp = 1 : nChirpLoad
                fseek(fp, ((iFrm - 1) * nIQ1Frm + (iChirp(nChirpLoad) - 1) * nIQ1Chirp) * 2, 'bof');
                adcData = fread(fp, nIQ1Chirp, 'uint16');
                neg = logical(bitget(adcData, 16));
                adcData(neg) = adcData(neg) - 2 ^ 16;
                adcData = complex(adcData(1 : 2 : end), adcData(2 : 2 : end));
                adcDataComplex(:, :, :, iiChirp) = reshape(adcData, nRx1Dev, nAdc1Chirp, nTx);
            end
        end
        fclose(fp);
    end

end
