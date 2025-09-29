function radarData = readTDA2_1Frm(iFrm)
% 读取MMWCAS-DSP-EVM采集的2243数据的某帧
% 适用于Master+Slave3
% 暂不支持自动切换文件
% 输入:
% iFrm: 要提取的帧
% 输出:
% radarData: 雷达数据矩阵, [ADC, Chirp, Rx, Tx]
% 作者: 刘涵凯
% 更新: 2023-8-28

%% 读取参数
load('config.mat', 'binFileHandleMaster', 'binFileHandleSlave3', 'nAdc1Chirp', 'nChirp1Frm', 'nFrm')
iFrm = iFrm + 1; % 硬件本身已删去一帧, 此处在其基础上再删一帧, 即, iFrm=1对应实际的第3帧
if iFrm > nFrm; error(['超出帧数限制(', num2str(nFrm), ')']); end
disp(['雷达数据读取: 第', num2str(iFrm - 1), '帧']);

%% 数据读取
nIQ1Frm = nAdc1Chirp * 6 * 4 * 2 * nChirp1Frm; % 每个芯片每帧的IQ信号数. 6, 4指6Tx4Rx. 2指1ADC=2IQ. 
radarDataMaster = readBinFile(binFileHandleMaster, iFrm, nIQ1Frm, 6, 4, nAdc1Chirp, nChirp1Frm); % Master
radarDataSlave3 = readBinFile(binFileHandleSlave3, iFrm, nIQ1Frm, 6, 4, nAdc1Chirp, nChirp1Frm); % Slave3
radarData = permute(cat(1, radarDataMaster, radarDataSlave3), [2, 4, 1, 3, 5]); % 重组数据为[ADC, Chirp, Rx, Tx, Frame]
end

function adcData = readBinFile(fileFullPath, iFrm, nIQ1Frm, nTx, nRx, nAdc1Chirp, nChirp1Frm)
fp = fopen(fileFullPath, 'r');
fseek(fp, ((iFrm - 1) * nIQ1Frm) * 2, 'bof');
adcData = fread(fp, nIQ1Frm, 'uint16');
neg = logical(bitget(adcData, 16));
adcData(neg) = adcData(neg) - 2^16;
adcData = complex(adcData(1 : 2 : end), adcData(2 : 2 : end));
% 重组数据为[Rx, ADC, Tx, Chirp]
adcData = reshape(adcData, nRx, nAdc1Chirp, nTx, nChirp1Frm);
fclose(fp);
end
