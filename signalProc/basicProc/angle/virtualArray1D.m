function virtualArray = virtualArray1D(radarData, doaAlgo)
% 将原始雷达数据矩阵转换为一维虚拟阵列数据矩阵
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC/Range, Chirp/Velocity, Rx, Tx]
% 2. doaAlgo: DoA算法类型. 'DBF'或'FFT'
%     'DBF'-输出各阵元的相对位置及数据
%     'FFT'-将阵元按相对位置排列, 需要时补零
% 输出:
% virtualArray: 雷达数据矩阵及虚拟阵列位置
% - .signal
%   doaAlgo为'DBF': 雷达数据矩阵, [Antenna, Chirp/Velocity, ADC/Range]
%   doaAlgo为'FFT': 雷达数据矩阵, [ADC/Range, Chirp/Velocity, Antenna]
% - .arrayPos: 虚拟阵列水平位置向量(仅'DBF'输出此值)
% 作者: 刘涵凯
% 更新: 2022-8-24

%% 载入雷达配置和虚拟阵列相对位置
load('config.mat', 'device', 'arrayType')
load array1D.mat

%% 获取数据矩阵尺寸
[nAdc, nChirp, nRx, nTx] = size(radarData);

%% 构建虚拟阵列
switch doaAlgo
    case 'DBF' % 将Tx, Rx维度重排为1个维度
        switch device
            case '2243'
                switch arrayType
                    case 'half'
                        switch nTx
                            case 1 % Tx1(仅使能master的Tx1)
                                virtualArray.arrayPos = array1D.pos.cascade.half.txSet1;
                                virtualArray.signal = reshape(radarData, [nAdc, nChirp, nRx]);
                                virtualArray.signal = virtualArray.signal(:, :, array1D.iSort.cascade.half.txSet1);
                            case 3 % Tx10~12(仅使能slave3的3个Tx)
                                virtualArray.arrayPos = array1D.pos.cascade.half.txSet2;
                                virtualArray.signal = reshape(radarData, [nAdc, nChirp, 3 * nRx]);
                                virtualArray.signal = virtualArray.signal(:, :, array1D.iSort.cascade.half.txSet2);
                            case 6 % Tx1~3+10~12(使能全部6个Tx, 取Tx10~12)
                                virtualArray.arrayPos = array1D.pos.cascade.half.txSet3;
                                virtualArray.signal = reshape(radarData(:, :, :, 4 : 6), [nAdc, nChirp, 3 * nRx]);
                                virtualArray.signal = virtualArray.signal(:, :, array1D.iSort.cascade.half.txSet3);
                        end
                end
            case '1843'
                switch arrayType
                    case '1T4R'
                        virtualArray.arrayPos = array1D.pos.xwr1843.array1.txSet1;
                        virtualArray.signal = reshape(radarData, [nAdc, nChirp, nRx]); % 将两个天线维度合并
                        virtualArray.signal = virtualArray.signal(:, :, array1D.iSort.xwr1843.array1.txSet1);
                end
        end
        % 将数据矩阵维度由[Range, Chirp, Rx]转换为[Rx, Chirp, Range], 方便后续处理
        virtualArray.signal = permute(virtualArray.signal, [3, 2, 1]);

    case 'FFT' % 构建位置均匀连续的信号向量
        % 没写的配置等用到时再写
        switch device
            case '2243'
                switch arrayType
                    case 'half'
                        switch nTx
                            case 1 % Tx1(仅使能master的Tx1)
                                signalRaw = reshape(radarData, [nAdc, nChirp, nRx]); % 将两个天线维度合并
                                signalRaw = signalRaw(:, :, array1D.iSort.cascade.half.txSet1); % 将数据按天线位置顺序重排
                                virtualArray.signal = zeros(nAdc, nChirp, 15); % 完整阵列尺寸的全零矩阵, 用于补零
                                virtualArray.signal(:, :, array1D.iGlobal.cascade.half.txSet1.real) = signalRaw;
                                % 用插值而非补零的方式补全阵列
                                    for iChirp = 1 : nChirp
                                        for iAdc = 1 : nAdc
                                            phReal = unwrap(angle(squeeze(signalRaw(iAdc, iChirp, :))));
                                            phDif = ((phReal(4) - phReal(1)) / 3 + (phReal(8) - phReal(5)) / 3) / 2;
                                            % 这个for可以省去, 以后再改
                                            for iRxVirtual = 5 : 11
                                                virtualArray.signal(iAdc, iChirp, iRxVirtual) = ...
                                                    signalRaw(iAdc, iChirp, 4) * exp(1i * (iRxVirtual - 4) * phDif);
                                            end
                                        end
                                end
                            case 3 % Tx10~12(仅使能slave3的3个Tx)
                                virtualArray.signal = reshape(radarData, [nAdc, nChirp, 3 * nRx]);
                                virtualArray.signal = virtualArray.signal(:, :, array1D.iSort.cascade.half.txSet2);
                            case 6 % Tx1~3+10~12(使能全部6个Tx, 取Tx10~12)
                                virtualArray.signal = reshape(radarData(:, :, :, 4 : 6), [nAdc, nChirp, 3 * nRx]);
                                virtualArray.signal = virtualArray.signal(:, :, array1D.iSort.cascade.half.txSet3);
                        end
                end
            case '1843'
                switch arrayType
                    case '1T4R'
                        virtualArray.signal = reshape(radarData, [nAdc, nChirp, nRx]); % 将两个天线维度合并
                        virtualArray.signal = virtualArray.signal(:, :, array1D.iSort.xwr1843.array1.txSet1);
                end
        end
end
