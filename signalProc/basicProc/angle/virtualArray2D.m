function virtualArray = virtualArray2D(radarData, doaAlgo)
% 将原始雷达数据矩阵转换为二维虚拟阵列数据矩阵
% 输入: 
% 1. radarData: 雷达数据矩阵, [ADC/Range, Chirp/Velocity, Rx, Tx]
% 2. doaAlgo: DoA算法类型. 'DBF'或'FFT'
%     'DBF'-输出各阵元的相对位置及数据
%     'FFT'-将阵元按相对位置排列, 需要时补零
% 输出: 
% virtualArray: 雷达数据矩阵及虚拟阵列位置
% - .signal
%   doaAlgo为'DBF': 雷达数据矩阵, [Antenna, Chirp/Velocity, ADC/Range]
%   doaAlgo为'FFT': 雷达数据矩阵, [ADC/Range, Chirp/Velocity, Azimuth, Elevation]
% - .arrayPosX: 虚拟阵列的水平相对位置
% - .arrayPosZ: 虚拟阵列的垂直相对位置
% 作者: 刘涵凯
% 更新: 2022-7-3

%% 载入雷达配置和虚拟阵列相对位置
load('config.mat', 'device', 'arrayType')
load array2D.mat

%% 获取数据矩阵尺寸
[nAdc, nChirp, nRx, nTx] = size(radarData);

%% 构建虚拟阵列
switch doaAlgo
    case 'DBF' % 将Tx, Rx维度重排为1个维度
        switch device
            case '2243'
                switch arrayType
                    case 'half' % Tx1~3+10~12
                        virtualArray.arrayPosX = array2D.pos.cascade.half(:, 1);
                        virtualArray.arrayPosZ = array2D.pos.cascade.half(:, 2);
                        virtualArray.signal = reshape(radarData, [nAdc, nChirp, nTx * nRx]);
                        virtualArray.signal = virtualArray.signal(:, :, array2D.iSort.cascade.half);
                end
        end
        virtualArray.signal = permute(virtualArray.signal, [3, 2, 1]);

    case 'FFT' % 构建位置均匀连续的信号向量
        % 没写的配置等用到时再写
        switch device
            case '2243'
                switch arrayType
                    case 'half'
                        signalRaw = reshape(radarData, [nAdc, nChirp, 6 * nRx]); % 将两个天线维度合并
                        signalRaw = signalRaw(:, :, array2D.iSort.cascade.half); % 将数据按天线位置顺序重排
                        virtualArray.signal = zeros(nAdc, nChirp, 26 * 7); % 完整阵列尺寸的全零矩阵, 用于补零
                        virtualArray.signal(:, :, array2D.iGlobal.cascade.half, :) = signalRaw; % 补零(即向全零矩阵的相应位置覆盖数据)
                        virtualArray.signal = reshape(virtualArray.signal, [nAdc, nChirp, 26, 7]); % 将水平和垂直维度分开
                        % 用插值而非补零的方式补全阵列
                        % 现在写得很逆天, 计算量极大(虽然实际耗时不高)
                        % 考虑到实际上很少用到2D Angle FFT, 以后有需要再改吧, 当前版本先为了演示凑合用
                            for iChirp = 1 : nChirp
                                for iAdc = 1 : nAdc

                                    % 用高度1的数据将高度7补全
                                    phRealHoriz1= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 23 : 26, 1))));
                                    phDifHoriz1 = phRealHoriz1(2 : 4) - phRealHoriz1(1);
                                    % 这个for和之后的for都可以省去, 以后再写
                                    for iRxVirtual = 24 : 26
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 7) = ...
                                            virtualArray.signal(iAdc, iChirp, 23, 7) * exp(1i * phDifHoriz1(iRxVirtual - 23));
                                    end

                                    % 用高度7的数据将高度1补全
                                    % 第一间隔
                                    phRealHoriz7= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 1 : 12, 7))));
                                    phDifHoriz7 = phRealHoriz7(1 : 11) - phRealHoriz7(12);
                                    for iRxVirtual = 1 : 11
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 1) = ...
                                            virtualArray.signal(iAdc, iChirp, 12, 1) * exp(1i * phDifHoriz7(iRxVirtual));
                                    end
                                    % 第二间隔
                                    phRealHoriz7= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 16 : 23, 7))));
                                    phDifHoriz7 = phRealHoriz7(1 : 7) - phRealHoriz7(8);
                                    for iRxVirtual = 16 : 22
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 1) = ...
                                            virtualArray.signal(iAdc, iChirp, 23, 1) * exp(1i * phDifHoriz7(iRxVirtual - 15));
                                    end

                                    % 用高度7的数据将高度3补全
                                    % 第一间隔
                                    phRealHoriz7= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 1 : 11, 7))));
                                    phDifHoriz7 = phRealHoriz7(1 : 10) - phRealHoriz7(11);
                                    for iRxVirtual = 1 : 10
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 3) = ...
                                            virtualArray.signal(iAdc, iChirp, 11, 3) * exp(1i * phDifHoriz7(iRxVirtual));
                                    end
                                    % 第二间隔
                                    phRealHoriz7= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 15 : 22, 7))));
                                    phDifHoriz7 = phRealHoriz7(1 : 7) - phRealHoriz7(8);
                                    for iRxVirtual = 15 : 21
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 3) = ...
                                            virtualArray.signal(iAdc, iChirp, 23, 3) * exp(1i * phDifHoriz7(iRxVirtual - 14));
                                    end
                                    % 第三间隔
                                    phRealHoriz7= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 25 : 26, 7))));
                                    phDifHoriz7 = phRealHoriz7(2) - phRealHoriz7(1);
                                    virtualArray.signal(iAdc, iChirp, 26, 3) = ...
                                        virtualArray.signal(iAdc, iChirp, 25, 3) * exp(1i * phDifHoriz7);

                                    % 用高度7的数据将高度6补全
                                    % 第一间隔
                                    phRealHoriz7= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 1 : 10, 7))));
                                    phDifHoriz7 = phRealHoriz7(1 : 9) - phRealHoriz7(10);
                                    for iRxVirtual = 1 : 9
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 6) = ...
                                            virtualArray.signal(iAdc, iChirp, 10, 6) * exp(1i * phDifHoriz7(iRxVirtual));
                                    end
                                    % 第二间隔
                                    phRealHoriz7= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 14 : 21, 7))));
                                    phDifHoriz7 = phRealHoriz7(1 : 7) - phRealHoriz7(8);
                                    for iRxVirtual = 14 : 20
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 6) = ...
                                            virtualArray.signal(iAdc, iChirp, 23, 6) * exp(1i * phDifHoriz7(iRxVirtual - 13));
                                    end
                                    % 第三间隔
                                    phRealHoriz1= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, 24 : 26, 1))));
                                    phDifHoriz1 = phRealHoriz1(2 : 3) - phRealHoriz1(1);
                                    for iRxVirtual = 25 : 26
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 6) = ...
                                            virtualArray.signal(iAdc, iChirp, 24, 6) * exp(1i * phDifHoriz1(iRxVirtual - 24));
                                    end

                                    % 用高度6和7算垂直相位差
                                    phRealVert= unwrap(angle(squeeze(virtualArray.signal(iAdc, iChirp, :, 6 : 7))), [], 2);
                                    phDifVert = phRealVert(:, 2) - phRealVert(:, 1);

                                    % 用高度1的数据将高度2补全
                                    for iRxVirtual = 1 : 26
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 2) = ...
                                            virtualArray.signal(iAdc, iChirp, iRxVirtual, 1) * exp(1i * phDifVert(iRxVirtual));
                                    end

                                    % 用高度3的数据将高度4补全
                                    for iRxVirtual = 1 : 26
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 4) = ...
                                            virtualArray.signal(iAdc, iChirp, iRxVirtual, 3) * exp(1i * phDifVert(iRxVirtual));
                                    end

                                    % 用高度6的数据将高度5补全
                                    for iRxVirtual = 1 : 26
                                        virtualArray.signal(iAdc, iChirp, iRxVirtual, 5) = ...
                                            virtualArray.signal(iAdc, iChirp, iRxVirtual, 6) * exp(-1i * phDifVert(iRxVirtual));
                                    end
                                end
                            end
                end
        end
end
