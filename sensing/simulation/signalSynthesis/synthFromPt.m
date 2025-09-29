function simSignal = synthFromPt(pt, ampl)
% 通过反射点合成雷达信号
% 输入:
% 1. pt: 反射点坐标
% 2. ampl: 各点反射信号幅度
% 输出:
% simSignal: 仿真信号, [ADC, Chirp, Rx, Tx]
% 作者: 刘涵凯
% 更新: 2023-6-28

%% 参数对象
p = simParamShare.param;

%% 信号模拟
simSignal = zeros(p.nAdc1Chirp, p.nChirp1Frm, p.nRx, p.nTx);
for iTx = 1 : p.nTx
    % 该chirp的反射点
    posTx = p.posTx(iTx, :);
    posTx = [posTx(1), 0, posTx(2)]; % 加入y坐标(0)
    posTx = posTx + p.posRadar;
    for iRx = 1 : p.nRx
        iChannel = p.nRx * (iTx - 1) + iRx;
        posRx = p.posRx(iRx, :);
        posRx = [posRx(1), 0, posRx(2)]; % 加入y坐标(0)
        posRx = posRx + p.posRadar;
        for iChirp = 1 : p.nChirp1Frm
            ptTemp = pt(:, :, p.nTx * (iChirp - 1) + iTx);
            % 传播距离
            dist = sqrt(sum((ptTemp - posTx).^2, 2)) + sqrt(sum((ptTemp - posRx).^2, 2));
            % 传播时间
            tau = dist / physconst('LightSpeed');
            % 信号计算
            ph = 2 * pi * tau .* (p.fStart + p.s .* p.adcSlot);
            simSignal(:, iChirp, iRx, iTx) = sum(ampl .* exp(1i * ph), 1);
        end
        if p.channelMismatchEn % 信道失配
            simSignal(:, :, iRx, iTx) = repmat(p.channelMismatch(iChannel, :)', 1, p.nChirp1Frm) .* simSignal(:, :, iRx, iTx);
        end
    end
end
