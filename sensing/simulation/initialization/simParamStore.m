classdef simParamStore < handle
    % 存储仿真参数
    % 作者: 刘涵凯
    % 更新: 2024-11-29

    properties
        %% 路径设置
        handleSMPL
        handleTrajectory
        handleRemakeTrajectory
        handleResp
        handleSimSignal

        %% 人体模型
        smthTimeWin
        smthTimeLink
        smthTimeWinLink
        segPw

        %% 采样参数
        % 采样
        s
        tAdcStart
        fStart
        nAdc1Chirp
        adcRate
        nChirp1Frm
        tChirp
        tFrm
        nFrm
        % 属性
        tRamp
        fCenter
        lambda
        bw
        adcSlot
        fFrm
        % 功率
        friisFactor
        txAmpl

        %% 天线阵列
        % 雷达位置
        posRadar
        posArrayTx
        posArrayRx
        % 启用顺序
        orderTx
        orderRx
        nTx
        nRx
        nTransmit
        % 天线位置
        posTx
        posRx

        %% 轨迹重组
        nFrmRmk
        k
        NRp

        %% 信道失配
        channelMismatchEn
        channelMismatch

        %% 多径
        segPwGhost
        amplGhostFac
        reflector
        ghostWeakRatio
        ghostNumLmt

        %% 呼吸编码
        distAmplCorrect
        respDistMean
        respDistStd
        respCycleMean
        respCycleStd
        respCycleGuard
        code
        codeCycle
        codeRedun
        codePeriod
        decodeCycleWin
    end

    methods
        function renewProperty(obj, property, value)
            obj.(property) = value;
        end
    end

end
