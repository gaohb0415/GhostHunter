classdef trackParamStore < handle
% 存储追踪参数
% 作者: 刘涵凯
% 更新: 2023-3-7

    properties
        %% 常量部分
        % 场景
        limitR
        limitAng
        limitX
        limitY
        epsilon
        minpts
        % 帧
        iFrmLoad
        nFrmLoad
        % 卡尔曼滤波
        trackAlgo
        motionType
        % 模块开关
        staticEnhEn
        fgClusterEn
        ghostSupprEn
        linkEn
        backtrackEn
        ovlpProcEn
        waitZoneEn
        multiverseEn
        identifyEn
        paddingEn
        % 追踪
        % 基本逻辑
        candWin
        nFrmNotGhost
        presRatioNew
        presRatioDel
        nFrmLost
        waitAgeLmt
        % 鬼影标记
        guardAz
        guardFactor
        ghostAtten
        azShiftTh
        azFrontWidthTh
        % 点云重叠及分离
        persWidth
        persWidthTh
        distOvlp
        thOvlp
        sepIdleWin
        % 轨迹回溯
        nBacktrackMissTh
        % 轨迹续接
        retrvFrmDif
        retrvDistDif
        retrvDistDif1Frm
        % 静态目标增强
        staticEnhIntvl
        bodyRgWidth
        bodyAngWidth
        minptsStatic
        epsilonStatic
        staticEnhanceTh
        % 匈牙利算法
        costCand
        costConfirm
        costBacktrack
        extraCostGhost
        % 平滑
        smthMeth
        smthLen
        smthWin
        smthDifTh
        nSmthNewConfirm
        % 身份识别
        name
        identWin
        identIntvl
        costImu
        costPenal
        % 有效区间筛选
        dispTh
        dispIntvl
        effWin
        effWinSlide
        effWinGuard
        effRatio
        nFrmThId
        % 路径
        handleData
        handleDataImu
        handleTrkRslt

        %% 变量部分
        backtrackFlag
        sepDelFlag
        identLockFrm
        nPpl
        nMultiv
        anchorFrmMultiv
    end

    methods
        function renewProperty(obj, property, value)
            obj.(property) = value;
        end
    end

end
