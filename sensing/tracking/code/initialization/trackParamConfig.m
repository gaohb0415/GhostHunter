function p = trackParamConfig(varargin)
% 参数设置
% 输入:
% varargin:
% - trackRsltDelEn: 是否删除旧追踪结果. 1-是; 0-否
% 输出:
% p: 参数对象
% 作者: 刘涵凯
% 更新: 2023-3-7

%% 默认参数
param = inputParser();
param.CaseSensitive = false;
param.addOptional('trackRsltDelEn', 0);
param.parse(varargin{:});
trackRsltDelEn = param.Results.trackRsltDelEn;

%% 参数对象
p = trackParamShare.param;

%% 场景范围设置
limitT = [0, 10];              % 时间 s. 目前iFrmLoad(iFrm)和iFrm存在混淆使用, 因此起始时间应为0, 否则大概率出bug
p.limitR = [0.5, 8];          % 距离 m
p.limitAng = [-80, 80];   % 水平角 °
p.limitX = [-3.1, 3.1];      % 点云x范围 m [-3.1, 3.1]
p.limitY = [1.3, 6.7];       % 点云y范围 m [1.3, 6.7]
p.epsilon = 0.25;            % DBSCAN参数epsilon
p.minpts = 5;                 % DBSCAN参数minpts

%% 帧范围
load('config.mat', 'tFrm', 'nFrm')
iFrmStart = max(1, floor(limitT(1) / tFrm));            % 起始帧
iFrmEnd = min(nFrm , ceil(limitT(2) / tFrm) + 1);   % 结束帧
p.iFrmLoad = iFrmStart : iFrmEnd;                         % 读取的帧的索引
p.nFrmLoad = length(p.iFrmLoad);                        % 读取的帧数

%% 卡尔曼滤波
p.trackAlgo = 'NN';                              % 追踪算法. 'NN'-最近邻; 'KF'-卡尔曼滤波器
p.motionType = 'ConstantVelocity';     % 'ConstantVelocity'-匀速; 'ConstantAcceleration'-匀加速

%% 模块开关
p.staticEnhEn = 0;
p.fgClusterEn = 1;
p.ghostSupprEn = 1;
p.linkEn = 1;
p.backtrackEn = 1;
p.ovlpProcEn = 1;
p.waitZoneEn = 1;
p.multiverseEn = 1;
p.identifyEn = 1;
p.paddingEn = 1;

%% 追踪
% 基本逻辑
p.candWin = 20;                     % 候选区presence记录的帧窗
p.nFrmNotGhost = 5;             % 候选区转正的"连续无鬼影嫌疑记录"条件
p.presRatioNew = 0.9;            % 候选区转正的"存在占比"条件
p.presRatioDel = 0.5;              % 候选区删除的"存在占比"条件
p.nFrmLost = 100;                  % 轨迹处于miss状态连续nFrmLost帧后, 将其转入丢失区
p.waitAgeLmt = 80;                % 等待区age上限 帧 80
% 鬼影标记
p.guardAz = 1;                        % 计算角度范围阈值时的保护角度
p.guardFactor = 0.1;                % 计算角度范围阈值时的保护因子
p.ghostAtten = 0.9;                 % 鬼影衰减系数
p.azShiftTh = [30, 10, 5];         % 角度偏移阈值
p.azFrontWidthTh = 10;          % 居前簇角度宽度阈值
% 点云重叠及分离
p.persWidthTh = 0.95;                 % 细粒度聚类中的人体宽度阈值 m 0.95
p.persWidth = 0.75;                % 细粒度聚类中的人体宽度 m/人
p.distOvlp = 0.4;                     % 重叠判定的距离阈值
p.thOvlp = [0.6, 0.1, 0.3, 0.8];  % 重叠判定的cost/距离阈值
p.sepIdleWin = 40;                  % 分离记录的空闲窗 帧
% 轨迹回溯
p.nBacktrackMissTh = 10;       % 轨迹回溯的连续关联失败数上限
% 轨迹续接
p.retrvFrmDif = 20;                 % 轨迹续接时的失踪时间差上限 帧
p.retrvDistDif = [1.5, 0.5];        % 续接判断时的距离阈值基准
p.retrvDistDif1Frm = 0.1;        % 续接判断时的距离阈值步进
% 静态目标增强
p.staticEnhIntvl= 15;               % 静态目标增强周期 15
p.bodyRgWidth = 0.4;             % 点云生成时的距离宽度 m
p.bodyAngWidth = 20;           % 点云生成时的角度宽度 °
p.minptsStatic = 4;                  % 静态目标增强中DBSCAN的minpts
p.epsilonStatic = 0.3;               % 静态目标增强中DBSCAN的epsilon
p.staticEnhanceTh = 15;          % 对失迹/等待时间大于staticEnhanceTh帧的轨迹做静态目标增强
% 匈牙利算法
p.costCand = 0.3;                    % 候选区-点云簇关联cost阈值
p.costConfirm = 0.4;                % 确立区-点云簇关联cost阈值
p.costBacktrack = 0.6;             % 轨迹回溯关联cost阈值
p.extraCostGhost = 0.1;          % 鬼影簇额外cost

%% 平滑
p.smthMeth = 'gaussian';       % 'movmean' 'gaussian'
p.smthLen = 20;                      % 对轨迹的最后smoothLength个坐标进行平滑 15 20 10
p.smthWin = 12;                     % 轨迹平滑的窗口长度 8 12 6
p.smthDifTh = 0.3;                  % 轨迹平滑前后的位置偏移阈值 0.3
p.nSmthNewConfirm = 5;       % 轨迹回溯的平滑次数

%% 身份识别
p.name = ["A", "B", "C", "D", "E"];
p.identWin = 300;               % 身份识别帧窗
p.identIntvl = 20;                 % 身份识别周期
p.costImu = 1.0;                  % 匈牙利算法关联cost
p.costPenal = 10;                 % 匹配失败的惩罚cost 10

%% 有效区间筛选
p.dispTh = 0.25;                  % 对dispIntvl帧间位移大于dispTh m的部分进行匹配
p.dispIntvl = 10;
p.effWin = 40;                     % 有效区间筛选窗长度 帧
p.effWinSlide = 20;             % 有效区间筛选窗滑动距离 帧
p.effWinGuard = 5;             % 有效区间筛选保护时间 帧
p.effRatio = 0.5;                  % 比例阈值
p.nFrmThId = 40;                % 有效帧数>nFrmThId才参与身份识别

%% 保存绘图所需参数
if trackRsltDelEn
    % 删除旧追踪结果
    delete(p.handleTrkRslt);
    % 写入新参数
    limitX = p.limitX;
    limitY = p.limitY;
    save(p.handleTrkRslt, 'tFrm', 'limitX', 'limitY')
end
