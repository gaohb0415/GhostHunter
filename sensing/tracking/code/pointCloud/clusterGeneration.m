function clusters = clusterGeneration(pcMode)
% 点云簇生成/载入
% 输入: 
% pcMode: 模式. 'generate'-生成点云; 'load'-载入点云
% 输出: 
% clusters: 聚类结果
% - .cluster: 簇
%    * .pc: 簇内点云坐标
%    * .centroid: 簇质心坐标
%    * .ghostLabel: 鬼影标记
% - .noise: 离散点
%    * .pc: 离散点坐标
% - .pcInput: 输入的点云坐标
% - .pw: 各点云的强度
% - .clusterIdx: 簇序号
% 作者: 刘涵凯
% 更新: 2023-3-8

%% 参数对象
p = trackParamShare.param;

%% 点云生成/读取
switch pcMode
    case 'generate'
        % 点云生成
        for iFrm = 1 : p.nFrmLoad
            radarData= readBin(p.iFrmLoad(iFrm), 0); % 提取某帧
            [fftRsltRg, ~] = fftRange(radarData); % Range FFT
            [~, pcRA] = dbfProc1D(fftRsltRg, 'limitR', p.limitR, 'limitAng', p.limitAng, 'pcEn', 1); % XY点云
            clusters(iFrm) = pcCluster2D([pcRA.x, pcRA.y], 'pw', pcRA.power, ...
                'epsilon', p.epsilon, 'minpts', p.minpts, 'limitX', p.limitX, 'limitY', p.limitY); % 聚类
            clusters(iFrm) = ghostInit(clusters(iFrm)); % 初始化鬼影标记
        end
        iFrmSaved = p.iFrmLoad; % 保存clusters的同时, 记录其帧范围
        save(p.handleData, 'clusters', 'iFrmSaved')

    case 'load'
        % 点云载入
        load(p.handleData) % 载入clusters
        % 时间范围校正
        if p.iFrmLoad(1) < iFrmSaved(1)
            warning('设定帧的左边界超出范围');
            p.iFrmLoad(p.iFrmLoad < iFrmSaved(1)) = [];
        end
        if p.iFrmLoad(end) > iFrmSaved(end)
            warning('设定帧的右边界超出范围');
            p.iFrmLoad(p.iFrmLoad > iFrmSaved(end)) = [];
        end
        clusters = clusters(ismember(iFrmSaved, p.iFrmLoad));
        p.nFrmLoad = length(p.iFrmLoad);
end

%% 保存绘图所需参数
iFrmLoad = p.iFrmLoad;
save(p.handleTrkRslt, '-append', 'iFrmLoad')
