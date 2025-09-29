function [pwRA, pcRA] = dbfProc1D(radarData, varargin)
% 1D DBF处理, 含点云生成和点云信号重构
% 输入:
% 1. radarData: 雷达数据矩阵, [ADC/Range, Chirp, Rx, Tx]
% 2. varargin:
%     - limitR: 距离范围. []-不设范围
%     - rg: 距离刻度. []-从0开始的默认刻度
%     - limitAng: 角度范围
%     - resAng: 角度步进间隔
%     - pcEn: 是否计算点云. 0-否; 1-是
%     - cfarPfa: 手动设定CFAR PFA
%     - sigReconsEn: 是否执行信号重构. 0-否; 1-是
%     - velocityEn: 是否计算点云速度. 0-否; 1-是
%     - drawEn: 是否绘图. 0-否; 1-是
%     - logEn: 是否将RAM颜色幅度设为dB. 0-否; 1-是
% 输出:
% 1. pwRA: 反射强度矩阵, [ADC/Range, Angle]
% 2. pcRA: RA点云
%     - .iRange: range bin索引
%     - .iAngle: angle bin索引
%     - .range: 距离
%     - .angle: 角度
%     - .x: x坐标
%     - .y: y坐标
%     - .velocity: 速度
%     - .power: 反射强度
%     - .signal: 重构信号, [Chirp, 点云]
% 作者: 刘涵凯
% 更新: 2023-8-28

%% 完整运行流程：
%% 1. 调用该函数，循环到了一个距离（圆圈）
%% 2. 调用dbf，传入要处理的角度列表
%% 3. dbf返回当前距离上每个角度对应的反射强度，一整串的值就构成了当前距离的空间功率图谱
%% 4. 在该图谱上寻找峰值


%% 对于这个dbfProc1D函数，需要调整的参数就是下面三个方面：
%% 1. 图像的清晰度（分辨率）
%% 2. 目标检测的准确性（CFAR）
%% 3. 处理的效率（范围限定）
%% 默认参数
%% 雷达图像模糊、无法情绪分辨目标，可以尝试减少角度分辨率resAng
%% 雷达处理速度很慢（只关心特定区域的图像）更改limitR距离范围和limitAng角度范围
p = inputParser();         % 配置函数"开关"
p.CaseSensitive = false;
p.addOptional('limitR', []);        % 限定距离范围
p.addOptional('rg', []);
p.addOptional('limitAng', [-90, 90]);   % 限定角度范围
p.addOptional('resAng',0.5);             % 角度分辨率/步进，分辨率变小、划分越精细，但是计算量显著增大
p.addOptional('pcEn', 0);
p.addOptional('cfarPfa', []); % 当需要用到不同的CFAR PFA时, 输入此值
p.addOptional('sigReconsEn', 0);        % 是否进行信号重构
p.addOptional('velocityEn', 0);         % 是否计算每个点的速度
p.addOptional('drawEn', 0);
p.addOptional('logEn', 0);
p.parse(varargin{:});

limitR = p.Results.limitR;
rg = p.Results.rg;
limitAng = p.Results.limitAng;
resAng = p.Results.resAng;
pcEn = p.Results.pcEn;
cfarPfa = p.Results.cfarPfa;
sigReconsEn = p.Results.sigReconsEn;
velocityEn = p.Results.velocityEn;
drawEn = p.Results.drawEn;
logEn = p.Results.logEn;

% 参数优先级: velocityEn > sigReconsEn > pcEn
% 测速这个高级功能必须以信号重构为前提
if velocityEn; sigReconsEn = 1; end
if sigReconsEn; pcEn = 1; end


%% 雷达参数
load('config.mat', 'resR', 'spacingCal', 'resV')
nRg = size(radarData, 1);
if isempty(rg); rg = resR * (0 : nRg - 1)'; end % 总距离刻度

%% 提取所选距离范围内的数据
%% 根据你输入的 limitR，从 radarData 中把对应距离范围的数据切片出来，后续只处理这部分数据
if any(limitR)
    aoiRg = [max(limitR(1), rg(1)), min(limitR(2), rg(end))];  % 距离AOI(area of interest)
    iAoiRg = rg >= aoiRg(1) & rg <= aoiRg(2);
    rg = rg(iAoiRg);
    radarData = radarData(iAoiRg, :, :, :);
end

%% 生成虚拟阵列（目的是同时可以采集多个方向的反射强度：形成圆环）
%% 假设你的雷达有 nTx 个发射天线和 nRx 个接收天线。
%% 通过让 nTx 个天线轮流发射信号，nRx 个天线同时接收，你可以等效出一个拥有 nTx * nRx 个接收天线的虚拟阵列
antArray = virtualArray1D(radarData, 'DBF');

%% 获取数据尺寸与刻度
nRg = size(antArray.signal, 3);
ang = (max(limitAng(1), -90) : resAng : min(limitAng(end), 90))'; % 扫描角度
nAng = length(ang);

%% 执行DBF数字波束形成(核心计算)
% 初始化反射强度和权重结果
pwRA = zeros(nAng * nRg, 1);

%% 提取各距离的信号进行DBF计算
%% （波束扫描）：基础的波束形成算法分为CBF和Capon
%% 需要高刷新率的实时人体追踪应用。每次只有一个人或者少数几个人在场，并且距离很远（CBF）
%% 高分辨场景或者刷新率分辨率均衡场景（Capon不需要更改dbf中配置）
%% 代码逐个距离进行处理。可以想象成把雷达的探测区域按距离切成了一片片的“同心圆环"
for iRg = 1 : nRg
    % 将[nAngle, nRange]矩阵"拉伸"成[nAngle * nRange]向量后, 第iRange个距离的角度在该向量中的索引
    iBinRA = (iRg - 1) * nAng + 1 : iRg * nAng;
    % 执行DBF, 获得反射强度
    [pwRA(iBinRA), ~] = dbf(ang, [], antArray.signal(:, :, iRg), antArray.arrayPos, [], 'spacingCal', spacingCal);
end
% 注意下面的维度顺序
% 波束扫描之后，将pwRA这个单一长向量，重新组织成一个二维矩阵[nAng, nRg]
pwRA = reshape(pwRA, [nAng, nRg]);
% 行代表距离Range，列代表角度Angle
pwRA = pwRA'; % 将维度转换为为[Range, Angle]，这就是最终的距离-角度图矩阵（热力图）

%% 点云生成（2D-CFAR）
%% 将上面的 距离-角度 热力图中的离散的、有意义的目标点的信息打包成一个结构化的数据"点云"
%% pcRA：最终处理结果的存放地点（空的点云结构体）
pcRA = struct('iRange', [], 'iAngle', [], 'range', [], 'angle', [], 'x', [], 'y', [], 'velocity', [], 'power', [], 'signal', []);
if pcEn
    % 2D CFAR
    % 将config.mat中雷达的数据写入到cfarParamRA中

    load('config.mat', 'cfarParamRA')
    if ~isempty(cfarPfa); cfarParamRA.pfa = cfarPfa; end % 优先采用调用函数时设置的PFA
    [pcRA.iRange, pcRA.iAngle, ~] = cfar2D(pwRA, cfarParamRA); % 执行CFAR
    if isempty(pcRA.iRange); warning('未检测到RA点云'); end
    % 计算点云信息
    if ~isempty(pcRA.iRange)
        powerVec = pwRA(:); % 将RA矩阵转换为向量, 用于获得点云反射强度
        pcRA.range = rg(pcRA.iRange);
        pcRA.angle = ang(pcRA.iAngle);
        pcRA.x = pcRA.range .* sind(pcRA.angle);
        pcRA.y = pcRA.range .* cosd(pcRA.angle);
        iRA = sub2ind([nRg, nAng], pcRA.iRange, pcRA.iAngle);
        pcRA.power = powerVec(iRA);
    end
    nPc = length(pcRA.iRange); % 总点云数
end

%% 信号重构
%% 在找到一个目标（比如在3米，-15°）之后
%% dbfRecons 函数会返回到最原始的虚拟天线数据，然后利用DBF的原理，只提取朝向-15°方向的信号，并把它合成出来
%% 相当于进行了一次“空间滤波”，从杂乱的混合信号中，单独分离出了只属于这一个目标的纯净信号 
%% pcRA.signal。这个信号包含了该目标在一个完整帧内的所有Chirp信息
if sigReconsEn && nPc
    pcRA.signal = dbfRecons(radarData, [pcRA.iRange, pcRA.angle]);
end

%% 计算点云速度(使用的是上一步信号重构之后的纯净信号pcRA.signal)
if velocityEn && nPc
    nChirp = size(radarData, 2);
    vel = resV * (-nChirp / 2 : nChirp / 2 - 1)'; % 速度轴
    pcRA.velocity = zeros(nPc, 1); % 初始化
    for iPc = 1 : nPc
        [fftRsltDop, ~] = fftDoppler(pcRA.signal(:, iPc).'); % 注意非共轭转置
        fftRsltDop = abs(fftRsltDop);
        % 若速度峰不够强, 即强度占比低于某阈值, 则将点云速度设为0
        if max(fftRsltDop) < mean(fftRsltDop) * 5
            pcRA.velocity(iPc) = vel(nChirp / 2 + 1);
        else
            [~, velSort] = sort(fftRsltDop, 'descend');
            % 若0速度左右的两个速度是速度谱中最强的两个速度, 则将点云速度设为0
            if ismember(nChirp / 2, velSort([1, 2])) && ismember(nChirp / 2 + 2, velSort([1, 2]))
                pcRA.velocity(iPc) = vel(nChirp / 2 + 1);
            else % 否则, 取强度最大的速度作为点云速度
                pcRA.velocity(iPc) = vel(velSort(1));
            end
        end
    end
end

%% 绘图(直接绘制雷达热力图)
%{
if drawEn
    % 解决使用polarPcolor绘图时角度与色块的对齐问题
    pwRA = [pwRA, pwRA(:, end)];
    ang = [ang(1); ang(2 : end) - resAng / 2; ang(end)];
    drawRAM(pwRA, rg, ang, 'pcRA', pcRA, 'logEn', logEn); drawnow;
end
%}

%% 绘图(绘制雷达热力图并且可以查看图中的点的速度)
if drawEn
    % --- 步骤 1: 调用你现有的函数，画出完整的背景热力图 ---
    % (我们假设 drawRAM 会创建一个新的 figure 或者 axes)
    % 为了避免它画出原有的红点，最理想的情况是修改drawRAM，让它只画背景。
    % 但如果不能修改，下面的代码会用带颜色的新点覆盖掉它画的旧点。
    drawRAM(pwRA, rg, ang, 'pcRA', pcRA, 'logEn', logEn); 
    
    hold on; % 关键：保持住当前图像，允许在上面继续画，而不会覆盖掉背景

    % --- 步骤 2: 绘制带有速度颜色编码的点云 ---
    % 确保点云存在，并且速度已经被计算
    if pcEn && velocityEn && ~isempty(pcRA.x)
        
        % 定义点的大小和颜色数据
        pointSize = 50;           % 你可以根据喜好调整点的大小
        colors = pcRA.velocity;   % 核心！直接用速度向量作为每个点的颜色数据
        
        % 使用 scatter 函数进行绘制
        scatter(pcRA.x, pcRA.y, pointSize, colors, 'filled', 'MarkerEdgeColor', 'k');
        
        % --- 关键参数解释 ---
        % pcRA.x, pcRA.y:    点云的笛卡尔坐标
        % pointSize:         指定所有点的大小
        % colors:            为每个点指定一个用于颜色映射的数值（这里就是速度）
        % 'filled':          将标记填充为实心
        % 'MarkerEdgeColor','k': 给每个点加上一个黑色的轮廓，让它在热力图上更显眼
        
    end

    % --- 步骤 3: 添加辅助元素，让图更容易理解 ---
    
    % 设置颜色映射表
    colormap('jet'); % 'jet' 是一个常用的颜色表(蓝->红), 'coolwarm' 也是很好的选择
    
    % 添加颜色条 (Colorbar)，并给它加上标签
    h = colorbar;
    ylabel(h, '速度 (m/s)'); % 解释颜色代表的物理意义和单位

    % (可选，但推荐) 固定颜色条的范围，使多次绘图具有可比性
    % 比如，如果你知道最大速度不会超过5m/s，可以这样设置：
    % maxVel = 5;
    % caxis([-maxVel, maxVel]); 
    
    hold off; % 释放图像，之后的绘图指令会新建画布
    drawnow;
end
