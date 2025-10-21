function drawRAMnew(mapRA, rg, ang, varargin)
% 绘制Range-Angle Map, 含点云 (New Version)
%
% 功能更新:
% - 支持在指定的坐标系(axes)上绘图，避免创建新窗口。
% - 通过可选参数 'ax' 传入坐标系句柄。
% - 若不提供'ax'参数，则行为与旧版完全相同（创建新窗口），保证向后兼容。
%
% 输入: 
% 1. mapRA: Range-Angle实数矩阵
% 2. rg: 距离刻度
% 3. ang: 角度刻度
% 4. varargin
%     - 'pcRA': RA点云结构体
%     - 'logEn': 是否使用dB尺度 (0-否; 1-是)
%     - 'ax': (新增) 用于绘图的坐标系句柄
%
% 作者: 刘涵凯 (由Gemini根据需求修改)
% 更新: 2025-10-20

%% 1. 参数解析
p = inputParser();
p.CaseSensitive = false;
p.addOptional('pcRA', struct('range', []));
p.addOptional('logEn', 0);
% --- 新增核心参数: 'ax'，用于接收外部传入的坐标系句柄 ---
p.addParameter('ax', [], @(x) isempty(x) || isa(x, 'matlab.graphics.axis.Axes'));
p.parse(varargin{:});

pcRA = p.Results.pcRA;
logEn = p.Results.logEn;
ax = p.Results.ax;

%% 2. 准备绘图目标
% --- 核心逻辑: 判断在哪里绘图 ---
if isempty(ax)
    % 如果外部没有指定坐标系(ax为空)，则维持旧行为：自己创建新窗口。
    figure;
    ax = gca; % 获取这个新窗口的坐标系句柄
else
    % 如果外部指定了坐标系，则将其激活为当前绘图目标。
    % 这样做可以确保后续的绘图指令(如polarPcolor)和设置指令(如title)都作用于正确的位置。
    axes(ax); 
end

% 获取当前坐标系所在的图形窗口句柄，用于后续设置
fig = ax.Parent;

%% 3. 绘图
fontSize = 16;

% 使用 cla(ax) 清空目标坐标系，而不是clf清空整个窗口
cla(ax, 'reset');

if isempty(pcRA.range) % 判断是否绘制点云
    polarPcolor(rg', ang', mapRA, 'colBar', 0, 'Ncircles', 5, 'Nspokes', 7, 'textFontSize', fontSize, 'axisEn', 0);
else
    % 注意：原代码中这行有潜在问题，它会修改传入的pcRA结构体。
    % pcRA.range = pcRA.range / rg(end); 
    % 更安全的做法是创建一个临时变量
    temp_pc_range = pcRA.range / rg(end);
    polarPcolor(rg', ang', mapRA, 'colBar', 0, 'Ncircles', 5, 'Nspokes', 7, 'pcPlot', 1, ...
        'pointCloud', [temp_pc_range, pcRA.angle], 'textFontSize', fontSize);
end

%% 4. 图像设置 (所有gca/gcf都用ax/fig句柄替代，更精确)
if logEn % 将幅度转化为dB
    set(ax, 'ColorScale', 'log')
end

set(ax, 'Fontsize', fontSize);
set(ax, 'LooseInset', get(ax, 'TightInset'))
set(ax, 'Units', 'centimeters', 'position', [1.1 -0.7 13.3 9])

set(fig, 'color', 'w')
set(fig, 'Units', 'centimeters', 'Position', [2 2 15.4 7.7]);

end