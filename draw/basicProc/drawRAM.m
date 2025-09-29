function drawRAM(mapRA, rg, ang, varargin)
% 绘制Range-Angle Map, 含点云
% 输入: 
% 1. mapRA: Range-Angle实数矩阵
% 2. rg: 距离刻度
% 3. ang: 角度刻度
% 4. varargin
%     - pcRA: RA点云
%       * .range: 距离
%       * .angle: 角度
%     - logEn: 是否将RAM颜色幅度设为dB. 0-否; 1-是
% 作者: 刘涵凯
% 更新: 2022-7-11

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('pcRA', struct('range', []));
p.addOptional('logEn', 0);
p.parse(varargin{:});
pcRA = p.Results.pcRA;
logEn = p.Results.logEn;

%% 图像参数
fontSize=16;

%% 绘图
figure
if isempty(pcRA.range) % 判断是否绘制点云
    polarPcolor(rg', ang', mapRA, 'colBar', 0, 'Ncircles', 5, 'Nspokes', 7, 'textFontSize', fontSize, axisEn = 0);
    % polarPcolor(rg', ang', mapRA, 'colBar', 0, 'Ncircles', 4, 'Nspokes', 7, 'textFontSize', fontSize, 'textEn', 0);
    % polarPcolor(rg', ang', mapRA, 'colBar', 0, 'Ncircles', 4, 'Nspokes', 7, 'textFontSize', fontSize, 'RtickLabel', [{'0'}, {'2.5'}, {'5'}, {'10'}], axisEn = 0);
else
    pcRA.range = pcRA.range / rg(end);
    polarPcolor(rg', ang', mapRA, 'colBar', 0, 'Ncircles', 5, 'Nspokes', 7, 'pcPlot', 1, ...
        'pointCloud', [pcRA.range, pcRA.angle], 'textFontSize', fontSize);
end

%% 图像设置
if logEn % 将幅度转化为dB
    set(gca, 'ColorScale', 'log')
end
set(gca, 'Fontsize', fontSize);
set(gca, 'LooseInset', get(gca, 'TightInset'))
set(gcf, 'color', 'w')
set(gca, 'Units', 'centimeters', 'position', [1.1 -0.7 13.3 9])
set(gcf, 'Units', 'centimeters', 'Position', [2 2 15.4 7.7]);
