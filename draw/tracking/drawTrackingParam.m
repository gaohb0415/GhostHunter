function p = drawTrackingParam(varargin)
% 追踪绘图的参数
% 作者: 刘涵凯
% 更新: 2023-5-22

%% 默认参数
param = inputParser();
param.CaseSensitive = false;
param.addOptional('idxCal', 0);
param.addOptional('shape', 'square'); % 'square' 'rectangle'
param.addOptional('tickEn', 1);
param.parse(varargin{:});
idxCal = param.Results.idxCal;
shape = param.Results.shape;
tickEn = param.Results.tickEn;

%% 坐标校准参数
switch idxCal
    case 0 % 无校准
        p.calDisp = [0, 0]; p.calRot = 0;
    case 1 % 23.4.24
        p.calDisp = [0, 0]; p.calRot = 1.5;
    case 2 % 23.5.5
        p.calDisp = [0, 0]; p.calRot = 2.5;
    case 3 % 23.5.9
        p.calDisp = [0, 0]; p.calRot = 1.5;
    case 4 % 23.7.17空旷
        p.calDisp = [0, -0.05]; p.calRot = 2;
    case 5 % 23.7.17复杂
        p.calDisp = [0, -0.05]; p.calRot = 2;
end

%% 轨迹颜色
load colorLib.mat
p.color.A = colorBlue;
p.color.B = colorOrange;
p.color.C = colorYellow;
p.color.D = colorGreen;
p.color.E = colorPurple;
p.color.N = colorGray;
p.color.num1 = p.color.A;
p.color.num2 = p.color.B;
p.color.num3 = p.color.C;
p.color.num4 = p.color.D;
p.color.num5 = p.color.E;
p.color.num6 = p.color.N;
p.color.num7 = p.color.N;
p.color.num8 = p.color.N;
p.color.num9 = p.color.N;
p.color.num10 = p.color.N;
p.color.lost = colorGray2;
% 渐变参数
p.colorGradProtect = 50; % 最近的colorGradProtect帧不渐变
p.colorGradFac1 = 0.25; % 影响最浅颜色
p.colorGradFac2 = 0.7;   % 影响变浅速度 % 0.4 0.7

%% Marker
load trackingMarker.mat
p.mkrTgt = targetMarker;

%% 图像参数
p.lineWidth = 4;
p.mkrRadar = '^';
p.mkrPc = '.';
p.mkrSizeRadar = 7;
p.mkrSizePc = 10;
p.mkrSizeTraj = 12;

%% 图框参数
p.xLim = [-3.2, 3.2];
p.yLim = [1.2, 6.8];
p.xTick = 0.4 + 0.8 * (- 10 : 10);
p.yTick = 0.8 * (- 10 : 10);
switch shape
    case 'square' % 正方形
        p.mkrSizeTgt = [0.7, 0.7];
        p.fontSize = 13;
        switch tickEn
            case 1 % 有刻度
                p.xLabel = 'X (m)';
                p.yLabel = 'Y (m)';
                p.gca = [1.5 1.45 7.4 6.5];
                p.gcf = [2 2 9 8];
            case 0 % 无刻度
                p.xLabel = 'X';
                p.yLabel = 'Y';
                p.gca = [0.85 0.8 8.05 7.15];
                p.gcf = [2 2 9 8];
        end
    case 'rectangle' % 长方形
                p.fontSize = 15;
        switch tickEn
            case 1 % 有刻度
                p.mkrSizeTgt = [0.5, 0.85];
                p.xLabel = 'X (m)';
                p.yLabel = 'Y (m)';
                p.gca = [1.7 1.7 12.2 6.2];
                p.gcf = [2 2 14 8];
            case 0 % 无刻度
                p.mkrSizeTgt = [0.5, 0.8];
                p.xLabel = 'X';
                p.yLabel = 'Y';
                p.gca = [0.9 0.9 13.0 7.0];
                p.gcf = [2 2 14 8];
        end
end
