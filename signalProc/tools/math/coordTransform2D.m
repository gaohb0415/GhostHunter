function coordOut = coordTransform2D(coordIn, rot, trans, varargin)
% 二维坐标变换
% 如果输入的是三维坐标, 则第三维数据不做改变
% 输入:
% 1. coordIn: 坐标
% 2. rot: 旋转角度 右手螺旋, °
% 3. trans: 位移, m
% 4. varargin:
%     - firstOperation: 先进行旋转还是平移. 'rot'; 'disp'
% 输出: 
% coordOut: 坐标转换后的坐标
% 作者: 刘涵凯
% 更新: 2023-6-7

%% 默认参数
p = inputParser();
p.CaseSensitive = false;
p.addOptional('firstOperation', 'rot');
p.parse(varargin{:});
firstOperation = p.Results.firstOperation;

%% 判断是否为三维坐标
judge3D = 0;
if size(coordIn, 2) == 3
    judge3D = 1;
    z = coordIn(:, 3);
    coordIn = coordIn(:, [1, 2]);
end

%% 坐标转换
if isempty(coordIn) || (isempty(rot) && isempty(trans))
    % 不进行坐标转换
    coordOut = coordIn;
elseif ~isempty(rot) && isempty(trans)
    % 仅旋转
    coordOut = [coordIn(:, 1) .* cosd(rot) - coordIn(:, 2) .* sind(rot), coordIn(:, 1) .* sind(rot) + coordIn(:, 2) .* cosd(rot)];
elseif isempty(rot) && ~isempty(trans)
    % 仅平移
    coordOut = coordIn + trans;
else
    switch firstOperation
        case 'rot'
            % 先旋转后平移
        coordOut = [coordIn(:, 1) .* cosd(rot) - coordIn(:, 2) .* sind(rot), coordIn(:, 1) .* sind(rot) + coordIn(:, 2) .* cosd(rot)];
        coordOut = coordOut + trans;
        case 'disp'
            % 先平移后旋转
            coordOut = coordIn + trans;
            coordOut = [coordOut(:, 1) .* cosd(rot) - coordOut(:, 2) .* sind(rot), coordOut(:, 1) .* sind(rot) + coordOut(:, 2) .* cosd(rot)];
    end
end

%% 恢复三维坐标
if judge3D; coordOut = [coordOut, z]; end
