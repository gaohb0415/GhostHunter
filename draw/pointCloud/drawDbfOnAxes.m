function [pwRA, pcRA] = drawDbfOnAxes(h_axes, radarData, params_cell)
% 这是一个包装函数，用于在指定的坐标轴(axes)上调用并绘制dbfProc1D的结果。
% 它不会修改 dbfProc1D.m 文件本身。
%
% 输入:
%   h_axes          - 目标坐标轴的句柄 (Handle), 即你希望在哪里绘图。
%   radarData       - 传入 dbfProc1D 的雷达数据。
%   params_cell     - 一个元胞数组(cell array)，包含所有需要传递给
%                     dbfProc1D 的'键'-'值'对参数。
%
% 输出:
%   pwRA, pcRA      - 从 dbfProc1D 返回的原始数据。

% --- 步骤 1: 保存当前的绘图状态 ---
% 记录在调用此函数之前，哪个坐标轴是激活的，以便后续恢复
original_axes = gca;
% 记录目标坐标轴原始的 hold 状态
was_hold_on = ishold(h_axes);

% --- 步骤 2: 准备并激活你的目标绘图环境 ---
% 将 MATLAB 的当前绘图目标强制切换到你传入的 h_axes
axes(h_axes);
% 强制打开 hold on，确保后续的绘图是叠加的
hold on;

% --- 步骤 3: 准备参数并调用原始的 dbfProc1D ---
% 我们需要确保 'drawEn' 参数为 1，这样 dbfProc1D 内部的绘图代码才会执行
% 检查 'drawEn' 是否已经存在于用户传入的参数中
drawEn_idx = find(strcmpi(params_cell, 'drawEn'));
if isempty(drawEn_idx)
    % 如果不存在，就在末尾添加 'drawEn', 1
    params_for_dbf = [params_cell, {'drawEn', 1}];
else
    % 如果已存在，就强制把它覆盖为 1
    params_for_dbf = params_cell;
    params_for_dbf{drawEn_idx + 1} = 1;
end

% 以绘图模式调用原始函数。所有绘图操作都会在 h_axes 上发生
[pwRA, pcRA] = dbfProc1D(radarData, params_for_dbf{:});

% --- 步骤 4: 恢复原始的绘图状态 ---
% dbfProc1D 内部的 'hold off' 会影响 h_axes 的状态。
% 我们需要根据它原始的状态来决定是否恢复 hold off。
if ~was_hold_on
    % 如果目标坐标轴原本不是 hold on 状态，我们就把它恢复原样
    hold(h_axes, 'off');
end

% 将 MATLAB 的当前绘图目标切换回最初的坐标轴
axes(original_axes);

end