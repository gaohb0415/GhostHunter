function padTrajectory
% 轨迹补全, 将轨迹存在帧扩展至全iFrmLoad
% 作者: 刘涵凯
% 更新: 2024-6-22

%% 参数对象及全局变量
p = trackParamShare.param;
global iFrm trajectory

%% 补全
for iTraj = 1 : structLength(trajectory(iFrm).track, 'iPeople')
    trajOld = trajectory(iFrm).track(iTraj).trajectory;
    frameOld = trajectory(iFrm).track(iTraj).frame;
    trajNew = zeros(p.nFrmLoad, 2);
    % 轨迹出现之前与消失之后的帧, 赋予其出现或消失时的帧
    trajNew(1 : frameOld(1), :) = repmat(trajectory(iFrm).track(iTraj).trajectory(1, :), frameOld(1), 1);
    trajNew(frameOld(end) : end, :) = repmat(trajectory(iFrm).track(iTraj).trajectory(end, :), p.iFrmLoad(end) - frameOld(end) + 1, 1);
    % "内部帧"作插值处理
    idxInner = (frameOld(1) + 1 : frameOld(end) - 1)';
    trajNew(idxInner, :) = [interp1(frameOld, trajOld(:, 1), idxInner, 'spline'), interp1(frameOld, trajOld(:, 2), idxInner, 'spline')];
    trajectory(iFrm).track(iTraj).trajectory = trajNew;
    trajectory(iFrm).track(iTraj).frame = p.iFrmLoad;
end
