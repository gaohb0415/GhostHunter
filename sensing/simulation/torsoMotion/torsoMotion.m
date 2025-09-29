function vert = torsoMotion(vert, joint, respDist)
% 模拟身体随呼吸进行张缩
% 输入:
% 1. vert: 网格顶点坐标
% 2. joint: 关节点坐标
% 3. respDist: 呼吸曲线, m, 若为[]则自动生成呼吸曲线
% 输出:
% vert: 加入张缩后的网格顶点坐标
% 作者: 刘涵凯
% 更新: 2023-6-28

%% 参数对象
p = simParamShare.param;

%% 胸腔位移修正
respDist = respDist * p.distAmplCorrect;

%% 载入数据
load('smplSeg.mat', 'segContainVert') % SMPL部位-顶点包含关系

%% 生成呼吸曲线
if isempty(respDist)
    load(p.handleResp)
    idx = randi(length(segLib.amplMean),1); % 随机选取一组曲线参数
    p.respDistMean = segLib.amplMean(idx);
    p.respDistStd = segLib.amplStd(idx);
    p.respCycleMean = segLib.timeMean(idx);
    p.respCycleStd = segLib.timeStd(idx);
    tic
    [respDist, ~] = respCoding(segLib); % 生成呼吸曲线
    toc
end

%% 设置张缩部位
torsoSeg = ["spine1", "spine"]; % 由上至下 "spine2", "spine1", "spine", "hips"

%% 顶点张缩
tic
for iFrm = 1 : p.nFrm + 1
    for iSeg = 1 : length(torsoSeg)
        % 部位顶点索
        idxVert = segContainVert.(torsoSeg(iSeg));
        % 关节点连线
        joint1 = joint(7, :, iFrm);
        joint2 = joint(10, :, iFrm);
        % 计算垂足
        vec1 = joint2 - joint1;
        vec2 = vert(idxVert, :, iFrm) - joint1;
        vec3 = dot(vec2, repmat(vec1, length(idxVert), 1), 2)  / dot(vec1, vec1) .* vec1; % vec2在vec1的投影
        projPoint = joint1 + vec3;
        % 加入波动
        vec4 = vert(idxVert, :, iFrm) - projPoint; % 垂线方向
        len = vecnorm(vec4, 2, 2);
        vert(idxVert, :, iFrm) = projPoint + vec4 .* (1 - respDist(iFrm) ./ len);
    end
end
