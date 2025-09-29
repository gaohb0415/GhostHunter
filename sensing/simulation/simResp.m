% 雷达信号仿真-呼吸
% 作者: 刘涵凯
% 更新: 2024-3-29

close all; clear

%% 参数对象
config22431T1R;
p = simParamInit('resp');
load('mesh0.mat', 'faces')

%% SMPL生成
[verts, joint] = smplForResp;

%% 加入呼吸
vert = torsoMotion(verts, joint, []);

%% 计算三角面质心
ct = zeros([size(faces), p.nFrm + 1]); % 初始化面质心
for iFrm = 1 : p.nFrm + 1
    face = permute(reshape(vert(faces(:), :, iFrm), [13776, 3, 3]), [1, 3, 2]); % 面顶点坐标, [面索引, 坐标, 顶点索引]
    ct(:, :, iFrm) = mean(face, 3);
end

%% 反射信号模拟
simSignal = zeros(p.nAdc1Chirp, p.nChirp1Frm, p.nRx, p.nTx, p.nFrm);
for iFrm = 1 : p.nFrm
    disp(['信号仿真: 第', num2str(iFrm), '帧'])

    %% 反射点生成
    [faceIdx, rcs] = hiddenRemoval(vert(:, :, iFrm), p.posRadar, faces, ct(:, :, iFrm)); % 遮挡点/面移除
    [faceIdxDs, rcsDs] = faceDownsample(faceIdx, rcs, 'normal'); % 降采样
    ptNow = ct(faceIdxDs, :, iFrm); ptNext = ct(faceIdxDs, :, iFrm + 1); % 本帧及下一帧质心
    ampl = amplLos(ptNow, rcsDs);

    %% 帧间坐标插值
    ptInterp = ptChirpInterp(ptNow, ptNext);

    %% 信号合成
    simSignal(:, :, :, :, iFrm) = synthFromPt(ptInterp, ampl);

    %% 实时人体模型绘图
    % if exist('h', 'var'); delete(h); else figure; end
    % h = plot3(vert(:, 1, iFrm), vert(:, 2, iFrm), vert(:, 3, iFrm), '.', 'markersize', 0.05, 'color', 'b'); axis equal; drawnow;
    % h = trisurf(faces(faceIdx, :), vert(:, 1, iFrm), vert(:, 2, iFrm), vert(:, 3, iFrm), 'FaceColor', 'y'); axis equal; drawnow;
end

%% 加入空采环境信号
% 注意, 由于独特的设计, 这里与真实数据参数对应
simSignal = reshape(simSignal, [256, 200, 1, 1, 61]); 
simSignal(:, end, :, :, :) = [];  % 删除每帧的最后一个chirp
config22431T1REmpty;
radarData = simSignal + (readTDA2(1 : 61, 0));

%% 保存数据
% hSave = [p.handleSimSignal, '\resp\simSignal.mat'];
% save(hSave, 'radarData')

%% 呼吸监测
% dataInfo = struct('handle', hSave, 'saveMode', 'allFrm');
% respMonitor(dataType = 'sim', dataInfo = dataInfo)
