% 雷达信号仿真-行走-SISO
% 作者: 刘涵凯
% 更新: 2024-3-30

close all; clear

%% 参数对象
config2243; % 6T8R的config并不会影响本仿真的1T1R
p = simParamInit('siso');
load('mesh0.mat', 'faces')

%% 运动模拟
iTraj = 50; % 轨迹编号
load([p.handleTrajectory]) % 载入轨迹
load('G:\radarData\simulation\md\radarData\231226\iSmpl.mat') % 载入适合的SMPL数据编号
[vert, joint, vertDif, jointDif] = bodyMotion(traj(iTraj), iSmpl = iSmpl(iTraj));

%% 计算三角面质心
ct = zeros([size(faces), p.nFrm + 1]); % 初始化面质心
for iFrm = 1 : p.nFrm + 1
    face = permute(reshape(vert(faces(:), :, iFrm), [13776, 3, 3]), [1, 3, 2]); % 面顶点坐标, [面索引, 坐标, 顶点索引]
    ct(:, :, iFrm) = mean(face, 3);
end

%% 下一帧的三角面质心(使用修正后的vertDif)
ctNext = zeros([size(faces), p.nFrm]); % 初始化面质心
for iFrm = 1 : p.nFrm
    face = permute(reshape(vert(faces(:), :, iFrm) + vertDif(faces(:), :, iFrm), [13776, 3, 3]), [1, 3, 2]); % 面顶点坐标, [面索引, 坐标, 顶点索引]
    ctNext(:, :, iFrm) = mean(face, 3);
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

    % 实时人体模型绘图
    % if exist('h', 'var'); delete(h); else figure; end
    % h = plot3(ct(faceIdxDs, 1, iFrm), ct(faceIdxDs, 2, iFrm), ct(faceIdxDs, 3, iFrm), '.', 'markersize', 5, 'color', 'b');
    % h= trisurf(faces(faceIdx, :), vert(:, 1, iFrm), vert(:, 2, iFrm), vert(:, 3, iFrm), 'FaceColor', 'y'); 
    % axis equal; xlim([-3.2, 3.2]); ylim([1.2, 6.8]); zlim([0, 2]); grid on; drawnow;
end

%% 加入空采环境信号
hEmpty = 'G:\radarData\23.11.20\empty\48chn\10s\1chn\'; % 整理后的空采数据地址
load([hEmpty, num2str(randi(960)), '.mat']) % 注意empty1Chn没有天线维度
radarData = simSignal + permute(empty1Chn(:, :, 1 : p.nFrm), [1, 2, 4, 5, 3]);

%% 保存数据
% hSave = [p.handleSimSignal, '\siso\simSignal.mat'];
% save(hSave, 'radarData')

%% 微多普勒谱
% dataInfo = struct('handle', hSave, 'saveMode', 'allFrm');
% microDoppler(dataType = 'sim', dataInfo = dataInfo)
