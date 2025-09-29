function [vert, joint] = smplForResp(varargin)
% 用于呼吸仿真的SMPL生成
% 输入: 
% varargin:
% - pos: 二维坐标, m
% - ori: 身体朝向, °
% 输出:
% 1. vert: 顶点坐标, [顶点索引, 坐标, 帧索引]
% 2. joint: 关节点坐标, [关节索引, 坐标, 帧索引]
% 作者: 刘涵凯
% 更新: 2024-3-28

%% 默认参数
param = inputParser();
param.CaseSensitive = false;
param.addOptional('pos', [0.22, 1.5]);
param.addOptional('ori', -90);
param.parse(varargin{:});
pos = param.Results.pos;
ori = param.Results.ori;

%% 参数对象
p = simParamShare.param;

%% 载入SMPL数据
load('G:\radarData\simulation\SMPL\sitting\mesh75-18-0.mat')
load('smplSeg.mat', 'vertInSeg') % 面-部位从属关系

%% 维度转换
trans = permute(trans, [2, 1]); % 将位移维度转换为[坐标, 帧索引]
vertices = permute(vertices, [2, 3, 1]); % 将顶点维度转换为[顶点索引, 坐标, 帧索引]
joints = permute(joints, [2, 3, 1]); % 将关节点维度转换为[关节索引, 坐标, 帧索引]

%% 加入身高
vertices(:, 3, :) = vertices(:, 3, :) + permute(repmat(trans(3, :), [1, 1, 6890]), [3, 1, 2]);
joints(:, 3, :) = joints(:, 3, :) + permute(repmat(trans(3, :), [1, 1, 52]), [3, 1, 2]);

%% 网格平滑
smoothWin = ceil(30 * framerate);
joints = smoothdata(joints, 3, 'movmean', smoothWin);
vertices = smoothdata(vertices, 3, 'movmean', smoothWin);

%% 数据提取
vert = vertices(:, :, 1 : (framerate / p.fFrm) : end);
joint = joints(:, :, 1 : (framerate / p.fFrm) : end);

%% 提取雷达帧数范围内的SMPL数据
vert = vert(:, :, 1 : p.nFrm + 1);
joint = joint(:, :, 1 : p.nFrm + 1);

%% 位移与旋转
for iFrm = 1 : p.nFrm + 1
    vert(:, :, iFrm) = coordTransform2D(vert(:, :, iFrm), ori, pos);
    joint(:, :, iFrm) = coordTransform2D(joint(:, :, iFrm), ori, pos);
end
