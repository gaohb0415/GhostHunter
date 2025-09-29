function trajectory = extractTrajectory(trajectory, frame)
% 提取轨迹记录中在frame帧之前(含)的部分
% 输入:
% 1. trajectory: 主宇宙轨迹记录
%  - .iPeople: 轨迹计数ID
%  - .name: 人员姓名ID
%  - .trajectory: 轨迹记录
%  - .frame: 帧记录
%  - .status: 轨迹状态
%  - .pcLast: 最后帧的点云
%  - .kalmanFilter: 卡尔曼滤波器
%  - .statusAge: 状态年龄
% 2. frame: 目标帧
% 输出: 
% trajectory: 同输入, 提取后的轨迹记录
% 作者: 刘涵凯
% 更新: 2023-3-15

%% 若轨迹起点在目标帧之后, 则删除此轨迹
iDel = [];
for iTraj = 1 : structLength(trajectory, 'iPeople')
    if trajectory(iTraj).frame(1) > frame
        iDel = [iDel, iTraj];
    end
end
trajectory = structRowDelete(trajectory, iDel);

%% 若轨迹重点在目标帧之后, 则对后段进行删除
for iTraj = 1 : structLength(trajectory, 'iPeople')
    if trajectory(iTraj).frame(end) > frame
        iRetain = find(trajectory(iTraj).frame <= frame);
        trajectory(iTraj).trajectory = trajectory(iTraj).trajectory(iRetain, :);
        trajectory(iTraj).frame = trajectory(iTraj).frame(iRetain);
        trajectory(iTraj).pcLast = [];
    end
end
