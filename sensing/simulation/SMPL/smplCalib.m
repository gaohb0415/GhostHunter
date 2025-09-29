% SMPL位移及朝向校准
% 作者: 刘涵凯
% 更新: 2024-3-28

clear; close all

%% 参数对象
p = simParamShare.param;

%% 朝向校准
for iSmpl = 1
    % 载入SMPL数据
    handleSMPL = [p.handleSMPL, 'walking\CMU\mesh_walking_CMU_', num2str(iSmpl), '.mat'];
    load(handleSMPL)

    % 提取一帧数据
    iFrm = 1;
    vert = squeeze(vertices(iFrm, :, :)); % 将顶点维度转换为[顶点索引, 坐标, 帧索引]
    disp = trans(iFrm, [1, 2]); % 将位移维度转换为[坐标, 帧索引]
    ori = rad2deg(orientation(iFrm, 2));

    % 消除原位移和旋转
    vert = coordTransform2D(vert, -ori, -disp, 'firstOperation', 'disp');

    % 坐标变换
    % transShift = [-1, 0.9];
    % oriShift = 70;
    vert = coordTransform2D(vert, oriShift, transShift);

    % 绘图以查看效果
    if exist('h', 'var'); delete(h); end
    h = plot(vert(:, 1), vert(:, 2), '.', MarkerSize = 10); hold on; axis equal; grid on
    plot(0, 0, '.', 'color', 'r', MarkerSize = 30)
    set(gca, 'LooseInset', get(gca, 'TightInset'))
    set(gcf, 'Units', 'centimeters', 'Position', [-15 -6 12 9])
    drawnow;
    
    % 保存参数
    % save(handleSMPL, '-append', 'oriShift', 'transShift');
end
