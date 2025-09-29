function mmFilterDeleteConfig
% 删除mmFilter的相关参数
% 作者: 刘涵凯
% 更新: 2022-12-1

delete('.\postProc\mmFilter\chirpDisorgOrder.mat', '.\postProc\mmFilter\phFlctnInterFrm.mat',...
    '.\postProc\mmFilter\pPosLockAng.mat', '.\postProc\mmFilter\pPosLockRg.mat', ...
    '.\postProc\mmFilter\spInfoErasureConfig.mat', '.\postProc\mmFilter\velDisorgOrder.mat')
% delete('.\postProc\mmFilter\pPosLockAng.mat', '.\postProc\mmFilter\pPosLockRg.mat', ...
%     '.\postProc\mmFilter\spInfoErasureConfig.mat')