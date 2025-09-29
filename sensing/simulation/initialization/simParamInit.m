function p = simParamInit(mode)
% 根据仿真类型进行参数配置
% 输入
% mode: 仿真类型, 'mimo'; 'siso'; 'resp'
% 作者: 刘涵凯
% 更新: 2024-3-30

%% 参数对象
p = simParamConfig;

%% 重新配置
switch mode
    case 'mimo'
        p.nChirp1Frm = 128;
        p.tChirp = 50e-6;
        p.tFrm = 50e-3;
        p.nFrm = 201;
        p.orderTx = 4 : 6;
        p.orderRx = 1 : 8;
        p.txAmpl = 100e6;
    case 'siso'
        p.nChirp1Frm = 128;
        p.tChirp = 300e-6;
        p.tFrm = 50e-3;
        p.nFrm = 205;
        p.orderTx = 1 : 1;
        p.orderRx = 1 : 1;
        p.txAmpl = 35e6;
    case 'resp'
        p.nChirp1Frm = 10;
        p.tChirp = 5000e-6;
        p.tFrm = 50e-3;
        p.nFrm = 1220;
        p.orderTx = 1 : 1;
        p.orderRx = 1 : 1;
        p.txAmpl = 35e6;
end

%% 重新配置天线
p.nTx = length(p.orderTx);                 % 启用的Tx数
p.nRx = length(p.orderRx);                % 启用的Rx数
p.nTransmit = p.nTx * p.nChirp1Frm; % 一帧中发射的chirp数
% 计算中心位置并做差分
center = mean([p.posArrayTx(p.orderTx, :); p.posArrayRx(p.orderRx, :)]);
p.posTx = p.posArrayTx(p.orderTx, :) - center;                  % Tx坐标
p.posRx = p.posArrayRx(p.orderRx, :) - center;                 % Rx坐标
