function radarData = readBin(iFrm, iChirp, varargin)
% 读取雷达数据
% 输入:
% 1. iFrm: 0-提取所有帧; 单独数字-提取某帧; 向量-提取多帧
% 2. iChirp: 同理提取相应Chirp
% 3. varargin:
%     - staticRmvEn: 是否执行静态杂波滤除. 0; 1
%     - dataType: 数据类型. 'real'; 'sim'
%     - dataInfo: 仿真数据信息
%       * .handle: 地址
%       * .frmMode: 存储模式. 'allFrm'; '1Frm'
% 输出:
% radarData: 雷达数据矩阵, [ADC, Chirp, Rx, Tx, (Frame)]
% 作者: 刘涵凯
% 更新: 2024-3-29

%% 默认参数
p = inputParser();                  % 可选参数处理开关
p.CaseSensitive = false;
p.addOptional('staticRmvEn', 1);    % 添加一个可选参数staticRmvEn，默认开启静态杂波滤除
p.addOptional('dataType', 'real');
p.addOptional('dataInfo', []);
p.parse(varargin{:});
staticRmvEn = p.Results.staticRmvEn;
dataType = p.Results.dataType;
dataInfo = p.Results.dataInfo;

%% 数据载入

switch dataType
    case 'real' % 真实数据
        load('config.mat', 'device', 'arrayType', 'nFrm')
        if any(iFrm > nFrm)
            error(['超出帧数限制(', num2str(nFrm), ')']);
        end
        switch device
            case '2243'
                switch arrayType
                    case 'half'
                        radarData = readTDA2(iFrm, iChirp);
                        % radarData = readTDA2_1Frm(iFrm);
                    case '1T1R'
                        % radarData = readTDA2(iFrm, iChirp);
                        radarData = readTDA2_1T1R_1Frm(iFrm);
                end
        end
    case 'sim' % 仿真数据
        switch dataInfo.saveMode
            case 'allFrm'
                load(dataInfo.handle)
                radarData = radarData(:, :, :, :, iFrm);
            case '1Frm’'
                load([dataInfo.handle, num2str(iFrm), '.mat'])
        end
        % 提取相应chirp
        if iChirp == 0; iChirp = 1 : size(radarData, 2); end
        radarData = radarData(:, iChirp, :, :, :);
end

%% 静态杂波滤除
if staticRmvEn; radarData = radarData - mean(radarData, 2); end
