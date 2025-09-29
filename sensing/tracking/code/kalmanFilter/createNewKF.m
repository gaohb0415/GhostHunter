% get KF default parameters
% 修改自https://github.com/Research-and-Project/mmWave_radar_tracking
function kalmanFilter = createNewKF(centroid, varargin)
% 输入: 
% 1. centroid: 初始坐标
% 2. varargin:
%     - motionType: 运动模式. 'ConstantVelocity'-匀速; 'ConstantAcceleration'-匀加速
% 输出: 
% kalmanFilter: kalmanFilter类对象

p = inputParser();
p.CaseSensitive = false;
p.addOptional('motionType', 'ConstantVelocity'); % ConstantVelocity or ConstantAcceleration
p.parse(varargin{:});
param.motionModel = p.Results.motionType;

if strcmp(param.motionModel, 'ConstantAcceleration')
    param.initialEstimateError  = 1E5 * ones(1, 3);
    param.motionNoise           = [0.5, 0.1, 0.02];
    param.measurementNoise      = 0.5;
elseif strcmp(param.motionModel, 'ConstantVelocity')
    param.initialEstimateError  = 1E5 * ones(1, 2);
    param.motionNoise           = [0.5, 0.1];
%     param.motionNoise           = [0.005, 0.001];
    param.measurementNoise      = 0.5;
else
    error(['No assigned motion type - ' param.motionModel])
end

kalmanFilter = configureKalmanFilter(param.motionModel, centroid, ...
    param.initialEstimateError, param.motionNoise, param.measurementNoise);
end
