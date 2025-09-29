function mcs = cadenceVelocityDiagram(mdMap)
% 由微多普勒谱计算步频
% 比较简陋, 需要时再细化
% 参考论文Toward Unobtrusive In-Home Gait Analysis Based on Radar Micro-Doppler Signatures
% 输入: 
% mdMap: 微多普勒谱
% 输出: 
% 1. cvd: 速度-步频谱
% 2. mcs: 平均步频谱
% 作者: 刘涵凯
% 更新: 2024-3-14

nFFT = size(mdMap, 2);
if mod(nFFT, 2) ~= 0; nFFT = nFFT + 1; end
cvd = abs(fft(mdMap, nFFT, 2));
cvd = cvd(:, 1 : nFFT / 2);
mcs = mean(cvd, 1);

% figure
% imagesc(cvd, 'CDataMapping', 'scaled');
% colormap(flipud(hot))
% set(gca, 'ColorScale', 'log')
% 
% figure
% plot(mcs)
