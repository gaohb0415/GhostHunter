clear; clc; close all;

%% 1. 参数设置 (保持不变)
T_period = 1e-3;       % 周期：1ms
Fs = 5e6;              % 采样率：5MHz
B = 125e3;             % 带宽：125kHz 

%% 2. 生成时间向量
N = round(T_period * Fs); 
t = (0:N-1) / Fs;

%% 3. 生成 Down-Chirp 信号 (反向)
% 关键点：公式里的指数加了负号 (-)
% Up-Chirp:   exp( j * pi * slope * t^2)
% Down-Chirp: exp(-j * pi * slope * t^2)
slope = B / T_period;
signal_down = exp(-1j * pi * slope * t.^2);

%% 4. 数据格式转换 (GNU Radio 格式)
% Interleaved Float32 (I0, Q0, I1, Q1...)
data_to_save = zeros(1, 2*N);
data_to_save(1:2:end) = real(signal_down); 
data_to_save(2:2:end) = imag(signal_down); 

%% 5. 保存为 .dat 文件
% 文件名加了 "down" 标记，防止弄混
filename = 'downchirp_1ms_125k_5M.dat';
fid = fopen(filename, 'wb');
if fid == -1
    error('无法打开文件写入');
end
fwrite(fid, data_to_save, 'float32');
fclose(fid);

fprintf('反向 Chirp 生成成功: %s\n', filename);

%% 6. 验证绘图 (针对 Down-Chirp 优化显示)
figure;

% --- 上图：时域波形 ---
subplot(2,1,1);
plot(t*1000, real(signal_down)); 
title('反向 Chirp 时域实部 (I路)');
xlabel('时间 (ms)'); ylabel('幅度');
grid on;
xlim([0 T_period*1000]);

% --- 下图：时频图 (显示负频率) ---
subplot(2,1,2);

% 计算短时傅里叶变换
[s, f, t_spec] = spectrogram(signal_down, 256, 128, 256, Fs, 'centered'); 
% 注意：加了 'centered' 参数，让 MATLAB 自动把 0Hz 放在中间

% 转换单位
t_ms = t_spec * 1000;      % 秒 -> ms
f_khz = f / 1e3;           % Hz -> kHz

% 绘图
imagesc(t_ms, f_khz, 20*log10(abs(s))); 
axis xy; 
colormap('jet'); 
colorbar;

xlabel('时间 (ms)');
ylabel('频率 (kHz)');
title('反向信号时频图 (注意频率是负的)');

% 关键优化：限制 Y 轴范围看负半轴
% 因为是 Down-Chirp，频率是从 0 降到 -125kHz
ylim([-200 50]);