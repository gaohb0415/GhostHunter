clear; clc; close all;

%% 1. 参数设置
T_period = 1e-3;       % 周期：1ms
Fs = 5e6;              % 采样率：5MHz
B = 125e3;             % 带宽：125kHz (可在此处自由修改)

%% 2. 生成时间向量
% 采样点数 N = 周期 * 采样率
N = round(T_period * Fs); 
t = (0:N-1) / Fs;

%% 3. 生成 Chirp 信号 (复数信号)
% 公式：s(t) = exp(j * pi * (B/T) * t^2)
% 这是一个从 0 Hz 扫频到 B Hz 的 Up-Chirp
slope = B / T_period;
signal_complex = exp(1j * pi * slope * t.^2);

% 如果你需要从 -B/2 扫到 B/2 (居中)，使用下面这行代替上面：
% signal_complex = exp(1j * pi * slope * (t - T_period/2).^2);

%% 4. 数据格式转换 (适应 GNU Radio/USRP)
% GNU Radio 需要 interleaved float32 格式: I0, Q0, I1, Q1, ...
% 将实部(I)和虚部(Q)交错排列
data_to_save = zeros(1, 2*N);
data_to_save(1:2:end) = real(signal_complex); % 奇数位存实部 (I)
data_to_save(2:2:end) = imag(signal_complex); % 偶数位存虚部 (Q)

%% 5. 保存为 .dat 文件
filename = 'chirp_1ms_125k_5M.dat';
fid = fopen(filename, 'wb'); % 'wb' 表示以二进制写入
if fid == -1
    error('无法打开文件写入');
end
fwrite(fid, data_to_save, 'float32'); % 写入 32位浮点数
fclose(fid);

fprintf('文件生成成功: %s\n', filename);
fprintf('样本总数: %d (I+Q)\n', N);
fprintf('文件大小应为: %d bytes\n', N * 2 * 4);

%% 6. (可选) 验证绘图 - 优化显示版 (横坐标改为 ms)
figure;

% --- 上图：时域波形 (I路) ---
subplot(2,1,1);
plot(t*1000, real(signal_complex)); 
title('Chirp 信号实部 (I路)');
xlabel('时间 (ms)');       % 强制单位为 ms
ylabel('幅度');
grid on;
xlim([0 T_period*1000]);   % 锁定 X 轴范围为 0 到 1ms

% --- 下图：时频图 (手动绘制，单位改为 ms) ---
subplot(2,1,2);

% 1. 获取频谱数据，而不是直接让 MATLAB 画图
%    s: 频谱矩阵, f: 频率向量, t_spec: 时间向量
[s, f, t_spec] = spectrogram(signal_complex, 256, 128, 256, Fs);

% 2. 手动转换单位
t_ms = t_spec * 1000;      % 时间：秒 -> 毫秒
f_khz = f / 1e3;           % 频率：Hz -> kHz (为了看清 125k，用 kHz 更直观)

% 3. 使用 imagesc 绘图
%    20*log10(...) 是为了把能量转换为 dB，看起来对比度更高
imagesc(t_ms, f_khz, 20*log10(abs(s))); 

axis xy;           % 修正 Y 轴方向 (让低频在下方)
colormap('jet');   % 换个颜色，看着更清晰
colorbar;          % 显示颜色条

xlabel('时间 (ms)');
ylabel('频率 (kHz)');
title('信号时频图 (验证带宽)');

% 4. 关键优化：限制 Y 轴范围
%    因为你的带宽只有 125kHz，而采样率高达 5000kHz (5MHz)
%    如果不限制，信号就会被压在最底下一条线。
%    这里我们只显示 0 到 200kHz，让你清楚看到 Chirp 的斜坡。
ylim([0 200]);