----------------------------------------User Constants 用户常量与设置--------------------------------------------
dev_list = {1, 2, 4, 8} -- Device map 定义系统中的设备ID
RadarDevice = {1, 0, 0, 1} -- {dev1, dev4}, 1: Enable, 0: Disable 设备1、4 已经开启
cascade_mode_list = {1, 2, 2, 2} -- 0: Single chip, 1: Master, 2: Slave dev1是主设备，2、3、4是从设备

-- F/W Download Path 雷达固件文件加载位置
metaImagePath = "E:\\mmwave_dfp\\mmwave_dfp_02_02_03_01\\firmware\\xwr22xx_metaImage.bin"

-- IP Address for the TDA2 Host Board
-- Change this accordingly for your setup
-- TDA2板处理器的IP地址，雷达将最终的数据发送给这个处理器
TDA_IPAddress = "192.168.33.180"

-- Device map of all the devices to be enabled by TDA
-- 1 - master ; 2- slave3
-- 位掩码 Bitmask，将RadarDevice数组转换成一个单一的数字，用数字的二进制表示法来表示那些设备需要同时操作
deviceMapOverall = RadarDevice[1] + (RadarDevice[4] * 8)
deviceMapSlaves = 8

------------------------------------------- Sensor Configuration 传感器核心参数 ------------------------------------------------
-- Profile configuration
-- 定义了雷达发射的单个无线电脉冲（称为 Chirp）的基本属性，
-- 比如它的频率范围、持续时间、强度等。这决定了雷达的最大探测距离和距离分辨率。
local profile_indx = 0
local start_freq = 77 -- 起始频率 GHz
local slope = 99 -- 频率斜率 MHz/us
local idle_time = 10 -- 脉冲空间时间 us
local adc_start_time = 3 -- us
local adc_samples = 256 -- 每次采样的点数 Number of samples per chirp
local sample_freq = 7040 -- 采样率 ksps
local ramp_end_time = 40 -- 脉冲持续时间 us
local rx_gain = 48 -- 接收增益 dB
local tx0OutPowerBackoffCode = 0
local tx1OutPowerBackoffCode = 0
local tx2OutPowerBackoffCode = 0
local tx0PhaseShifter = 0
local tx1PhaseShifter = 0
local tx2PhaseShifter = 0
local txStartTimeUSec = 0
local hpfCornerFreq1 = 0 -- 0: 175KHz, 1: 235KHz, 2: 350KHz, 3: 700KHz
local hpfCornerFreq2 = 0 -- 0: 350KHz, 1: 700KHz, 2: 1.4MHz, 3: 2.8MHz

-- Frame configuration    
-- 定义了雷达如何将这些单独的脉冲组合成一个有意义的“数据包”。
-- 一帧包含多个 Chirp。这决定了雷达的最大速度探测范围和速度分辨率，以及刷新率
local start_chirp_tx = 0
local end_chirp_tx = 5 -- lhk
local nchirp_loops = 128 -- 每帧包含的Chirp数量 Number of chirps per frame
local nframes_master = 210 -- 主设备要发送的总帧数 Number of Frames for Master
local nframes_slave = 210 -- Number of Frames for Slaves
local Inter_Frame_Interval = 50 -- 帧数间隔 ms
local trigger_delay = 0 -- us
local trig_list = {1, 2, 2, 2} -- 1: Software trigger, 2: Hardware trigger    

--[[
Function to configure the chirps specific to a device
-- Note: The syntax for this API is:
-- ar1.ChirpConfig_mult(RadarDeviceId, chirpStartIdx, chirpEndIdx, profileId, startFreqVar, 
--                      freqSlopeVar, idleTimeVar, adcStartTimeVar, tx0Enable, tx1Enable, tx2Enable)
]]

-- 用来配置每个设备上不同Chirp的细节
-- 这个函数根据传入的设备ID（i），为该设备配置一系列的Chirp
function Configure_Chirps(i)

    if (i == 1) then

        -- Chirp 0
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 1, 0, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 0 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 0 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 1
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 1, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 1 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 1 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 2
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 0, 0, 1)) then
            WriteToLog("Device " .. i .. " : Chirp 2 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 2 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 3
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 0 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 0 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 4
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 1 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 1 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 5
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 2 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 2 Configuration failed\n", "red")
            return -4
        end

    elseif (i == 4) then

        -- Chirp 0
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 0 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 0 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 1
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 1 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 1 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 2
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 2 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 2 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 3
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 1, 0, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 0 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 0 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 4
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 1, 0)) then
            WriteToLog("Device " .. i .. " : Chirp 1 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 1 Configuration failed\n", "red")
            return -4
        end

        -- Chirp 5
        if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 0, 0, 1)) then
            WriteToLog("Device " .. i .. " : Chirp 2 Configuration successful\n", "green")
        else
            WriteToLog("Device " .. i .. " : Chirp 2 Configuration failed\n", "red")
            return -4
        end

    end

end

------------------------------ API Configuration 主执行流程 ------------------------------------------------

-- 1. Connection to TDA.                连接处理主机
-- 2. Selecting Cascade/Single Chip.    选择级联模式
-- 3. Selecting 2-chip/4-chip           

WriteToLog("Setting up Studio for Cascade started..\n", "blue")

-- 使用 ar1 这个雷达遥控器，通过网络连接到 IP 地址为 192.168.33.180 的数据处理主板，
-- 连接的目标服务端口是 5001，并且明确告知这次连接将会操作雷达设备1和设备4
-- TDA IP 、端口 、 要操作的设备的位掩码
if (0 == ar1.ConnectTDA(TDA_IPAddress, 5001, deviceMapOverall)) then
    WriteToLog("ConnectTDA Successful\n", "green")
else
    WriteToLog("ConnectTDA Failed\n", "red")
    return -1
end

-- 通过 ar1 这个遥控器，向 TDA 主机发送一条指令
-- 正式声明：‘我们要启用多芯片级联模式进行工作
if (0 == ar1.selectCascadeMode(1)) then
    WriteToLog("selectCascadeMode Successful\n", "green")
else
    WriteToLog("selectCascadeMode Failed\n", "red")
    return -1
end

WriteToLog("Setting up Studio for Cascade ended..\n", "blue")

-- Master Initialization


-- SOP Mode Configuration

-- 通过 ar1 遥控器，向设备1（主设备）发送一条指令
-- 命令它立即进入 SOP 4 模式，也就是 固件刷写模式
-- 后续芯片的引导加载程序会准备好接收新固件，并且将其烧录到闪存中
if (0 == ar1.SOPControl_mult(1, 4)) then
    WriteToLog("Master : SOP Reset Successful\n", "green")
else
    WriteToLog("Master : SOP Reset Failed\n", "red")
    return -1
end

-- SPI Connect
-- 过 ar1 遥控器，向设备1（主设备）发送指令：请激活你的处理器并准备好SPI通信接口，
-- 在执行此操作时不要进行硬件复位，完成后请等待1秒钟以确保一切稳定

-- Device Map 这条指令的目标是设备1 、 不执行硬件复位 、 延迟脚本刷新时间 、 保留参数（4、5）
if (0 == ar1.PowerOn_mult(1, 0, 1000, 0, 0)) then
    WriteToLog("Master : SPI Connection Successful\n", "green")
else
    WriteToLog("Master : SPI Connection Failed\n", "red")
    return -1
end

-- Slave Initialization 从雷达配置

-- SOP Mode Configuration
-- id为8的从设备（slave3），命令它立刻进入SOP4固件刷写模式
if (0 == ar1.SOPControl_mult(8, 4)) then
    WriteToLog("Slave3 : SOP Reset Successful\n", "green")
else
    WriteToLog("Slave3 : SOP Reset Failed\n", "red")
    return -1
end

-- SPI Connect   
-- 将设备ID为8的从设备添加到我的控制列表中，并建立与它的SPI通信链路
if (0 == ar1.AddDevice(8)) then
    WriteToLog("Slave3 : SPI Connection Successful\n", "green")
else
    WriteToLog("Slave3 : SPI Connection Failed\n", "red")
    return -1
end

-- Firmware Download. (SOP 4 - MetaImage)
-- 读取路径为 metaImagePath 的固件文件，然后通过已经建立的SPI通信链路
-- 将这个固件同时广播并烧录到 deviceMapOverall 指定的所有雷达芯片（即主设备和所有从设备）中
if (0 == ar1.DownloadBssFwOvSPI_mult(deviceMapOverall, metaImagePath)) then
    WriteToLog("Slaves : FW Download Successful\n", "green")
else
    WriteToLog("Slaves : FW Download Failed\n", "red")
    return -1
end

-- RF Power Up 启动雷达射频硬件系统指令
-- 向所有已成功加载固件的雷达芯片（主设备和从设备）发送一条指令：
-- 现在，请给你们的射频部分通电，激活发射器和接收器硬件

if (0 == ar1.RfEnable_mult(deviceMapOverall)) then
    WriteToLog("Slaves : RF Power Up Successful\n", "green")
else
    WriteToLog("Slaves : RF Power Up Failed\n", "red")
    return -1
end

-- All devices together        

-- Channel & ADC Configuration
-- 通道 与 ADC的配置
-- 主master雷达的配置
if (0 == ar1.ChanNAdcConfig_mult(1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 0, 1)) then
    WriteToLog("Master : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Master : Channel & ADC Configuration Failed\n", "red")
    return -2
end
-- 从slave雷达的配置
if (0 == ar1.ChanNAdcConfig_mult(deviceMapSlaves, 1, 1, 1, 1, 1, 1, 1, 2, 1, 0, 2)) then
    WriteToLog("Slaves : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Slaves : Channel & ADC Configuration Failed\n", "red")
    return -2
end

-- Including this depends on the type of board being used.
-- LDO configuration
-- 请将你们内部为射频电路供电的LDO稳压器完全禁用（旁路）
-- 因为我们正在使用的这块电路板上设计了更高性能的外部电源来直接为射频电路供电

if (0 == ar1.RfLdoBypassConfig_mult(deviceMapOverall, 3)) then
    WriteToLog("LDO Bypass Successful\n", "green")
else
    WriteToLog("LDO Bypass failed\n", "red")
    return -2
end

-- Low Power Mode Configuration
-- 雷达不使用低功耗配置
-- 确保你们都处于常规的、全功率的工作模式下，禁用任何形式的低功耗或省电模式
if (0 == ar1.LPModConfig_mult(deviceMapOverall, 0, 0)) then
    WriteToLog("Low Power Mode Configuration Successful\n", "green")
else
    WriteToLog("Low Power Mode Configuration failed\n", "red")
    return -2
end

-- Miscellaneous Control Configuration
if (0 == ar1.SetMiscConfig_mult(deviceMapOverall, 1, 0, 0, 0)) then
    WriteToLog("Misc Control Configuration Successful\n", "green")
else
    WriteToLog("Misc Control Configuration failed\n", "red")
    return -2
end

-- Edit this API to enable/disable the boot time calibration. Enabled by default.
-- RF Init Calibration Configuration
-- 配置射频初始化校准
if (0 == ar1.RfInitCalibConfig_mult(deviceMapOverall, 1, 1, 1, 1, 1, 1, 1, 65537)) then
    WriteToLog("RF Init Calibration Successful\n", "green")
else
    WriteToLog("RF Init Calibration failed\n", "red")
    return -2
end

-- RF Init
-- 执行射频初始化
if (0 == ar1.RfInit_mult(deviceMapOverall)) then
    WriteToLog("RF Init Successful\n", "green")
else
    WriteToLog("RF Init failed\n", "red")
    return -2
end

---------------------------Data Configuration----------------------------------

-- Data path Configuration
-- 数据传输路径配置
if (0 == ar1.DataPathConfig_mult(deviceMapOverall, 0, 1, 0)) then
    WriteToLog("Data Path Configuration Successful\n", "green")
else
    WriteToLog("Data Path Configuration failed\n", "red")
    return -3
end

-- Clock Configuration
-- 数据时钟配置
-- 启动雷达用于高速传输数据的内部时钟，并且将其设定为标准的工作模式
-- 准备为接下来的数据传输提供同步节拍
if (0 == ar1.LvdsClkConfig_mult(deviceMapOverall, 1, 1)) then
    WriteToLog("Clock Configuration Successful\n", "green")
else
    WriteToLog("Clock Configuration failed\n", "red")
    return -3
end

-- CSI2 Configuration
-- 激活CSI2高速输出端口，并且按照预设的电路板设计，将内部时钟和数据精确地分配到指定的物理输出通道上
if (0 == ar1.CSI2LaneConfig_mult(deviceMapOverall, 1, 0, 2, 0, 4, 0, 5, 0, 3, 0, 0)) then
    WriteToLog("CSI2 Configuration Successful\n", "green")
else
    WriteToLog("CSI2 Configuration failed\n", "red")
    return -3
end

---------------------------Sensor Configuration-------------------------

-- Profile Configuration
-- 定义FMCW调频连续波中单个Chirp信号所有的物理属性
-- 这个函数调用了脚本最开头 Sensor Configuration 部分定义的许多变量，将它们正式应用到硬件上。
if (0 ==
    ar1.ProfileConfig_mult(deviceMapOverall, 0, start_freq, idle_time, adc_start_time, ramp_end_time, 0, 0, 0, 0, 0, 0,
        slope, 0, adc_samples, sample_freq, 0, 0, rx_gain)) then
    WriteToLog("Profile Configuration successful\n", "green")
else
    WriteToLog("Profile Configuration failed\n", "red")
    return -4
end

-- Chirp Configuration 
for i = 1, table.getn(RadarDevice) do
    if ((RadarDevice[1] == 1) and (RadarDevice[i] == 1)) then
        Configure_Chirps(i)
    end
end

-- Frame Configuration
-- 帧周期明确分工配置
-- Master

if (0 == ar1.FrameConfig_mult(1, start_chirp_tx, end_chirp_tx, nframes_master, nchirp_loops, Inter_Frame_Interval, 0, 1)) then
    WriteToLog("Master : Frame Configuration successful\n", "green")
else
    WriteToLog("Master : Frame Configuration failed\n", "red")
end
-- Slaves 
if (0 ==
    ar1.FrameConfig_mult(deviceMapSlaves, start_chirp_tx, end_chirp_tx, nframes_slave, nchirp_loops,
        Inter_Frame_Interval, 0, 2)) then
    WriteToLog("Slaves : Frame Configuration successful\n", "green")
else
    WriteToLog("Slaves : Frame Configuration failed\n", "red")
end
