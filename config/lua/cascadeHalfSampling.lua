----------------------------------------User Constants--------------------------------------------
-- 级联雷达系统控制
-- 配置并启动一个由主控（Master）和从属（Slave）雷达设备组成的级联系统
-- 让它们同步开始工作并采集数据

-- 启动设备1、4，禁用设备2、3
RadarDevice = {1, 0, 0, 1} -- {dev1, dev4}, 1: Enable, 0: Disable

-- Device map of all the devices to be enabled by TDA
-- 1 - master ; 2- slave1 ; 4 - slave2 ; 8 - slave3
-- 位掩码：1001
deviceMapOverall = RadarDevice[1] + (RadarDevice[4] * 8)

-- Start TDACaptureCard 
-- 命令 TDA 数据采集卡为多个雷达设备准备并开始数据录制会话
-- 进入准备状态，并非开始采集数据
-- adc_data：数模转换器数据
if (0 == ar1.TDACaptureCard_StartRecord_mult(deviceMapOverall, 0, 0, adc_data, 0)) then
    WriteToLog("TDA ARM Successful\n", "green")
else
    WriteToLog("TDA ARM Failed\n", "red")
    return -5
end

-- Start Frame
-- 启动从设备slave
-- Slave3
-- slave必须先行准备好，启动内部的帧循环逻辑。但是不会立刻发生雷达信号，而是等待来自master的同步脉冲信号
if (0 == ar1.StartFrame_mult(8)) then
    WriteToLog("Slave3 : Start Frame Successful\n", "green")
else
    WriteToLog("Slave3 : Start Frame Failed\n", "red")
    return -5
end
-- Master
-- 启动主设备master
-- 运行这条指令的时候，master芯片不仅启动自己的帧循环，更重要的是，它会立刻生成并且发出那个同步信号给所有的slave设备
if (0 == ar1.StartFrame_mult(1)) then
    WriteToLog("Master : Start Frame Successful\n", "green")
else
    WriteToLog("Master : Start Frame Failed\n", "red")
    return -5
end

-- 在接收到同步信号的那一刻，Slave3 和 Master 在完全相同的时间点，一起开始发射雷达波并采集回波数据

--[[
-- Transfer Files Using WinSCP
WriteToLog("Starting Transfer files using WinSCP..\n", "blue")
if (0 == ar1.TransferFilesUsingWinSCP_mult(1)) then
    WriteToLog("Transferred files! COMPLETE!\n", "green")
else
    WriteToLog("Transferring files FAILED!\n", "red")
    return -5
end
]]

--ar1.StartMatlabPostProc("D:\\Softwares\\ti\\mmwave_studio_03_00_00_14\\mmWaveStudio\\PostProc\\adc_data\\")
