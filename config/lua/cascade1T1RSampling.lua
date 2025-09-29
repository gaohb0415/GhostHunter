----------------------------------------User Constants--------------------------------------------
RadarDevice = {1, 0, 0, 0} -- {dev1}, 1: Enable, 0: Disable

-- Device map of all the devices to be enabled by TDA
-- 1 - master ; 2- slave1 ; 4 - slave2 ; 8 - slave3
deviceMapOverall = RadarDevice[1]

-- Start TDACaptureCard 
if (0 == ar1.TDACaptureCard_StartRecord_mult(deviceMapOverall, 0, 0, adc_data, 0)) then
    WriteToLog("TDA ARM Successful\n", "green")
else
    WriteToLog("TDA ARM Failed\n", "red")
    return -5
end

-- Start Frame
-- Master
if (0 == ar1.StartFrame_mult(1)) then
    WriteToLog("Master : Start Frame Successful\n", "green")
else
    WriteToLog("Master : Start Frame Failed\n", "red")
    return -5
end

-- -- Transfer Files Using WinSCP
-- WriteToLog("Starting Transfer files using WinSCP..\n", "blue")
-- if (0 == ar1.TransferFilesUsingWinSCP_mult(1)) then
--     WriteToLog("Transferred files! COMPLETE!\n", "green")
-- else
--     WriteToLog("Transferring files FAILED!\n", "red")
--     return -5
-- end


