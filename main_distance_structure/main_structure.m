function main()
    % 主函数 - IMU传感器数据采集与处理
    %获取需要添加的根目录（这里以当前工作目录为例）
    targetFolder = pwd; 
    %生成包含所有子目录的路径字符串
    pathString = genpath(targetFolder);
    %将这些路径添加到MA
    addpath(pathString);
    
    % 1. 系统参数初始化
    [sysParams, imuParams] = initializeSystemParameters();
    
    % 2. 串口初始化与连接
    serialObj = initializeSerialPort(sysParams);
    if isempty(serialObj)
        return;
    end
    
    % 3. 数据存储结构初始化
    dataStore = initializeDataStorage(sysParams.maxPoints);
    
    % 4. 图形显示界面初始化
    guiHandles = initializeGUI();
    
    % 5. 陀螺仪零偏校准阶段
    gyroBiases = performGyroCalibration(serialObj, sysParams, imuParams);
    
    % 6. 主数据采集处理循环
    dataStore = runMainAcquisitionLoop(serialObj, sysParams, imuParams, dataStore, gyroBiases, guiHandles);
    
    % 7. 系统清理与数据保存
    cleanupSystem(serialObj, dataStore);
end

