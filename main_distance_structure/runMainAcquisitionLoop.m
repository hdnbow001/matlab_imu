function dataStore = runMainAcquisitionLoop(serialObj, sysParams, imuParams, dataStore, gyroBiases, guiHandles)
    % 主数据采集和处理循环
    
    fprintf('开始主数据采集...\n');
    printDataHeader(); % 打印数据表头
    
    for pointIndex = 1:sysParams.maxPoints
        try
            % 1. 读取并解析传感器数据帧
            [sensorData, hexStrings, syncValid] = readAndParseDataFrame(serialObj, sysParams);
            
            % 2. 存储原始数据
            dataStore = storeRawSensorData(sensorData, dataStore, pointIndex);
            
            % 3. 应用零偏补偿（使用现有的gyroBiasCompensation函数）
            dataStore = applyBiasCompensation(sensorData, dataStore, gyroBiases, imuParams, pointIndex);
            
            % 4. 计算姿态角度（使用现有的calculateAttitude函数）
            dataStore = calculateSensorAttitude(dataStore, imuParams, sysParams, pointIndex);
            
            % 5. 更新位移窗口管理
            dataStore = updateDisplacementWindow(dataStore, imuParams, pointIndex);
            
            % 6. 计算位移（使用现有的calculateDisplacement函数）
            dataStore = calculateCurrentDisplacement(dataStore, imuParams, sysParams, pointIndex);
            
            % 7. 显示输出和更新图形 - 确保传递所有必要参数
            displayAndUpdateGUI(dataStore, guiHandles, sensorData, hexStrings, ...
                imuParams, sysParams, pointIndex, syncValid);
            
        catch ME
            % 错误处理
            fprintf('点 %d 处理错误: %s\n', pointIndex, ME.message);
            dataStore = handleProcessingError(serialObj, sysParams, dataStore, pointIndex);
        end
    end
    
    fprintf('\n数据采集完成，共处理 %d 个数据点\n', sysParams.maxPoints);
end