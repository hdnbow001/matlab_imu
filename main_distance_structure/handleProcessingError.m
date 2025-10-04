function dataStore = handleProcessingError(serialObj, sysParams, dataStore, pointIndex)
    % 处理数据处理过程中的错误
    
    fprintf('点 %d 处理错误，正在尝试恢复数据流...\n', pointIndex);
    
    % 重新同步串口
    resyncSuccess = resyncSerialPort(serialObj, sysParams);
    if resyncSuccess
        fprintf('数据流恢复成功\n');
    else
        fprintf('数据流恢复失败\n');
    end
    
    % 用前一个有效数据填充当前点（避免数据中断）
    if pointIndex > 1
        dataStore.raw.gyroX(pointIndex) = dataStore.raw.gyroX(pointIndex-1);
        dataStore.raw.gyroY(pointIndex) = dataStore.raw.gyroY(pointIndex-1);
        dataStore.raw.gyroZ(pointIndex) = dataStore.raw.gyroZ(pointIndex-1);
        dataStore.raw.accelX(pointIndex) = dataStore.raw.accelX(pointIndex-1);
        dataStore.raw.accelY(pointIndex) = dataStore.raw.accelY(pointIndex-1);
        dataStore.raw.accelZ(pointIndex) = dataStore.raw.accelZ(pointIndex-1);
        
        % 对于补偿后的数据也进行相同处理
        dataStore.compensated.gyroX(pointIndex) = dataStore.compensated.gyroX(pointIndex-1);
        dataStore.compensated.gyroY(pointIndex) = dataStore.compensated.gyroY(pointIndex-1);
        dataStore.compensated.gyroZ(pointIndex) = dataStore.compensated.gyroZ(pointIndex-1);
    else
        % 如果是第一个点出错，用零值填充
        dataStore.raw.gyroX(pointIndex) = 0;
        dataStore.raw.gyroY(pointIndex) = 0;
        dataStore.raw.gyroZ(pointIndex) = 0;
        dataStore.raw.accelX(pointIndex) = 0;
        dataStore.raw.accelY(pointIndex) = 0;
        dataStore.raw.accelZ(pointIndex) = 0;
        dataStore.compensated.gyroX(pointIndex) = 0;
        dataStore.compensated.gyroY(pointIndex) = 0;
        dataStore.compensated.gyroZ(pointIndex) = 0;
    end
end