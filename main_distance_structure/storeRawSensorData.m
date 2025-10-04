function dataStore = storeRawSensorData(sensorData, dataStore, pointIndex)
    % 存储原始传感器数据到数据存储结构
    
    try
        % 存储陀螺仪原始数据
        dataStore.raw.gyroX(pointIndex) = sensorData.gyroX;
        dataStore.raw.gyroY(pointIndex) = sensorData.gyroY;
        dataStore.raw.gyroZ(pointIndex) = sensorData.gyroZ;
        
        % 存储加速度计原始数据
        dataStore.raw.accelX(pointIndex) = sensorData.accelX;
        dataStore.raw.accelY(pointIndex) = sensorData.accelY;
        dataStore.raw.accelZ(pointIndex) = sensorData.accelZ;
        
    catch ME
        fprintf('存储原始数据错误 (点 %d): %s\n', pointIndex, ME.message);
        % 如果出错，用零值填充
        dataStore.raw.gyroX(pointIndex) = 0;
        dataStore.raw.gyroY(pointIndex) = 0;
        dataStore.raw.gyroZ(pointIndex) = 0;
        dataStore.raw.accelX(pointIndex) = 0;
        dataStore.raw.accelY(pointIndex) = 0;
        dataStore.raw.accelZ(pointIndex) = 0;
    end
end