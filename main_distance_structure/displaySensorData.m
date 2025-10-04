function displaySensorData(pointIndex, hexStrings, sensorData, imuParams, dataStore)
    % 显示传感器数据到命令行
    
    try
        % 转换为物理单位
        gyroXDps = (double(sensorData.gyroX) / 32768) * imuParams.gyroRange;
        gyroYDps = (double(sensorData.gyroY) / 32768) * imuParams.gyroRange;
        gyroZDps = (double(sensorData.gyroZ) / 32768) * imuParams.gyroRange;
        accelXG = (double(sensorData.accelX) / 32768) * imuParams.accelRange;
        accelYG = (double(sensorData.accelY) / 32768) * imuParams.accelRange;
        accelZG = (double(sensorData.accelZ) / 32768) * imuParams.accelRange;
        
        fprintf('%d\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\n', ...
            pointIndex, ...
            hexStrings.gyroX, gyroXDps, ...
            hexStrings.gyroY, gyroYDps, ...
            hexStrings.gyroZ, gyroZDps, ...
            hexStrings.accelX, accelXG, ...
            hexStrings.accelY, accelYG, ...
            hexStrings.accelZ, accelZG);
        
        % 显示动态零偏估计
        if pointIndex > 1
            fprintf('动态零偏: X=%.2f, Y=%.2f, Z=%.2f\n', ...
                dataStore.bias.dynamicX, dataStore.bias.dynamicY, dataStore.bias.dynamicZ);
        end
        
    catch ME
        fprintf('数据显示错误 (点 %d): %s\n', pointIndex, ME.message);
    end
end