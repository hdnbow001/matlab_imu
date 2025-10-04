function gyroBiases = performGyroCalibration(serialObj, sysParams, imuParams)
    % 执行陀螺仪零偏校准（传感器需保持静止）
    
    fprintf('开始陀螺仪零偏校准，请保持传感器静止...\n');
    
    % 初始化校准数据存储
    calibData.gyroX = zeros(1, imuParams.calibrationSamples);
    calibData.gyroY = zeros(1, imuParams.calibrationSamples);
    calibData.gyroZ = zeros(1, imuParams.calibrationSamples);
    
    for calibIndex = 1:imuParams.calibrationSamples
        try
            % 读取并解析一帧数据
            [sensorData, hexStrings, syncValid] = readAndParseDataFrame(serialObj, sysParams);
            
            % 检查 syncValid 的类型并正确处理
            if isstruct(syncValid)
                % 如果 syncValid 是结构体，假设同步成功
                syncSuccess = true;
            else
                % 如果 syncValid 是逻辑值，直接使用
                syncSuccess = logical(syncValid);
            end
            
            if syncSuccess
                % 存储原始陀螺仪数据
                calibData.gyroX(calibIndex) = sensorData.gyroX;
                calibData.gyroY(calibIndex) = sensorData.gyroY;
                calibData.gyroZ(calibIndex) = sensorData.gyroZ;
            else
                fprintf('校准点 %d: 同步丢失\n', calibIndex);
                % 用前一个有效值填充（如果有的话）
                if calibIndex > 1
                    calibData.gyroX(calibIndex) = calibData.gyroX(calibIndex-1);
                    calibData.gyroY(calibIndex) = calibData.gyroY(calibIndex-1);
                    calibData.gyroZ(calibIndex) = calibData.gyroZ(calibIndex-1);
                end
            end
            
            % 显示校准进度
            if mod(calibIndex, 50) == 0
                fprintf('校准进度: %d/%d\n', calibIndex, imuParams.calibrationSamples);
            end
            
        catch ME
            fprintf('校准点 %d 读取失败: %s\n', calibIndex, ME.message);
            % 用前一个有效值填充（如果有的话）
            if calibIndex > 1
                calibData.gyroX(calibIndex) = calibData.gyroX(calibIndex-1);
                calibData.gyroY(calibIndex) = calibData.gyroY(calibIndex-1);
                calibData.gyroZ(calibIndex) = calibData.gyroZ(calibIndex-1);
            end
        end
    end
    
    % 计算零偏值（平均值）
    gyroBiases.x = mean(calibData.gyroX);
    gyroBiases.y = mean(calibData.gyroY);
    gyroBiases.z = mean(calibData.gyroZ);
    
    fprintf('零偏校准完成:\n');
    fprintf('  GYRO X轴零偏: %.2f LSB\n', gyroBiases.x);
    fprintf('  GYRO Y轴零偏: %.2f LSB\n', gyroBiases.y);
    fprintf('  GYRO Z轴零偏: %.2f LSB\n', gyroBiases.z);
end