function dataStore = calculateSensorAttitude(dataStore, imuParams, sysParams, pointIndex)
    % 计算传感器姿态角度（使用互补滤波）- 使用现有的calculateAttitude函数
    
    % 从加速度计计算姿态 - 使用现有的calculateAttitude函数
    [accelPitch, accelRoll] = calculateAttitude(...
        double(dataStore.raw.accelX(pointIndex)), ...
        double(dataStore.raw.accelY(pointIndex)), ...
        double(dataStore.raw.accelZ(pointIndex)), ...
        imuParams.accelRange);
    
    if pointIndex == 1
        % 初始化融合角度
        dataStore.fused.pitch = accelPitch;
        dataStore.fused.roll = accelRoll;
        dataStore.fused.yaw = 0;
    else
        % 陀螺仪数据转换为度/秒
        gyroXDps = (dataStore.compensated.gyroX(pointIndex) / 32768) * imuParams.gyroRange;
        gyroYDps = (dataStore.compensated.gyroY(pointIndex) / 32768) * imuParams.gyroRange;
        gyroZDps = (dataStore.compensated.gyroZ(pointIndex) / 32768) * imuParams.gyroRange;
        
        % 陀螺仪积分更新角度
        dataStore.fused.pitch = dataStore.fused.pitch + gyroXDps * sysParams.dt;
        dataStore.fused.roll = dataStore.fused.roll + gyroYDps * sysParams.dt;
        dataStore.fused.yaw = dataStore.fused.yaw + gyroZDps * sysParams.dt;
        
        % 互补滤波融合加速度计角度
        dataStore.fused.pitch = imuParams.complementaryFilterAlpha * dataStore.fused.pitch + ...
            (1 - imuParams.complementaryFilterAlpha) * accelPitch;
        dataStore.fused.roll = imuParams.complementaryFilterAlpha * dataStore.fused.roll + ...
            (1 - imuParams.complementaryFilterAlpha) * accelRoll;
    end
    
    % 存储最终角度
    dataStore.attitude.pitch(pointIndex) = dataStore.fused.pitch;
    dataStore.attitude.roll(pointIndex) = dataStore.fused.roll;
    dataStore.attitude.yaw(pointIndex) = dataStore.fused.yaw;
end