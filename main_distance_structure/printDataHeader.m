function printDataHeader()
    % 打印数据输出表头
    
    fprintf('采样点\tGYRO_X(HEX)\tGYRO_X(dps)\tGYRO_Y(HEX)\tGYRO_Y(dps)\tGYRO_Z(HEX)\tGYRO_Z(dps)\tACCEL_X(HEX)\tACCEL_X(G)\tACCEL_Y(HEX)\tACCEL_Y(G)\tACCEL_Z(HEX)\tACCEL_Z(G)\n');
    fprintf('------\t-----------\t-----------\t-----------\t-----------\t-----------\t-----------\t------------\t----------\t------------\t----------\t------------\t----------\n');
end

function displayAndUpdateGUI(dataStore, guiHandles, sensorData, hexStrings, imuParams, pointIndex, syncValid)
    % 数据显示和图形界面更新
    
    % 定期显示数据到命令行
    if mod(pointIndex, 10) == 0 || pointIndex == 1
        displaySensorData(pointIndex, hexStrings, sensorData, imuParams, dataStore);
    end
    
    % 更新位移显示 - 使用现有的updateDisplacementDisplay函数
    updateDisplacementDisplay(guiHandles.displacement.plot, guiHandles.displacement.text, ...
        dataStore.displacement.x, dataStore.displacement.y, dataStore.displacement.z, ...
        pointIndex, guiHandles.displacement.axes, dataStore.window.currentStart, dataStore.window.count);
    
    % 定期更新姿态显示（性能优化）- 使用现有的updateAttitudeDisplay函数
    if mod(pointIndex, 10) == 0 || pointIndex == 1
        updateAttitudeDisplay(guiHandles.attitude.quiver, guiHandles.attitude.text, guiHandles.attitude.sphere, ...
            double(dataStore.raw.accelX(pointIndex)), double(dataStore.raw.accelY(pointIndex)), double(dataStore.raw.accelZ(pointIndex)), ...
            dataStore.compensated.gyroX(pointIndex), dataStore.compensated.gyroY(pointIndex), dataStore.compensated.gyroZ(pointIndex), ...
            guiHandles.attitude.axes, imuParams.accelRange, imuParams.gyroRange, ...
            dataStore.attitude.pitch(pointIndex), dataStore.attitude.roll(pointIndex), dataStore.attitude.yaw(pointIndex));
    end
    
    % 更新调试数据显示（可选）- 使用现有的calculateDebugData函数
    if pointIndex > 1 && mod(pointIndex, 5) == 0
        updateDebugDisplay(dataStore, pointIndex, imuParams, sysParams);
    end
    
    % 限制图形更新频率
    drawnow limitrate;
    
    % 显示同步状态警告
    if ~syncValid
        fprintf('点 %d: 数据帧同步丢失\n', pointIndex);
    end
end

function displaySensorData(pointIndex, hexStrings, sensorData, imuParams, dataStore)
    % 显示传感器数据到命令行
    
    % 转换为物理单位
    gyroXDps = (sensorData.gyroX / 32768) * imuParams.gyroRange;
    gyroYDps = (sensorData.gyroY / 32768) * imuParams.gyroRange;
    gyroZDps = (sensorData.gyroZ / 32768) * imuParams.gyroRange;
    accelXG = (sensorData.accelX / 32768) * imuParams.accelRange;
    accelYG = (sensorData.accelY / 32768) * imuParams.accelRange;
    accelZG = (sensorData.accelZ / 32768) * imuParams.accelRange;
    
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
end

