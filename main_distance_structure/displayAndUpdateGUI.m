function displayAndUpdateGUI(dataStore, guiHandles, sensorData, hexStrings, imuParams, sysParams, pointIndex, syncValid)
    % 数据显示和图形界面更新函数
    
    try
        % 定期显示数据到命令行
        if mod(pointIndex, 10) == 0 || pointIndex == 1
            displaySensorData(pointIndex, hexStrings, sensorData, imuParams, dataStore);
        end
        
        % 更新位移显示
        if isfield(guiHandles, 'displacement') && ~isempty(guiHandles.displacement.plot)
            updateDisplacementDisplay(guiHandles.displacement.plot, guiHandles.displacement.text, ...
                dataStore.displacement.x, dataStore.displacement.y, dataStore.displacement.z, ...
                pointIndex, guiHandles.displacement.axes, dataStore.window.currentStart, dataStore.window.count);
        end
        
        % 定期更新姿态显示（性能优化）
        if mod(pointIndex, 10) == 0 || pointIndex == 1
            if isfield(guiHandles, 'attitude') && ~isempty(guiHandles.attitude.quiver)
                updateAttitudeDisplay(guiHandles.attitude.quiver, guiHandles.attitude.text, guiHandles.attitude.sphere, ...
                    double(dataStore.raw.accelX(pointIndex)), double(dataStore.raw.accelY(pointIndex)), double(dataStore.raw.accelZ(pointIndex)), ...
                    dataStore.compensated.gyroX(pointIndex), dataStore.compensated.gyroY(pointIndex), dataStore.compensated.gyroZ(pointIndex), ...
                    guiHandles.attitude.axes, imuParams.accelRange, imuParams.gyroRange, ...
                    dataStore.attitude.pitch(pointIndex), dataStore.attitude.roll(pointIndex), dataStore.attitude.yaw(pointIndex));
            end
        end
        
        % 更新调试显示 - 添加 sysParams 参数
        if pointIndex > 1 && mod(pointIndex, 5) == 0
            updateDebugDisplay(dataStore, guiHandles, pointIndex, imuParams, sysParams);
        end
        
        % 限制图形更新频率
        drawnow limitrate;
        
        % 显示同步状态警告
        if ~syncValid
            fprintf('点 %d: 数据帧同步丢失\n', pointIndex);
        end
        
    catch ME
        fprintf('GUI更新错误 (点 %d): %s\n', pointIndex, ME.message);
    end
end