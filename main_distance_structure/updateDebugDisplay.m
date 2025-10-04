function updateDebugDisplay(dataStore, guiHandles, pointIndex, imuParams, sysParams)
    % 更新调试显示
    
    persistent debugDataUpdated
    
    if isempty(debugDataUpdated)
        debugDataUpdated = false;
    end
    
    try
        % 检查调试图形窗口是否还存在
        if ~isfield(guiHandles, 'debugFig') || ~isvalid(guiHandles.debugFig)
            return;
        end
        
        % 检查调试图形句柄是否存在
        if ~isfield(guiHandles, 'h_accel_x') || ~isvalid(guiHandles.h_accel_x)
            return;
        end
        
        % 计算最近50个点的调试数据
        windowStart = max(1, pointIndex - 50);
        windowIndices = windowStart:pointIndex;
        
        % 使用现有的calculateDebugData函数
        if exist('calculateDebugData', 'file')
            [linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = ...
                calculateDebugData(...
                double(dataStore.raw.accelX(windowIndices)), ...
                double(dataStore.raw.accelY(windowIndices)), ...
                double(dataStore.raw.accelZ(windowIndices)), ...
                dataStore.attitude.pitch(windowIndices), ...
                dataStore.attitude.roll(windowIndices), ...
                sysParams.dt, imuParams.accelRange);
            
            % 更新调试图形
            set(guiHandles.h_accel_x, 'XData', windowIndices, 'YData', linAccelX);
            set(guiHandles.h_accel_y, 'XData', windowIndices, 'YData', linAccelY);
            set(guiHandles.h_accel_z, 'XData', windowIndices, 'YData', linAccelZ);
            
            set(guiHandles.h_velocity_x, 'XData', windowIndices, 'YData', velX);
            set(guiHandles.h_velocity_y, 'XData', windowIndices, 'YData', velY);
            set(guiHandles.h_velocity_z, 'XData', windowIndices, 'YData', velZ);
            
            set(guiHandles.h_displacement_x, 'XData', windowIndices, 'YData', dataStore.displacement.x(windowIndices));
            set(guiHandles.h_displacement_y, 'XData', windowIndices, 'YData', dataStore.displacement.y(windowIndices));
            set(guiHandles.h_displacement_z, 'XData', windowIndices, 'YData', dataStore.displacement.z(windowIndices));
            
            % 刷新调试图形
            drawnow limitrate;
        end
        
        debugDataUpdated = true;
        
    catch ME
        if ~debugDataUpdated
            fprintf('调试显示更新失败: %s (此错误只显示一次)\n', ME.message);
            debugDataUpdated = true;
        end
    end
end