function updateDebugDisplay_(debugHandles, currentIndex, debugData, displacementX, displacementY, displacementZ)
    % 更新调试显示 - 使用calculateDisplacement计算的中间结果
    
    % 计算显示窗口（最近50个采样点）
    window_indices = max(1, currentIndex-50):currentIndex;
    
    % 确保数据长度匹配
    if length(window_indices) > length(debugData.linAccelX)
        window_indices = window_indices(1:length(debugData.linAccelX));
    end
    
    % 更新线性加速度曲线
    if currentIndex <= length(debugData.linAccelX)
        set(debugHandles.h_accel_x, 'XData', window_indices, 'YData', debugData.linAccelX(window_indices));
        set(debugHandles.h_accel_y, 'XData', window_indices, 'YData', debugData.linAccelY(window_indices));
        set(debugHandles.h_accel_z, 'XData', window_indices, 'YData', debugData.linAccelZ(window_indices));
    end
    
    % 更新速度曲线
    if currentIndex <= length(debugData.velX)
        set(debugHandles.h_velocity_x, 'XData', window_indices, 'YData', debugData.velX(window_indices));
        set(debugHandles.h_velocity_y, 'XData', window_indices, 'YData', debugData.velY(window_indices));
        set(debugHandles.h_velocity_z, 'XData', window_indices, 'YData', debugData.velZ(window_indices));
    end
    
    % 更新位移曲线
    if currentIndex <= length(displacementX)
        set(debugHandles.h_displacement_x, 'XData', window_indices, 'YData', displacementX(window_indices));
        set(debugHandles.h_displacement_y, 'XData', window_indices, 'YData', displacementY(window_indices));
        set(debugHandles.h_displacement_z, 'XData', window_indices, 'YData', displacementZ(window_indices));
    end
    
    % 刷新图形显示
    drawnow limitrate;
end