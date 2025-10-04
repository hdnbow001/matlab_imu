% 更新调试图表函数
% =========================================================================
function updateDebugDisplay(debugHandles, currentIndex, accelXData, accelYData, accelZData, ...
                         pitchAngles, rollAngles, displacementX, displacementY, displacementZ, ...
                         dt, accel_range)
    % 计算线性加速度、速度和位移用于调试
    window_indices = max(1, currentIndex-50):currentIndex;
    
    [linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = ...
        calculateDebugData(...
        double(accelXData(window_indices)), ...
        double(accelYData(window_indices)), ...
        double(accelZData(window_indices)), ...
        pitchAngles(window_indices), ...
        rollAngles(window_indices), ...
        dt, accel_range);
    
    % 分别更新每条曲线
    set(debugHandles.h_accel_x, 'XData', window_indices, 'YData', linAccelX);
    set(debugHandles.h_accel_y, 'XData', window_indices, 'YData', linAccelY);
    set(debugHandles.h_accel_z, 'XData', window_indices, 'YData', linAccelZ);
    
    set(debugHandles.h_velocity_x, 'XData', window_indices, 'YData', velX);
    set(debugHandles.h_velocity_y, 'XData', window_indices, 'YData', velY);
    set(debugHandles.h_velocity_z, 'XData', window_indices, 'YData', velZ);
    
    set(debugHandles.h_displacement_x, 'XData', window_indices, 'YData', displacementX(window_indices));
    set(debugHandles.h_displacement_y, 'XData', window_indices, 'YData', displacementY(window_indices));
    set(debugHandles.h_displacement_z, 'XData', window_indices, 'YData', displacementZ(window_indices));

    % 关键：强制坐标轴自动调整范围
    axis(debugHandles.ax_velocity, 'auto');  % 速度子图自动调整
    axis(debugHandles.ax_accel, 'auto');     % 加速度子图自动调整  
    axis(debugHandles.ax_displacement, 'auto'); % 位移子图自动调整
    
    % 刷新图形显示
    drawnow limitrate;
end