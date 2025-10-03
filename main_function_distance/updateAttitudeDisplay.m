% 更新姿态显示函数
function updateAttitudeDisplay(attitudeQuiver, attitudeText, attitudeSphere, ax, ay, az, gx, gy, gz, attitudeAxes, accel_range, gyro_range, pitch, roll, yaw)
    % 将原始数据转换为G单位
    ax_g = (ax / 32768) * accel_range;
    ay_g = (ay / 32768) * accel_range;
    az_g = (az / 32768) * accel_range;
    
    % 归一化加速度向量
    normAccel = sqrt(ax_g^2 + ay_g^2 + az_g^2);
    if normAccel > 0
        ux = ax_g / normAccel;
        uy = ay_g / normAccel;
        uz = az_g / normAccel;
    else
        ux = 0; uy = 0; uz = 0;
    end
    
    % 设置当前坐标轴
    axes(attitudeAxes);
    
    % 更新加速度矢量
    set(attitudeQuiver, 'UData', ux, 'VData', uy, 'WData', uz);
    
    % 修正文本更新 - 使用sprintf生成带换行的文本
    angleText = sprintf('俯仰角: %.2f°\n滚转角: %.2f°\n偏航角: %.2f°', pitch, roll, yaw);
    set(attitudeText, 'String', angleText);
    
    % 确保球面和其他图形元素保持可见
    set(attitudeSphere, 'Visible', 'on');
    
    % 刷新图形
    drawnow limitrate;
end