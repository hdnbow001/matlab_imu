function [pitch, roll] = calculateAttitude(ax, ay, az, accel_range)
    % 将原始数据转换为G单位
    ax_g = double(ax / 32768) * accel_range;
    ay_g = double(ay / 32768) * accel_range;
    az_g = double(az / 32768) * accel_range;
    
    % 计算重力向量的大小
    g_magnitude = sqrt(ax_g^2 + ay_g^2 + az_g^2);
    
    % 处理异常情况
    if g_magnitude < 0.5  % 重力太小，可能处于自由落体或剧烈运动
        pitch = 0;
        roll = 0;
        return;
    end
    
    % 归一化加速度向量
    ax_norm = ax_g / g_magnitude;
    ay_norm = ay_g / g_magnitude;
    az_norm = az_g / g_magnitude;
    
    % 计算俯仰角(pitch) - 基于Z-X平面，绕Y轴的转角
    if abs(az_norm) < 1e-6
        if ax_norm >= 0
            pitch_rad = pi/2;
        else
            pitch_rad = -pi/2;
        end
    else
        pitch_rad = atan2(ax_norm, az_norm);
    end
    
    % 计算滚转角(roll) - 基于Z-Y平面，绕X轴的转角
    if abs(az_norm) < 1e-6
        if ay_norm >= 0
            roll_rad = pi/2;
        else
            roll_rad = -pi/2;
        end
    else
        roll_rad = atan2(ay_norm, az_norm);
    end
    
    % 转换为角度并规范化到0-360°
    pitch_deg = pitch_rad * 180/pi;
    roll_deg = roll_rad * 180/pi;
    
    pitch = normalizeAngle360(pitch_deg);
    roll = normalizeAngle360(roll_deg);
    
    % 验证角度计算的合理性
    if abs(pitch) > 360 || abs(roll) > 360
        warning('角度计算异常: pitch=%.1f°, roll=%.1f°', pitch, roll);
        pitch = mod(pitch, 360);
        roll = mod(roll, 360);
    end
end

function angle_out = normalizeAngle360(angle_in)
    % 将角度标准化到0-360°范围
    angle_out = mod(angle_in, 360);
    if angle_out < 0
        angle_out = angle_out + 360;
    end
end