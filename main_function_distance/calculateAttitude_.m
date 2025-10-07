function [pitch, roll] = calculateAttitude_(ax, ay, az, accel_range)
    % 将原始数据转换为G单位
    ax_g = double(ax / 32768) * accel_range;
    ay_g = double(ay / 32768) * accel_range;
    az_g = double(az / 32768) * accel_range;
    
    % ========== 基于明确坐标轴定义的计算 ==========
    % 坐标轴定义：
    % - Z轴正方向：accelZData = 1G 时的方向
    % - X轴正方向：accelXData = 1G 时的方向
    % - Y轴正方向：accelYData = 1G 时的方向
    %
    % 角度定义：
    % - 俯仰角(pitch)：绕X轴旋转，在Z-Y平面内变化
    % - 滚转角(roll)：绕Y轴旋转，在Z-X平面内变化
    % - 航向角(yaw)：固定为0（X-Y平面内）
    
    % 计算重力向量的大小
    g_magnitude = sqrt(ax_g^2 + ay_g^2 + az_g^2);
    
    % 处理异常情况
    if g_magnitude < 0.5  % 重力太小，可能处于自由落体或剧烈运动
        pitch = 0;
        roll = 0;
        return;
    end
    
    % ========== 俯仰角(pitch)计算 ==========
    % 绕X轴旋转，在Z-Y平面内变化
    % 当设备绕X轴旋转时，Y和Z分量变化
    % 使用atan2确保正确的象限
    
    % 避免除零错误
    if abs(az_g) < 1e-6
        if ay_g >= 0
            pitch_rad = pi/2;
        else
            pitch_rad = -pi/2;
        end
    else
        pitch_rad = atan2(ay_g, az_g);
    end
    
    % ========== 滚转角(roll)计算 ==========
    % 绕Y轴旋转，在Z-X平面内变化
    % 当设备绕Y轴旋转时，X和Z分量变化
    % 使用atan2确保正确的象限
    
    % 避免除零错误
    if abs(az_g) < 1e-6
        if ax_g >= 0
            roll_rad = pi/2;
        else
            roll_rad = -pi/2;
        end
    else
        roll_rad = atan2(ax_g, az_g);
    end
    
    % 转换为角度
    pitch_deg = pitch_rad * 180/pi;
    roll_deg = roll_rad * 180/pi;
    
    % ========== 转换为0-360°范围 ==========
    
    pitch = normalizeAngle(pitch_deg);
    roll = normalizeAngle(roll_deg);
    
    % 调试输出（可选）
    % fprintf('姿态计算: ax=%.3fG, ay=%.3fG, az=%.3fG -> pitch=%.1f°, roll=%.1f°\n', ...
    %         ax_g, ay_g, az_g, pitch, roll);
end

function angle_out = normalizeAngle(angle_in)
    % 将角度标准化到0-360°范围
    % 先将角度规范化到[-180, 180]范围
    angle_norm = mod(angle_in + 180, 360) - 180;
    
    % 转换为0-360°范围
    if angle_norm < 0
        angle_out = angle_norm + 360;
    else
        angle_out = angle_norm;
    end
    
    % 处理正好等于360的情况
    if angle_out >= 360
        angle_out = 0;
    end
end