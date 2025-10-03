% 更新姿态显示函数
function updateAttitudeDisplay(attitudeQuiver, attitudeText, ax, ay, az, gx, gy, gz)
    % 地球重力加速度参考值 (m/s?)
    G = 9.80665;
    
    % 计算俯仰角(pitch)和滚转角(roll) - 从加速度计
    % 由于传感器z轴与重力方向相反，需要调整符号
    pitch_acc = atan2(-ax, sqrt(ay^2 + az^2)) * 180/pi;
    roll_acc = atan2(-ay, -az) * 180/pi;
    
    % 将角度转换为0-360度范围
    pitch_acc = mod(pitch_acc, 360);
    roll_acc = mod(roll_acc, 360);
    
    % 简化的偏航角(yaw)估计 - 从陀螺仪积分（非常简化）
    % 注意：这是一个非常简化的方法，实际应用中需要更复杂的传感器融合算法
    persistent yaw_angle;
    if isempty(yaw_angle)
        yaw_angle = 0;
    end
    
    % 简单的积分计算偏航角
    dt = 0.2; % 采样间隔（假设5Hz采样率）
    yaw_angle = yaw_angle + gz * dt; % 使用Z轴陀螺仪数据
    
    % 将偏航角限制在0-360度范围内
    yaw_angle = mod(yaw_angle, 360);
    
    % 归一化加速度向量
    normAccel = sqrt(ax^2 + ay^2 + az^2);
    if normAccel > 0
        ux = ax / normAccel;
        uy = ay / normAccel;
        uz = az / normAccel;
    else
        ux = 0; uy = 0; uz = 0;
    end
    
    % 更新加速度矢量
    set(attitudeQuiver, 'UData', ux, 'VData', uy, 'WData', uz);
    
    % 更新角度信息文本
    set(attitudeText, 'String', sprintf('俯仰角: %.2f°\n滚转角: %.2f°\n偏航角: %.2f°', ...
        pitch_acc, roll_acc, yaw_angle));
end