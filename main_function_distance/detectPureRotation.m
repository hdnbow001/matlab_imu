% =========================================================================
% 纯旋转检测函数
% =========================================================================
function is_pure_rotation = detectPureRotation(accel_sensor, gravity_sensor, gyroX, gyroY, gyroZ)
    % 检测是否为纯旋转运动
    
    G = 9.80665;
    
    % 计算传感器读数的模
    accel_norm = norm(accel_sensor);
    gravity_norm = norm(gravity_sensor);
    
    % 计算角速度幅度
    gyro_magnitude = sqrt(gyroX^2 + gyroY^2 + gyroZ^2);
    
    % 纯旋转的判定条件：
    % 1. 加速度计读数接近重力大小（说明没有明显的线性加速度）
    % 2. 角速度超过阈值（说明有旋转运动）
    % 3. 加速度计读数与重力向量的夹角变化主要由旋转引起
    
    condition1 = abs(accel_norm - G) < 0.2;  % 加速度模接近重力
    condition2 = gyro_magnitude > 10;        % 角速度大于10dps
    condition3 = gyro_magnitude > 5;         % 保守条件
    
    % 计算加速度向量与重力向量的夹角余弦
    cos_angle = dot(accel_sensor, gravity_sensor) / (accel_norm * gravity_norm);
    angle_diff = acos(abs(cos_angle)) * 180/pi;
    
    condition4 = angle_diff < 15;  % 夹角变化较小
    
    % 综合判定
    is_pure_rotation = condition1 && condition2 && condition3 && condition4;
    
    % 如果角速度很大且加速度接近重力，强制判定为纯旋转
    if gyro_magnitude > 30 && abs(accel_norm - G) < 0.3
        is_pure_rotation = true;
    end
end