function [dx, dy, dz, linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = calculateDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range)
    % 窗口位移计算函数 - 计算当前窗口内的位移
    
    % 地球重力加速度参考值 (m/s²)
    G = 9.80665;
    
    % 数据转换
    accelX_g = double((accelX / 32768)) * accel_range;
    accelY_g = double((accelY / 32768)) * accel_range;
    accelZ_g = double((accelZ / 32768)) * accel_range;
    
    accelX_mps2 = accelX_g * G;
    accelY_mps2 = accelY_g * G;
    accelZ_mps2 = accelZ_g * G;
    
    gyroX_dps = (gyroX / 32768) * gyro_range;
    gyroY_dps = (gyroY / 32768) * gyro_range;
    gyroZ_dps = (gyroZ / 32768) * gyro_range;
    
    % 将角度转换为弧度
    pitchRad = pitchAngles * pi/180;
    rollRad = rollAngles * pi/180;
    
    % 初始化数组
    n = length(accelX_mps2);
    linearAccelX = zeros(size(accelX_mps2));
    linearAccelY = zeros(size(accelY_mps2));
    linearAccelZ = zeros(size(accelZ_mps2));
    stationary = false(size(accelX_mps2));
    
    % 初始化偏航角
    cumulative_yaw = 0;
    
    for i = 1:n
        % 计算基本旋转矩阵（俯仰和滚转）
        R_x = [1, 0, 0; 
               0, cos(rollRad(i)), -sin(rollRad(i)); 
               0, sin(rollRad(i)), cos(rollRad(i))];
           
        R_y = [cos(pitchRad(i)), 0, sin(pitchRad(i));
               0, 1, 0;
               -sin(pitchRad(i)), 0, cos(pitchRad(i))];
        
        % 计算偏航角变化（基于陀螺仪Z轴）
        if i > 1
            yaw_change = (gyroZ_dps(i) + gyroZ_dps(i-1)) * 0.5 * dt * pi/180;
            cumulative_yaw = cumulative_yaw + yaw_change;
        end
        
        % 偏航旋转矩阵
        R_z = [cos(cumulative_yaw), -sin(cumulative_yaw), 0;
               sin(cumulative_yaw), cos(cumulative_yaw), 0;
               0, 0, 1];
        
        % 完整的旋转矩阵：先偏航，再俯仰，最后滚转
        R_ws = R_x * R_y * R_z;
        
        % 重力向量在世界坐标系中
        gravity_world = [0; 0; -G];
        
        % 将重力向量旋转到传感器坐标系
        gravity_sensor = R_ws * gravity_world;
        
        % 传感器读数
        accel_sensor = [accelX_mps2(i); accelY_mps2(i); accelZ_mps2(i)];
        
        % 检测纯旋转运动
        current_gyro = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
        current_accel = norm(accel_sensor);
        is_pure_rotation = current_gyro > 15.0 && abs(current_accel - G) < 0.3 * G;
        
        if is_pure_rotation
            % 纯旋转情况：完全抑制线性加速度
            linearAccelSensor = [0; 0; 0];
            stationary(i) = true;
        else
            % 正常情况：减去重力分量
            linearAccelSensor = accel_sensor - gravity_sensor;
            stationary(i) = false;
        end
        
        % 将线性加速度从传感器坐标系旋转回世界坐标系
        linearAccelWorld = R_ws' * linearAccelSensor;
        
        linearAccelX(i) = linearAccelWorld(1);
        linearAccelY(i) = linearAccelWorld(2);
        linearAccelZ(i) = linearAccelWorld(3);
    end
    
    % 零速度检测
    stationary_detailed = detectStationaryPointsEnhanced(accelX_mps2, accelY_mps2, accelZ_mps2, ...
                                               gyroX_dps, gyroY_dps, gyroZ_dps,G);
    
    % 合并旋转检测和静止检测
    for i = 1:n
        if stationary_detailed(i)
            stationary(i) = true;
            linearAccelX(i) = 0;
            linearAccelY(i) = 0;
            linearAccelZ(i) = 0;
        end
    end
    
    % 偏置消除
    stationary_indices = find(stationary);
    if length(stationary_indices) >= 3
        bias_X = mean(linearAccelX(stationary_indices));
        bias_Y = mean(linearAccelY(stationary_indices));
        bias_Z = mean(linearAccelZ(stationary_indices));
    else
        % 使用前几个点计算偏置
        num_init = min(10, n);
        bias_X = mean(linearAccelX(1:num_init));
        bias_Y = mean(linearAccelY(1:num_init));
        bias_Z = mean(linearAccelZ(1:num_init));
    end
    
    linearAccelX_corrected = linearAccelX - bias_X;
    linearAccelY_corrected = linearAccelY - bias_Y;
    linearAccelZ_corrected = linearAccelZ - bias_Z;
    
    % 在静止点强制加速度为零
    for i = 1:n
        if stationary(i)
            linearAccelX_corrected(i) = 0;
            linearAccelY_corrected(i) = 0;
            linearAccelZ_corrected(i) = 0;
        end
    end
    
    % 应用低通滤波
    alpha = 0.8; % 滤波系数
    
    filteredAccelX = zeros(size(linearAccelX_corrected));
    filteredAccelY = zeros(size(linearAccelY_corrected));
    filteredAccelZ = zeros(size(linearAccelZ_corrected));
    
    for i = 1:n
        if i == 1
            filteredAccelX(i) = linearAccelX_corrected(i);
            filteredAccelY(i) = linearAccelY_corrected(i);
            filteredAccelZ(i) = linearAccelZ_corrected(i);
        else
            filteredAccelX(i) = alpha * filteredAccelX(i-1) + (1-alpha) * linearAccelX_corrected(i);
            filteredAccelY(i) = alpha * filteredAccelY(i-1) + (1-alpha) * linearAccelY_corrected(i);
            filteredAccelZ(i) = alpha * filteredAccelZ(i-1) + (1-alpha) * linearAccelZ_corrected(i);
        end
        
        % 在静止点强制加速度为零
        if stationary(i)
            filteredAccelX(i) = 0;
            filteredAccelY(i) = 0;
            filteredAccelZ(i) = 0;
        end
    end
    
    % 积分计算
    velocityX = integrateAcceleration(filteredAccelX, dt, stationary);
    velocityY = integrateAcceleration(filteredAccelY, dt, stationary);
    velocityZ = integrateAcceleration(filteredAccelZ, dt, stationary);
    
    % 第二次积分：速度 -> 位移
    dx_array = integrateVelocity(velocityX, dt);
    dy_array = integrateVelocity(velocityY, dt);
    dz_array = integrateVelocity(velocityZ, dt);
    
    % 返回结果
    linAccelX = filteredAccelX;
    linAccelY = filteredAccelY;
    linAccelZ = filteredAccelZ;
    velX = velocityX;
    velY = velocityY;
    velZ = velocityZ;
    
    % 返回最后一个位移值
    dx = dx_array(end);
    dy = dy_array(end);
    dz = dz_array(end);
end