function [dx, dy, dz, linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = calculateDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range, window_count)
    % 地球重力加速度参考值 (m/s²)
    G = 9.80665;
    
    % 数据转换（保持不变）
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
    linearAccelX = zeros(size(accelX_mps2));
    linearAccelY = zeros(size(accelY_mps2));
    linearAccelZ = zeros(size(accelZ_mps2));
    stationary = false(size(accelX_mps2));
    
    % 重力补偿（保持不变）
    for i = 1:length(accelX_mps2)
        R_x = [1, 0, 0; 0, cos(rollRad(i)), -sin(rollRad(i)); 0, sin(rollRad(i)), cos(rollRad(i))];
        R_y = [cos(pitchRad(i)), 0, sin(pitchRad(i)); 0, 1, 0; -sin(pitchRad(i)), 0, cos(pitchRad(i))];
        R_ws = R_x * R_y;
        
        gravity_world = [0; 0; -G];
        gravity_sensor = R_ws * gravity_world;
        
        linearAccelSensor = [accelX_mps2(i); accelY_mps2(i); accelZ_mps2(i)] - gravity_sensor;
        linearAccelWorld = R_ws' * linearAccelSensor;
        
        linearAccelX(i) = linearAccelWorld(1);
        linearAccelY(i) = linearAccelWorld(2);
        linearAccelZ(i) = linearAccelWorld(3);
        
        % 零速度检测
        accelNorm = sqrt(accelX_mps2(i)^2 + accelY_mps2(i)^2 + accelZ_mps2(i)^2);
        gyroNorm = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
        
        if abs(accelNorm - G) < 0.2 && gyroNorm < 2
            stationary(i) = true;
        end
    end
    
    % 偏置消除
    if length(linearAccelX) > 5
        initial_bias_X = mean(linearAccelX(1:5));
        initial_bias_Y = mean(linearAccelY(1:5));
        initial_bias_Z = mean(linearAccelZ(1:5));
    else
        initial_bias_X = mean(linearAccelX);
        initial_bias_Y = mean(linearAccelY);
        initial_bias_Z = mean(linearAccelZ);
    end
    
    linearAccelX = linearAccelX - initial_bias_X;
    linearAccelY = linearAccelY - initial_bias_Y;
    linearAccelZ = linearAccelZ - initial_bias_Z;
    
    % ========== 关键修改：简化滤波策略 ==========
    
    % 只在加速度层面应用一次高通滤波
    persistent prev_window_count_improved prev_alpha;
    if isempty(prev_window_count_improved) || prev_window_count_improved ~= window_count
        prev_alpha = 0.98; % 重置滤波器系数
        prev_window_count_improved = window_count;
    end
    
    % 应用自适应高通滤波（只在加速度层面）
    fc = 0.1; % 截止频率 0.1Hz
    alpha = dt / (1/(2*pi*fc) + dt);
    
    filteredAccelX = zeros(size(linearAccelX));
    filteredAccelY = zeros(size(linearAccelY));
    filteredAccelZ = zeros(size(linearAccelZ));
    
    for i = 1:length(linearAccelX)
        if i == 1
            filteredAccelX(i) = (1 - alpha) * linearAccelX(i);
            filteredAccelY(i) = (1 - alpha) * linearAccelY(i);
            filteredAccelZ(i) = (1 - alpha) * linearAccelZ(i);
        else
            filteredAccelX(i) = alpha * filteredAccelX(i-1) + (1 - alpha) * (linearAccelX(i) - linearAccelX(i-1));
            filteredAccelY(i) = alpha * filteredAccelY(i-1) + (1 - alpha) * (linearAccelY(i) - linearAccelY(i-1));
            filteredAccelZ(i) = alpha * filteredAccelZ(i-1) + (1 - alpha) * (linearAccelZ(i) - linearAccelZ(i-1));
        end
        
        % 在静止点强制加速度为零
        if stationary(i)
            filteredAccelX(i) = 0;
            filteredAccelY(i) = 0;
            filteredAccelZ(i) = 0;
        end
    end
    
    % ========== 积分计算 ==========
    
    % 第一次积分：加速度 -> 速度
    velocityX = cumtrapz(filteredAccelX) * dt;
    velocityY = cumtrapz(filteredAccelY) * dt;
    velocityZ = cumtrapz(filteredAccelZ) * dt;
    
    % 改进的静止点处理：只在检测到静止时重置速度
    % 而不是重置整个后续数组
    for i = 1:length(velocityX)
        if stationary(i)
            % 只重置当前点的速度，而不是整个后续数组
            velocityX(i) = 0;
            velocityY(i) = 0;
            velocityZ(i) = 0;
        end
    end
    
    % 第二次积分：速度 -> 位移
    dx_array = cumtrapz(velocityX) * dt;
    dy_array = cumtrapz(velocityY) * dt;
    dz_array = cumtrapz(velocityZ) * dt;
    
    % 返回中间结果
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