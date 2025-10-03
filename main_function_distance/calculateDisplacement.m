% 计算位移函数 - 改进版本，考虑重力加速度影响和零速度检测
function [dx, dy, dz] = calculateDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range, window_count)
    % 地球重力加速度参考值 (m/s?)
    G = 9.80665;
    
    % 将加速度数据转换为G单位，然后转换为m/s?
    accelX_g = (accelX / 32768) * accel_range;
    accelY_g = (accelY / 32768) * accel_range;
    accelZ_g = (accelZ / 32768) * accel_range;
    
    accelX_mps2 = accelX_g * G;
    accelY_mps2 = accelY_g * G;
    accelZ_mps2 = accelZ_g * G;
    
    % 将陀螺仪数据转换为度/秒
    gyroX_dps = (gyroX / 32768) * gyro_range;
    gyroY_dps = (gyroY / 32768) * gyro_range;
    gyroZ_dps = (gyroZ / 32768) * gyro_range;
    
    % 将角度转换为弧度
    pitchRad = pitchAngles * pi/180;
    rollRad = rollAngles * pi/180;
    
    % 初始化线性加速度数组
    linearAccelX = zeros(size(accelX_mps2));
    linearAccelY = zeros(size(accelY_mps2));
    linearAccelZ = zeros(size(accelZ_mps2));
    
    % 零速度检测标志
    stationary = false(size(accelX_mps2));
    
    % 对每个采样点处理
    for i = 1:length(accelX_mps2)
        % 计算旋转矩阵（从世界坐标系到传感器坐标系）
        R_x = [1, 0, 0;
               0, cos(rollRad(i)), -sin(rollRad(i));
               0, sin(rollRad(i)), cos(rollRad(i))];
           
        R_y = [cos(pitchRad(i)), 0, sin(pitchRad(i));
               0, 1, 0;
               -sin(pitchRad(i)), 0, cos(pitchRad(i))];
           
        % 组合旋转矩阵
        R_ws = R_x * R_y; % 先俯仰后滚转
        
        % 世界坐标系中的重力向量 [0; 0; -G] (Z轴向上)
        gravity_world = [0; 0; -G];
        
        % 将重力向量旋转到传感器坐标系
        gravity_sensor = R_ws * gravity_world;
        
        % 从传感器读数中减去重力分量
        linearAccelSensor = [accelX_mps2(i); accelY_mps2(i); accelZ_mps2(i)] - gravity_sensor;
        
        % 将线性加速度从传感器坐标系旋转回世界坐标系
        linearAccelWorld = R_ws' * linearAccelSensor;
        
        % 存储线性加速度
        linearAccelX(i) = linearAccelWorld(1);
        linearAccelY(i) = linearAccelWorld(2);
        linearAccelZ(i) = linearAccelWorld(3);
        
        % 零速度检测
        accelNorm = sqrt(accelX_mps2(i)^2 + accelY_mps2(i)^2 + accelZ_mps2(i)^2);
        gyroNorm = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
        
        % 检查是否静止（加速度模接近G，角速度模接近0）
        if abs(accelNorm - G) < 0.2 && gyroNorm < 2
            stationary(i) = true;
            % 如果是静止点，则将线性加速度置零
            linearAccelX(i) = 0;
            linearAccelY(i) = 0;
            linearAccelZ(i) = 0;
        end
    end
    
    % 在每个窗口开始时，减去初始偏置（前几个点的平均值）
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
    
    % 应用高通滤波器去除直流分量和低频噪声
    % 使用持久变量来保持滤波器状态，但每个窗口重置
    persistent prev_window_count prev_filteredAccelX prev_filteredAccelY prev_filteredAccelZ;
    
    % 检查是否是新的窗口
    if isempty(prev_window_count) || prev_window_count ~= window_count
        % 新窗口，重置滤波器状态
        prev_filteredAccelX = 0;
        prev_filteredAccelY = 0;
        prev_filteredAccelZ = 0;
        prev_window_count = window_count;
    end
    
    % 应用高通滤波器
    alpha = 0.95; % 滤波系数
    [filteredAccelX, prev_filteredAccelX] = improvedHighPassFilter(linearAccelX, alpha, dt, prev_filteredAccelX);
    [filteredAccelY, prev_filteredAccelY] = improvedHighPassFilter(linearAccelY, alpha, dt, prev_filteredAccelY);
    [filteredAccelZ, prev_filteredAccelZ] = improvedHighPassFilter(linearAccelZ, alpha, dt, prev_filteredAccelZ);
    
    % 双重积分计算位移
    % 第一次积分：加速度 -> 速度
    velocityX = cumtrapz(filteredAccelX) * dt;
    velocityY = cumtrapz(filteredAccelY) * dt;
    velocityZ = cumtrapz(filteredAccelZ) * dt;
    
    % 在静止点重置速度
    for i = 1:length(velocityX)
        if stationary(i)
            velocityX(i:end) = velocityX(i:end) - velocityX(i);
            velocityY(i:end) = velocityY(i:end) - velocityY(i);
            velocityZ(i:end) = velocityZ(i:end) - velocityZ(i);
        end
    end
    
    % 应用高通滤波器去除速度漂移
    [velocityX, ~] = improvedHighPassFilter(velocityX, alpha, dt, 0);
    [velocityY, ~] = improvedHighPassFilter(velocityY, alpha, dt, 0);
    [velocityZ, ~] = improvedHighPassFilter(velocityZ, alpha, dt, 0);
    
    % 第二次积分：速度 -> 位移
    dx = cumtrapz(velocityX) * dt;
    dy = cumtrapz(velocityY) * dt;
    dz = cumtrapz(velocityZ) * dt;
    
    % 在静止点重置位移
    for i = 1:length(dx)
        if stationary(i)
            dx(i:end) = dx(i:end) - dx(i);
            dy(i:end) = dy(i:end) - dy(i);
            dz(i:end) = dz(i:end) - dz(i);
        end
    end
    
    % 应用高通滤波器去除位移漂移
    [dx, ~] = improvedHighPassFilter(dx, alpha, dt, 0);
    [dy, ~] = improvedHighPassFilter(dy, alpha, dt, 0);
    [dz, ~] = improvedHighPassFilter(dz, alpha, dt, 0);
    
    % 返回最后一个位移值
    dx = dx(end);
    dy = dy(end);
    dz = dz(end);
end