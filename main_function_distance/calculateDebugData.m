% 计算调试数据函数
function [linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = calculateDebugData(accelX, accelY, accelZ, pitchAngles, rollAngles, dt, accel_range)
    % 地球重力加速度参考值 (m/s?)
    G = 9.80665;
    
    % 将加速度数据转换为G单位，然后转换为m/s?
    accelX_g = (accelX / 32768) * accel_range;
    accelY_g = (accelY / 32768) * accel_range;
    accelZ_g = (accelZ / 32768) * accel_range;
    
    accelX_mps2 = accelX_g * G;
    accelY_mps2 = accelY_g * G;
    accelZ_mps2 = accelZ_g * G;
    
    % 将角度转换为弧度
    pitchRad = pitchAngles * pi/180;
    rollRad = rollAngles * pi/180;
    
    % 初始化线性加速度数组
    linearAccelX = zeros(size(accelX_mps2));
    linearAccelY = zeros(size(accelY_mps2));
    linearAccelZ = zeros(size(accelZ_mps2));
    
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
    alpha = 0.95; % 滤波系数
    % filteredAccelX = improvedHighPassFilterSimple(linearAccelX, alpha, dt);
    % filteredAccelY = improvedHighPassFilterSimple(linearAccelY, alpha, dt);
    % filteredAccelZ = improvedHighPassFilterSimple(linearAccelZ, alpha, dt);
    filteredAccelX = improvedHighPassFilter(linearAccelX, alpha, dt);
    filteredAccelY = improvedHighPassFilter(linearAccelY, alpha, dt);
    filteredAccelZ = improvedHighPassFilter(linearAccelZ, alpha, dt);
    
    % 第一次积分：加速度 -> 速度
    velX = cumtrapz(filteredAccelX) * dt;
    velY = cumtrapz(filteredAccelY) * dt;
    velZ = cumtrapz(filteredAccelZ) * dt;
    
    % 应用高通滤波器去除速度漂移
    % velX = improvedHighPassFilterSimple(velX, alpha, dt);
    % velY = improvedHighPassFilterSimple(velY, alpha, dt);
    % velZ = improvedHighPassFilterSimple(velZ, alpha, dt);
    velX = improvedHighPassFilter(velX, alpha, dt);
    velY = improvedHighPassFilter(velY, alpha, dt);
    velZ = improvedHighPassFilter(velZ, alpha, dt);
    
    % 返回线性加速度和速度
    linAccelX = filteredAccelX;
    linAccelY = filteredAccelY;
    linAccelZ = filteredAccelZ;
end