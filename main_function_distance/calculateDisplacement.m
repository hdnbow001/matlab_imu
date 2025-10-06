function [dx, dy, dz, linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = calculateDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range, window_count)
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
    
    % ========== 增强的旋转处理 ==========
    
    % 使用持久变量保持状态
    persistent cumulative_yaw prev_gyroZ prev_linearAccel rotation_filter_state;
    
    if isempty(cumulative_yaw) || window_count == 1
        cumulative_yaw = 0;
        prev_gyroZ = 0;
        prev_linearAccel = [0; 0; 0];
        rotation_filter_state = struct('gyro_history', [], 'accel_history', []);
    end
    
    % 预处理：分析窗口内的运动特征
    gyro_magnitudes = sqrt(gyroX_dps.^2 + gyroY_dps.^2 + gyroZ_dps.^2);
    accel_magnitudes = sqrt(accelX_mps2.^2 + accelY_mps2.^2 + accelZ_mps2.^2);
    
    mean_gyro = mean(gyro_magnitudes);
    std_accel = std(accel_magnitudes);
    
    % 判断是否为旋转主导的运动窗口
    is_rotation_window = mean_gyro > 8.0 && std_accel < 0.3 * G;
    
    for i = 1:n
        % 计算当前时刻的旋转矩阵
        R_x = [1, 0, 0; 
               0, cos(rollRad(i)), -sin(rollRad(i)); 
               0, sin(rollRad(i)), cos(rollRad(i))];
           
        R_y = [cos(pitchRad(i)), 0, sin(pitchRad(i));
               0, 1, 0;
               -sin(pitchRad(i)), 0, cos(pitchRad(i))];
        
        % 更新偏航角（仅使用Z轴陀螺仪）
        if i > 1
            yaw_change = (gyroZ_dps(i) + prev_gyroZ) * 0.5 * dt * pi/180;
            cumulative_yaw = cumulative_yaw + yaw_change;
        end
        
        R_z = [cos(cumulative_yaw), -sin(cumulative_yaw), 0;
               sin(cumulative_yaw), cos(cumulative_yaw), 0;
               0, 0, 1];
        
        % 完整的旋转矩阵：从世界坐标系到传感器坐标系
        R_ws = R_x * R_y * R_z;
        
        % 重力向量在世界坐标系中
        gravity_world = [0; 0; -G];
        
        % 将重力向量旋转到传感器坐标系
        gravity_sensor = R_ws * gravity_world;
        
        % 当前传感器读数
        accel_sensor = [accelX_mps2(i); accelY_mps2(i); accelZ_mps2(i)];
        
        % ========== 关键改进：智能旋转检测 ==========
        
        current_gyro_mag = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
        current_accel_mag = norm(accel_sensor);
        gravity_mag = norm(gravity_sensor);
        
        % 计算加速度向量与重力向量的夹角
        if current_accel_mag > 0 && gravity_mag > 0
            cos_angle = dot(accel_sensor, gravity_sensor) / (current_accel_mag * gravity_mag);
            cos_angle = max(min(cos_angle, 1), -1); % 防止数值误差
            accel_gravity_angle = acos(cos_angle);
        else
            accel_gravity_angle = 0;
        end
        
        % 旋转检测条件
        condition1 = current_gyro_mag > 10.0 && abs(current_accel_mag - G) < 0.25 * G;
        condition2 = current_gyro_mag > 5.0 && accel_gravity_angle < 0.3; % 约17度
        condition3 = is_rotation_window && current_gyro_mag > 3.0;
        
        % 计算线性加速度残余（在补偿重力之前）
        residual_accel = norm(accel_sensor - gravity_sensor);
        condition4 = current_gyro_mag > 8.0 && residual_accel < 0.2 * G;
        
        is_pure_rotation = condition1 || condition2 || condition3 || condition4;
        
        if is_pure_rotation
            % 纯旋转情况：使用强抑制策略
            % 策略1：完全抑制线性加速度
            linearAccelSensor = [0; 0; 0];
            
            % 策略2：或者使用低通滤波（二选一）
            % linearAccelSensor = 0.05 * (accel_sensor - gravity_sensor) + 0.95 * prev_linearAccel;
            
            stationary(i) = true;
            
            % 调试信息
            if mod(i, 100) == 0 && window_count > 1
                fprintf('旋转抑制: 点 %d, 角速度=%.1f dps, 残余加速度=%.3f G\n', ...
                    i, current_gyro_mag, residual_accel/G);
            end
        else
            % 正常情况：标准重力补偿
            linearAccelSensor = accel_sensor - gravity_sensor;
            stationary(i) = false;
        end
        
        % 将线性加速度转换回世界坐标系
        linearAccelWorld = R_ws' * linearAccelSensor;
        
        linearAccelX(i) = linearAccelWorld(1);
        linearAccelY(i) = linearAccelWorld(2);
        linearAccelZ(i) = linearAccelWorld(3);
        
        % 更新状态变量
        prev_gyroZ = gyroZ_dps(i);
        prev_linearAccel = linearAccelSensor;
        
        % 零速度检测（增强版）
        if ~is_pure_rotation
            gyroNorm = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
            accelNorm = norm(accel_sensor);
            
            % 严格的静止条件
            if abs(accelNorm - G) < 0.08 * G && gyroNorm < 1.0
                stationary(i) = true;
                % 在静止点强制加速度为零
                linearAccelX(i) = 0;
                linearAccelY(i) = 0;
                linearAccelZ(i) = 0;
            end
        end
    end
    
    % ========== 偏置消除 ==========
    
    if n > 10
        % 使用前10个点计算初始偏置（如果大部分是静止的）
        initial_bias_X = mean(linearAccelX(1:min(10,n)));
        initial_bias_Y = mean(linearAccelY(1:min(10,n)));
        initial_bias_Z = mean(linearAccelZ(1:min(10,n)));
    else
        initial_bias_X = mean(linearAccelX);
        initial_bias_Y = mean(linearAccelY);
        initial_bias_Z = mean(linearAccelZ);
    end
    
    linearAccelX = linearAccelX - initial_bias_X;
    linearAccelY = linearAccelY - initial_bias_Y;
    linearAccelZ = linearAccelZ - initial_bias_Z;
    
    % ========== 滤波处理 ==========
    
    % 使用持久变量保持滤波器状态
    persistent prev_window_count_filter prev_filteredX prev_filteredY prev_filteredZ;
    
    if isempty(prev_window_count_filter) || prev_window_count_filter ~= window_count
        prev_filteredX = 0;
        prev_filteredY = 0;
        prev_filteredZ = 0;
        prev_window_count_filter = window_count;
    end
    
    % 高通滤波器参数（去除直流偏置）
    fc_high = 0.1; % 截止频率 0.1Hz
    alpha_high = dt / (1/(2*pi*fc_high) + dt);
    
    % 低通滤波器参数（降噪）
    fc_low = 5.0; % 截止频率 5Hz
    alpha_low = dt / (1/(2*pi*fc_low) + dt);
    
    filteredAccelX = zeros(size(linearAccelX));
    filteredAccelY = zeros(size(linearAccelY));
    filteredAccelZ = zeros(size(linearAccelZ));
    
    for i = 1:n
        % 高通滤波（去除直流偏置）
        if i == 1
            highpassX = (1 - alpha_high) * linearAccelX(i);
            highpassY = (1 - alpha_high) * linearAccelY(i);
            highpassZ = (1 - alpha_high) * linearAccelZ(i);
        else
            highpassX = alpha_high * prev_filteredX + (1 - alpha_high) * (linearAccelX(i) - linearAccelX(i-1));
            highpassY = alpha_high * prev_filteredY + (1 - alpha_high) * (linearAccelY(i) - linearAccelY(i-1));
            highpassZ = alpha_high * prev_filteredZ + (1 - alpha_high) * (linearAccelZ(i) - linearAccelZ(i-1));
        end
        
        % 低通滤波（降噪）
        if i == 1
            filteredAccelX(i) = alpha_low * highpassX;
            filteredAccelY(i) = alpha_low * highpassY;
            filteredAccelZ(i) = alpha_low * highpassZ;
        else
            filteredAccelX(i) = alpha_low * highpassX + (1 - alpha_low) * filteredAccelX(i-1);
            filteredAccelY(i) = alpha_low * highpassY + (1 - alpha_low) * filteredAccelY(i-1);
            filteredAccelZ(i) = alpha_low * highpassZ + (1 - alpha_low) * filteredAccelZ(i-1);
        end
        
        prev_filteredX = filteredAccelX(i);
        prev_filteredY = filteredAccelY(i);
        prev_filteredZ = filteredAccelZ(i);
        
        % 在静止点强制加速度为零
        if stationary(i)
            filteredAccelX(i) = 0;
            filteredAccelY(i) = 0;
            filteredAccelZ(i) = 0;
        end
    end
    
    % ========== 速度积分 ==========
    
    velocityX = zeros(size(filteredAccelX));
    velocityY = zeros(size(filteredAccelY));
    velocityZ = zeros(size(filteredAccelZ));
    
    for i = 1:n
        if i == 1
            velocityX(i) = filteredAccelX(i) * dt;
            velocityY(i) = filteredAccelY(i) * dt;
            velocityZ(i) = filteredAccelZ(i) * dt;
        else
            velocityX(i) = velocityX(i-1) + filteredAccelX(i) * dt;
            velocityY(i) = velocityY(i-1) + filteredAccelY(i) * dt;
            velocityZ(i) = velocityZ(i-1) + filteredAccelZ(i) * dt;
        end
        
        % 在静止点重置速度
        if stationary(i)
            velocityX(i) = 0;
            velocityY(i) = 0;
            velocityZ(i) = 0;
        end
    end
    
    % ========== 位移积分 ==========
    
    dx_array = zeros(size(velocityX));
    dy_array = zeros(size(velocityY));
    dz_array = zeros(size(velocityZ));
    
    for i = 1:n
        if i == 1
            dx_array(i) = velocityX(i) * dt;
            dy_array(i) = velocityY(i) * dt;
            dz_array(i) = velocityZ(i) * dt;
        else
            dx_array(i) = dx_array(i-1) + velocityX(i) * dt;
            dy_array(i) = dy_array(i-1) + velocityY(i) * dt;
            dz_array(i) = dz_array(i-1) + velocityZ(i) * dt;
        end
    end
    
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
    
    % 调试输出
    if window_count > 1 && mod(window_count, 5) == 0
        stationary_count = sum(stationary);
        rotation_count = sum(stationary & (gyro_magnitudes > 5.0));
        mean_displacement = sqrt(dx^2 + dy^2 + dz^2);
        max_gyro = max(gyro_magnitudes);
        
        fprintf('窗口 %d: 位移=%.3fm, 最大角速度=%.1fdps, 静止点=%d, 旋转点=%d\n', ...
            window_count, mean_displacement, max_gyro, stationary_count, rotation_count);
    end
end