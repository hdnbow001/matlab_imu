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
    
    % ========== 改进的重力补偿和旋转处理 ==========
    
    % 使用持久变量保持旋转状态
    persistent cumulative_yaw prev_gyroX prev_gyroY prev_gyroZ;
    
    if isempty(cumulative_yaw) || window_count == 1
        cumulative_yaw = 0;
        prev_gyroX = 0;
        prev_gyroY = 0;
        prev_gyroZ = 0;
    end
    
    % 预处理：分析运动特征
    gyro_magnitudes = sqrt(gyroX_dps.^2 + gyroY_dps.^2 + gyroZ_dps.^2);
    mean_gyro = mean(gyro_magnitudes);
    max_gyro = max(gyro_magnitudes);
    
    % 窗口级旋转检测
    if mean_gyro > 20.0 || max_gyro > 50.0
        % fprintf('窗口 %d: 检测到强烈旋转(平均=%.1fdps, 最大=%.1fdps)，跳过位移计算\n', ...
        %         window_count, mean_gyro, max_gyro);
        dx = 0; dy = 0; dz = 0;
        linAccelX = zeros(n, 1);
        linAccelY = zeros(n, 1);
        linAccelZ = zeros(n, 1);
        velX = zeros(n, 1);
        velY = zeros(n, 1);
        velZ = zeros(n, 1);
        return;
    end
    
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
            yaw_change = (gyroZ_dps(i) + prev_gyroZ) * 0.5 * dt * pi/180;
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
        
        % ========== 点级旋转检测 ==========
        
        % 计算角速度幅度
        current_gyro = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
        current_accel = norm(accel_sensor);
        
        % 检测纯旋转运动
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
        
        % 更新前一次陀螺仪值
        prev_gyroX = gyroX_dps(i);
        prev_gyroY = gyroY_dps(i);
        prev_gyroZ = gyroZ_dps(i);
    end
    
    % ========== 增强的零速度检测 ==========
    
    stationary_detailed = detectStationaryPoints(accelX_mps2, accelY_mps2, accelZ_mps2, ...
                                               gyroX_dps, gyroY_dps, gyroZ_dps);
    
    % 合并旋转检测和静止检测
    for i = 1:n
        if stationary_detailed(i)
            stationary(i) = true;
            linearAccelX(i) = 0;
            linearAccelY(i) = 0;
            linearAccelZ(i) = 0;
        end
    end
    
    % ========== 改进的偏置消除 ==========
    
    % 使用静止点计算偏置
    stationary_indices = find(stationary);
    if length(stationary_indices) >= 3
        bias_X = mean(linearAccelX(stationary_indices));
        bias_Y = mean(linearAccelY(stationary_indices));
        bias_Z = mean(linearAccelZ(stationary_indices));
    else
        % 如果没有足够静止点，使用前几个点
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
    
    % ========== 滤波策略 ==========
    
    % 使用持久变量保持滤波器状态
    persistent prev_window_count_filter prev_filteredAccelX prev_filteredAccelY prev_filteredAccelZ;
    
    if isempty(prev_window_count_filter) || prev_window_count_filter ~= window_count
        prev_filteredAccelX = 0;
        prev_filteredAccelY = 0;
        prev_filteredAccelZ = 0;
        prev_window_count_filter = window_count;
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
    
    % ========== 改进的积分计算 ==========
    
    % 第一次积分：加速度 -> 速度（使用梯形积分法）
    velocityX = integrateAcceleration(filteredAccelX, dt, stationary);
    velocityY = integrateAcceleration(filteredAccelY, dt, stationary);
    velocityZ = integrateAcceleration(filteredAccelZ, dt, stationary);
    
    % 第二次积分：速度 -> 位移
    dx_array = integrateVelocity(velocityX, dt);
    dy_array = integrateVelocity(velocityY, dt);
    dz_array = integrateVelocity(velocityZ, dt);
    
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
        pos_accel = sum(filteredAccelX > 0.1);
        neg_accel = sum(filteredAccelX < -0.1);
        stationary_count = sum(stationary);
        mean_displacement = sqrt(dx^2 + dy^2 + dz^2);
        
        % fprintf('窗口 %d: 位移=%.3fm, 正加速度点=%d, 负加速度点=%d, 静止点=%d\n', ...
        %     window_count, mean_displacement, pos_accel, neg_accel, stationary_count);
    end
end

% ========== 辅助函数 ==========

function stationary = detectStationaryPoints(accelX, accelY, accelZ, gyroX, gyroY, gyroZ)
    % 严格的静止点检测
    n = length(accelX);
    stationary = false(n, 1);
    G = 9.80665;
    
    for i = 1:n
        accel_mag = sqrt(accelX(i)^2 + accelY(i)^2 + accelZ(i)^2);
        gyro_mag = sqrt(gyroX(i)^2 + gyroY(i)^2 + gyroZ(i)^2);
        
        % 严格的静止条件
        condition1 = abs(accel_mag - G) < 0.08 * G;  % 接近重力
        condition2 = gyro_mag < 1.0;                 % 几乎没有旋转
        
        % 检查加速度变化
        if i > 1
            accel_change = sqrt((accelX(i)-accelX(i-1))^2 + ...
                               (accelY(i)-accelY(i-1))^2 + ...
                               (accelZ(i)-accelZ(i-1))^2);
            condition3 = accel_change < 0.1 * G;
        else
            condition3 = true;
        end
        
        stationary(i) = condition1 && condition2 && condition3;
    end
    
    % 形态学清理
    min_duration = 3; % 至少3个连续点
    stationary = morphologicalClean(stationary, min_duration);
end

function cleaned = morphologicalClean(stationary, min_duration)
    % 形态学清理：去除短于min_duration的区间
    n = length(stationary);
    cleaned = stationary;
    
    i = 1;
    while i <= n
        if stationary(i)
            % 找到连续静止段的结束
            j = i;
            while j < n && stationary(j+1)
                j = j + 1;
            end
            
            % 如果段太短，标记为非静止
            if (j - i + 1) < min_duration
                cleaned(i:j) = false;
            end
            
            i = j + 1;
        else
            i = i + 1;
        end
    end
end

function velocity = integrateAcceleration(acceleration, dt, stationary)
    % 正确的加速度积分
    n = length(acceleration);
    velocity = zeros(n, 1);
    
    current_vel = 0;
    for i = 1:n
        % 梯形积分
        if i == 1
            delta_v = acceleration(i) * dt;
        else
            delta_v = (acceleration(i-1) + acceleration(i)) * 0.5 * dt;
        end
        
        current_vel = current_vel + delta_v;
        
        % 在静止点重置速度
        if stationary(i)
            current_vel = 0;
        end
        
        velocity(i) = current_vel;
    end
end

function displacement = integrateVelocity(velocity, dt)
    % 速度积分
    n = length(velocity);
    displacement = zeros(n, 1);
    
    current_disp = 0;
    for i = 1:n
        % 梯形积分
        if i == 1
            delta_d = velocity(i) * dt;
        else
            delta_d = (velocity(i-1) + velocity(i)) * 0.5 * dt;
        end
        
        current_disp = current_disp + delta_d;
        displacement(i) = current_disp;
    end
end