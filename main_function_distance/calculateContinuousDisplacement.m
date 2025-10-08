% === 重构：基于速度复位和零速检测的位移计算 - 返回完整数组 ===
function [dx, dy, dz, linAccelX_full, linAccelY_full, linAccelZ_full, velX_full, velY_full, velZ_full] = calculateContinuousDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range, current_index)
    % 基于零速检测和速度复位的位移计算
    % 返回完整的中间结果数组，用于调试显示
    
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
    
    n = length(accelX_mps2);
    
    % 使用持久变量
    persistent cumulative_yaw prev_velX prev_velY prev_velZ prev_dx prev_dy prev_dz;
    persistent bias_X bias_Y bias_Z bias_count;
    
    if isempty(cumulative_yaw)
        cumulative_yaw = 0;
        prev_velX = 0; prev_velY = 0; prev_velZ = 0;
        prev_dx = 0; prev_dy = 0; prev_dz = 0;
        bias_X = 0; bias_Y = 0; bias_Z = 0;
        bias_count = 0;
    end
    
    % === 简化的重力消除（仅使用Z轴） ===
    linearAccelX = zeros(n,1);
    linearAccelY = zeros(n,1);
    linearAccelZ = zeros(n,1);
    
    for i = 1:n
        % 简化的重力消除 - 只考虑Z轴重力
        accel_total = sqrt(accelX_mps2(i)^2 + accelY_mps2(i)^2 + accelZ_mps2(i)^2);
        
        if abs(accel_total - G) < 0.2 * G
            % 接近静止状态，完全抑制加速度
            linearAccelX(i) = 0;
            linearAccelY(i) = 0;
            linearAccelZ(i) = 0;
        else
            % 简化的线性加速度计算
            linearAccelX(i) = accelX_mps2(i);
            linearAccelY(i) = accelY_mps2(i);
            linearAccelZ(i) = accelZ_mps2(i) - G; % 只从Z轴减去重力
        end
    end
    
    % === 改进的零速检测 ===
    stationary = detectStationaryPointsEnhanced(accelX_mps2, accelY_mps2, accelZ_mps2, gyroX_dps, gyroY_dps, gyroZ_dps, G);
    
    % === 零偏估计（只在静止时更新） ===
    stationary_indices = find(stationary);
    if length(stationary_indices) >= 5
        current_bias_X = mean(linearAccelX(stationary_indices));
        current_bias_Y = mean(linearAccelY(stationary_indices));
        current_bias_Z = mean(linearAccelZ(stationary_indices));
        
        % 平滑更新零偏
        alpha_bias = 0.05; % 很慢的更新
        bias_X = alpha_bias * current_bias_X + (1 - alpha_bias) * bias_X;
        bias_Y = alpha_bias * current_bias_Y + (1 - alpha_bias) * bias_Y;
        bias_Z = alpha_bias * current_bias_Z + (1 - alpha_bias) * bias_Z;
        
        bias_count = bias_count + 1;
    end
    
    % 应用零偏补偿
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
    
    % === 强滤波 ===
    alpha_strong = 0.95; % 强滤波系数
    filteredAccelX = zeros(n,1);
    filteredAccelY = zeros(n,1);
    filteredAccelZ = zeros(n,1);
    
    for i = 1:n
        if i == 1
            filteredAccelX(i) = linearAccelX_corrected(i);
            filteredAccelY(i) = linearAccelY_corrected(i);
            filteredAccelZ(i) = linearAccelZ_corrected(i);
        else
            filteredAccelX(i) = alpha_strong * filteredAccelX(i-1) + (1-alpha_strong) * linearAccelX_corrected(i);
            filteredAccelY(i) = alpha_strong * filteredAccelY(i-1) + (1-alpha_strong) * linearAccelY_corrected(i);
            filteredAccelZ(i) = alpha_strong * filteredAccelZ(i-1) + (1-alpha_strong) * linearAccelZ_corrected(i);
        end
        
        if stationary(i)
            filteredAccelX(i) = 0;
            filteredAccelY(i) = 0;
            filteredAccelZ(i) = 0;
        end
    end
    
    % === 积分计算（带速度复位） ===
    velocityX = zeros(n,1);
    velocityY = zeros(n,1);
    velocityZ = zeros(n,1);
    
    displacementX_array = zeros(n,1);
    displacementY_array = zeros(n,1);
    displacementZ_array = zeros(n,1);
    
    for i = 1:n
        % 速度积分
        if i == 1
            velocityX(i) = prev_velX + filteredAccelX(i) * dt;
            velocityY(i) = prev_velY + filteredAccelY(i) * dt;
            velocityZ(i) = prev_velZ + filteredAccelZ(i) * dt;
        else
            velocityX(i) = velocityX(i-1) + (filteredAccelX(i-1) + filteredAccelX(i)) * 0.5 * dt;
            velocityY(i) = velocityY(i-1) + (filteredAccelY(i-1) + filteredAccelY(i)) * 0.5 * dt;
            velocityZ(i) = velocityZ(i-1) + (filteredAccelZ(i-1) + filteredAccelZ(i)) * 0.5 * dt;
        end
        
        % === 关键：在静止点复位速度 ===
        if stationary(i)
            velocityX(i) = 0;
            velocityY(i) = 0;
            velocityZ(i) = 0;
        end
        
        % 位移积分
        if i == 1
            displacementX_array(i) = prev_dx + velocityX(i) * dt;
            displacementY_array(i) = prev_dy + velocityY(i) * dt;
            displacementZ_array(i) = prev_dz + velocityZ(i) * dt;
        else
            displacementX_array(i) = displacementX_array(i-1) + (velocityX(i-1) + velocityX(i)) * 0.5 * dt;
            displacementY_array(i) = displacementY_array(i-1) + (velocityY(i-1) + velocityY(i)) * 0.5 * dt;
            displacementZ_array(i) = displacementZ_array(i-1) + (velocityZ(i-1) + velocityZ(i)) * 0.5 * dt;
        end
    end
    
    % 更新持久状态
    prev_velX = velocityX(end);
    prev_velY = velocityY(end);
    prev_velZ = velocityZ(end);
    prev_dx = displacementX_array(end);
    prev_dy = displacementY_array(end);
    prev_dz = displacementZ_array(end);
    
    % === 返回完整的结果数组 ===
    % 位移结果（最后一个值）
    dx = displacementX_array(end);
    dy = displacementY_array(end);
    dz = displacementZ_array(end);
    
    % 完整的中间结果数组
    linAccelX_full = filteredAccelX;
    linAccelY_full = filteredAccelY;
    linAccelZ_full = filteredAccelZ;
    velX_full = velocityX;
    velY_full = velocityY;
    velZ_full = velocityZ;
    
    % 调试输出
    %if mod(current_index, 50) == 0
    if mod(current_index, 10) == 0
        stationary_percent = sum(stationary) / n * 100;
        fprintf('连续位移[点%d]: X=%.3f, Y=%.3f, Z=%.3f | 静止点: %.1f%% | 零偏: X=%.4f, Y=%.4f\n', ...
            current_index, dx, dy, dz, stationary_percent, bias_X, bias_Y);
    end
end
