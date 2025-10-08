% function [dx, dy, dz, linAccelX_full, linAccelY_full, linAccelZ_full, velX_full, velY_full, velZ_full] = calculateContinuousDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range, current_index)
%     % 基于零速检测和速度复位的位移计算 - 修正旋转误判版本
%     % 移除定期复位，修正重力消除
% 
%     % 地球重力加速度参考值 (m/s²)
%     G = 9.80665;
% 
%     % 数据转换
%     accelX_g = double((accelX / 32768)) * accel_range;
%     accelY_g = double((accelY / 32768)) * accel_range;
%     accelZ_g = double((accelZ / 32768)) * accel_range;
% 
%     accelX_mps2 = accelX_g * G;
%     accelY_mps2 = accelY_g * G;
%     accelZ_mps2 = accelZ_g * G;
% 
%     gyroX_dps = (gyroX / 32768) * gyro_range;
%     gyroY_dps = (gyroY / 32768) * gyro_range;
%     gyroZ_dps = (gyroZ / 32768) * gyro_range;
% 
%     % 将角度转换为弧度
%     pitchRad = pitchAngles * pi/180;
%     rollRad = rollAngles * pi/180;
% 
%     n = length(accelX_mps2);
% 
%     % 使用持久变量
%     persistent prev_velX prev_velY prev_velZ prev_dx prev_dy prev_dz;
%     persistent bias_X bias_Y bias_Z;
%     persistent rotation_filtered_gyroX rotation_filtered_gyroY rotation_filtered_gyroZ;
% 
%     if isempty(prev_velX)
%         prev_velX = 0; prev_velY = 0; prev_velZ = 0;
%         prev_dx = 0; prev_dy = 0; prev_dz = 0;
%         bias_X = 0; bias_Y = 0; bias_Z = 0;
%         rotation_filtered_gyroX = 0; rotation_filtered_gyroY = 0; rotation_filtered_gyroZ = 0;
%     end
% 
%     % === 零值截断阈值 ===
%     ACCEL_DEADZONE = 0.02 * G;    % 加速度死区 (约0.2 m/s²)
%     VELOCITY_DEADZONE = 0.005;    % 速度死区 (0.005 m/s)
%     DISPLACEMENT_DEADZONE = 0.002; % 位移死区 (2 mm)
% 
%     % === 关键改进：更准确的重力消除 ===
%     linearAccelX = zeros(n,1);
%     linearAccelY = zeros(n,1);
%     linearAccelZ = zeros(n,1);
% 
%     for i = 1:n
%         % 使用旋转矩阵进行准确的重力消除
%         % 旋转顺序：Z-Y-X (偏航-俯仰-滚转)
%         cosPitch = cos(pitchRad(i));
%         sinPitch = sin(pitchRad(i));
%         cosRoll = cos(rollRad(i));
%         sinRoll = sin(rollRad(i));
% 
%         % 重力在设备坐标系中的分量
%         gx = -G * sinPitch;
%         gy = G * cosPitch * sinRoll;
%         gz = G * cosPitch * cosRoll;
% 
%         % 从测量加速度中减去重力分量
%         linearAccelX(i) = accelX_mps2(i) - gx;
%         linearAccelY(i) = accelY_mps2(i) - gy;
%         linearAccelZ(i) = accelZ_mps2(i) - gz;
% 
%         % 调试：检查重力消除效果
%         if mod(current_index, 100) == 0 && i == 1
%             fprintf('重力消除[点%d]: g=[%.3f,%.3f,%.3f], 消除后=[%.3f,%.3f,%.3f]\n', ...
%                 current_index, gx, gy, gz, linearAccelX(i), linearAccelY(i), linearAccelZ(i));
%         end
%     end
% 
%     % === 改进的零速检测 - 区分旋转和线性运动 ===
%     stationary = false(n, 1);
%     is_rotating = false(n, 1);
% 
%     for i = 1:n
%         % 计算加速度和角速度幅值
%         accel_mag = sqrt(accelX_mps2(i)^2 + accelY_mps2(i)^2 + accelZ_mps2(i)^2);
%         gyro_mag = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
% 
%         % 判断是否旋转（角速度较大）
%         is_rotating(i) = gyro_mag > 10.0; % 角速度大于10 dps视为旋转
% 
%         % 零速检测条件（排除旋转情况）
%         if is_rotating(i)
%             % 旋转时不判定为静止
%             stationary(i) = false;
%         else
%             % 静止条件：加速度接近重力且角速度很小
%             condition1 = abs(accel_mag - G) < 0.08 * G;
%             condition2 = gyro_mag < 2.0;
%             stationary(i) = condition1 && condition2;
%         end
%     end
% 
%     % === 零偏估计（只在高质量静止时更新）===
%     stationary_indices = find(stationary);
%     if length(stationary_indices) >= 8
%         current_bias_X = mean(linearAccelX(stationary_indices));
%         current_bias_Y = mean(linearAccelY(stationary_indices));
%         current_bias_Z = mean(linearAccelZ(stationary_indices));
% 
%         % 检查零偏是否合理
%         max_reasonable_bias = 0.15 * G;
%         if abs(current_bias_X) < max_reasonable_bias && abs(current_bias_Y) < max_reasonable_bias
%             alpha_bias = 0.1;
%             bias_X = alpha_bias * current_bias_X + (1 - alpha_bias) * bias_X;
%             bias_Y = alpha_bias * current_bias_Y + (1 - alpha_bias) * bias_Y;
%             bias_Z = alpha_bias * current_bias_Z + (1 - alpha_bias) * bias_Z;
%         end
%     end
% 
%     % 应用零偏补偿
%     linearAccelX_corrected = linearAccelX - bias_X;
%     linearAccelY_corrected = linearAccelY - bias_Y;
%     linearAccelZ_corrected = linearAccelZ - bias_Z;
% 
%     % === 关键改进：旋转运动检测和补偿 ===
%     % 当检测到旋转时，减弱线性加速度信号
%     for i = 1:n
%         if is_rotating(i)
%             % 旋转时大幅减弱线性加速度信号（但不是完全归零）
%             rotation_factor = 0.1; % 旋转时只保留10%的线性加速度
%             linearAccelX_corrected(i) = linearAccelX_corrected(i) * rotation_factor;
%             linearAccelY_corrected(i) = linearAccelY_corrected(i) * rotation_factor;
%             linearAccelZ_corrected(i) = linearAccelZ_corrected(i) * rotation_factor;
%         end
% 
%         % 在静止点强制加速度为零
%         if stationary(i)
%             linearAccelX_corrected(i) = 0;
%             linearAccelY_corrected(i) = 0;
%             linearAccelZ_corrected(i) = 0;
%         end
%     end
% 
%     % === 稳健滤波 ===
%     alpha_filter = 0.7;
%     filteredAccelX = zeros(n,1);
%     filteredAccelY = zeros(n,1);
%     filteredAccelZ = zeros(n,1);
% 
%     for i = 1:n
%         if i == 1
%             filteredAccelX(i) = linearAccelX_corrected(i);
%             filteredAccelY(i) = linearAccelY_corrected(i);
%             filteredAccelZ(i) = linearAccelZ_corrected(i);
%         else
%             filteredAccelX(i) = alpha_filter * filteredAccelX(i-1) + (1-alpha_filter) * linearAccelX_corrected(i);
%             filteredAccelY(i) = alpha_filter * filteredAccelY(i-1) + (1-alpha_filter) * linearAccelY_corrected(i);
%             filteredAccelZ(i) = alpha_filter * filteredAccelZ(i-1) + (1-alpha_filter) * linearAccelZ_corrected(i);
%         end
% 
%         % 滤波后应用零值截断
%         if abs(filteredAccelX(i)) < ACCEL_DEADZONE
%             filteredAccelX(i) = 0;
%         end
%         if abs(filteredAccelY(i)) < ACCEL_DEADZONE
%             filteredAccelY(i) = 0;
%         end
%         if abs(filteredAccelZ(i)) < ACCEL_DEADZONE
%             filteredAccelZ(i) = 0;
%         end
%     end
% 
%     % === 改进的积分策略 - 防止震荡 ===
%     velocityX = zeros(n,1);
%     velocityY = zeros(n,1);
%     velocityZ = zeros(n,1);
% 
%     displacementX_array = zeros(n,1);
%     displacementY_array = zeros(n,1);
%     displacementZ_array = zeros(n,1);
% 
%     % 速度阻尼系数（防止微小漂移）
%     velocity_damping = 0.995;
% 
%     for i = 1:n
%         % 速度积分
%         if i == 1
%             velocityX(i) = prev_velX + filteredAccelX(i) * dt;
%             velocityY(i) = prev_velY + filteredAccelY(i) * dt;
%             velocityZ(i) = prev_velZ + filteredAccelZ(i) * dt;
%         else
%             % 使用梯形积分
%             velocityX(i) = velocityX(i-1) + (filteredAccelX(i-1) + filteredAccelX(i)) * 0.5 * dt;
%             velocityY(i) = velocityY(i-1) + (filteredAccelY(i-1) + filteredAccelY(i)) * 0.5 * dt;
%             velocityZ(i) = velocityZ(i-1) + (filteredAccelZ(i-1) + filteredAccelZ(i)) * 0.5 * dt;
%         end
% 
%         % 应用轻微的速度阻尼（防止长期漂移）
%         velocityX(i) = velocityX(i) * velocity_damping;
%         velocityY(i) = velocityY(i) * velocity_damping;
%         velocityZ(i) = velocityZ(i) * velocity_damping;
% 
%         % === 关键：只在确认静止时复位速度（不移除定期复位）===
%         if stationary(i)
%             velocityX(i) = 0;
%             velocityY(i) = 0;
%             velocityZ(i) = 0;
%         end
% 
%         % 速度零值截断
%         if abs(velocityX(i)) < VELOCITY_DEADZONE
%             velocityX(i) = 0;
%         end
%         if abs(velocityY(i)) < VELOCITY_DEADZONE
%             velocityY(i) = 0;
%         end
%         if abs(velocityZ(i)) < VELOCITY_DEADZONE
%             velocityZ(i) = 0;
%         end
% 
%         % 位移积分
%         if i == 1
%             displacementX_array(i) = prev_dx + velocityX(i) * dt;
%             displacementY_array(i) = prev_dy + velocityY(i) * dt;
%             displacementZ_array(i) = prev_dz + velocityZ(i) * dt;
%         else
%             displacementX_array(i) = displacementX_array(i-1) + (velocityX(i-1) + velocityX(i)) * 0.5 * dt;
%             displacementY_array(i) = displacementY_array(i-1) + (velocityY(i-1) + velocityY(i)) * 0.5 * dt;
%             displacementZ_array(i) = displacementZ_array(i-1) + (velocityZ(i-1) + velocityZ(i)) * 0.5 * dt;
%         end
% 
%         % 位移零值截断
%         if abs(displacementX_array(i)) < DISPLACEMENT_DEADZONE
%             displacementX_array(i) = 0;
%         end
%         if abs(displacementY_array(i)) < DISPLACEMENT_DEADZONE
%             displacementY_array(i) = 0;
%         end
%         if abs(displacementZ_array(i)) < DISPLACEMENT_DEADZONE
%             displacementZ_array(i) = 0;
%         end
%     end
% 
%     % === 移除定期复位机制 ===
%     % 只更新持久状态，不移除定期复位
% 
%     % 更新持久状态
%     prev_velX = velocityX(end);
%     prev_velY = velocityY(end);
%     prev_velZ = velocityZ(end);
%     prev_dx = displacementX_array(end);
%     prev_dy = displacementY_array(end);
%     prev_dz = displacementZ_array(end);
% 
%     % === 返回结果 ===
%     dx = displacementX_array(end);
%     dy = displacementY_array(end);
%     dz = displacementZ_array(end);
% 
%     linAccelX_full = filteredAccelX;
%     linAccelY_full = filteredAccelY;
%     linAccelZ_full = filteredAccelZ;
%     velX_full = velocityX;
%     velY_full = velocityY;
%     velZ_full = velocityZ;
% 
%     % === 详细的调试输出 ===
%     if mod(current_index, 20) == 0
%         stationary_percent = sum(stationary) / n * 100;
%         rotating_percent = sum(is_rotating) / n * 100;
%         max_accel = max([abs(filteredAccelX); abs(filteredAccelY); abs(filteredAccelZ)]);
%         max_vel = max([abs(velocityX); abs(velocityY); abs(velocityZ)]);
%         max_disp = max([abs(displacementX_array); abs(displacementY_array); abs(displacementZ_array)]);
% 
%         fprintf('=== 位移计算 [点%d] ===\n', current_index);
%         fprintf('运动状态: 静止%.1f%%, 旋转%.1f%%, 线性%.1f%%\n', ...
%             stationary_percent, rotating_percent, 100-stationary_percent-rotating_percent);
%         fprintf('信号幅值: 加速度=%.3f, 速度=%.3f, 位移=%.3f\n', max_accel, max_vel, max_disp);
%         fprintf('最终位移: X=%.3f, Y=%.3f, Z=%.3f\n', dx, dy, dz);
%         fprintf('零偏补偿: X=%.4f, Y=%.4f, Z=%.4f\n', bias_X, bias_Y, bias_Z);
% 
%         % 检查旋转误判
%         if rotating_percent > 50 && max_accel > 0.5
%             fprintf('注意：检测到大量旋转运动，线性加速度可能受影响\n');
%         end
% 
%         % 检查积分稳定性
%         vel_drift = mean([abs(mean(velocityX(stationary))), abs(mean(velocityY(stationary))), abs(mean(velocityZ(stationary)))]);
%         if vel_drift > 0.01
%             fprintf('警告：静止时速度漂移=%.4f m/s\n', vel_drift);
%         end
%     end
% end

function [dx, dy, dz, linAccelX_full, linAccelY_full, linAccelZ_full, velX_full, velY_full, velZ_full] = calculateContinuousDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range, current_index)
    % 基于零速检测的位移计算 - 防止位移发散版本
    % 在现有函数框架内解决位移发散问题
    
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
    persistent prev_velX prev_velY prev_velZ prev_dx prev_dy prev_dz;
    persistent bias_X bias_Y bias_Z;
    persistent displacement_decay_counter;
    
    if isempty(prev_velX)
        prev_velX = 0; prev_velY = 0; prev_velZ = 0;
        prev_dx = 0; prev_dy = 0; prev_dz = 0;
        bias_X = 0; bias_Y = 0; bias_Z = 0;
        displacement_decay_counter = 0;
    end
    
    % === 零值截断阈值 ===
    ACCEL_DEADZONE = 0.02 * G;
    VELOCITY_DEADZONE = 0.005;
    DISPLACEMENT_DEADZONE = 0.002;
    
    % === 重力消除 ===
    linearAccelX = zeros(n,1);
    linearAccelY = zeros(n,1);
    linearAccelZ = zeros(n,1);
    
    for i = 1:n
        % 使用旋转矩阵进行重力消除
        cosPitch = cos(pitchRad(i));
        sinPitch = sin(pitchRad(i));
        cosRoll = cos(rollRad(i));
        sinRoll = sin(rollRad(i));
        
        gx = -G * sinPitch;
        gy = G * cosPitch * sinRoll;
        gz = G * cosPitch * cosRoll;
        
        linearAccelX(i) = accelX_mps2(i) - gx;
        linearAccelY(i) = accelY_mps2(i) - gy;
        linearAccelZ(i) = accelZ_mps2(i) - gz;
        
        % 加速度零值截断
        if abs(linearAccelX(i)) < ACCEL_DEADZONE
            linearAccelX(i) = 0;
        end
        if abs(linearAccelY(i)) < ACCEL_DEADZONE
            linearAccelY(i) = 0;
        end
        if abs(linearAccelZ(i)) < ACCEL_DEADZONE
            linearAccelZ(i) = 0;
        end
    end
    
    % === 零速检测 ===
    stationary = false(n, 1);
    is_rotating = false(n, 1);
    
    for i = 1:n
        accel_mag = sqrt(accelX_mps2(i)^2 + accelY_mps2(i)^2 + accelZ_mps2(i)^2);
        gyro_mag = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
        
        is_rotating(i) = gyro_mag > 10.0;
        
        if is_rotating(i)
            stationary(i) = false;
        else
            condition1 = abs(accel_mag - G) < 0.08 * G;
            condition2 = gyro_mag < 2.0;
            stationary(i) = condition1 && condition2;
        end
    end
    
    % === 零偏估计 ===
    stationary_indices = find(stationary);
    if length(stationary_indices) >= 8
        current_bias_X = mean(linearAccelX(stationary_indices));
        current_bias_Y = mean(linearAccelY(stationary_indices));
        current_bias_Z = mean(linearAccelZ(stationary_indices));
        
        max_reasonable_bias = 0.15 * G;
        if abs(current_bias_X) < max_reasonable_bias && abs(current_bias_Y) < max_reasonable_bias
            alpha_bias = 0.1;
            bias_X = alpha_bias * current_bias_X + (1 - alpha_bias) * bias_X;
            bias_Y = alpha_bias * current_bias_Y + (1 - alpha_bias) * bias_Y;
            bias_Z = alpha_bias * current_bias_Z + (1 - alpha_bias) * bias_Z;
        end
    end
    
    % 应用零偏补偿
    linearAccelX_corrected = linearAccelX - bias_X;
    linearAccelY_corrected = linearAccelY - bias_Y;
    linearAccelZ_corrected = linearAccelZ - bias_Z;
    
    % === 旋转运动补偿 ===
    for i = 1:n
        if is_rotating(i)
            rotation_factor = 0.1;
            linearAccelX_corrected(i) = linearAccelX_corrected(i) * rotation_factor;
            linearAccelY_corrected(i) = linearAccelY_corrected(i) * rotation_factor;
            linearAccelZ_corrected(i) = linearAccelZ_corrected(i) * rotation_factor;
        end
        
        if stationary(i)
            linearAccelX_corrected(i) = 0;
            linearAccelY_corrected(i) = 0;
            linearAccelZ_corrected(i) = 0;
        end
    end
    
    % === 滤波 ===
    alpha_filter = 0.7;
    filteredAccelX = zeros(n,1);
    filteredAccelY = zeros(n,1);
    filteredAccelZ = zeros(n,1);
    
    for i = 1:n
        if i == 1
            filteredAccelX(i) = linearAccelX_corrected(i);
            filteredAccelY(i) = linearAccelY_corrected(i);
            filteredAccelZ(i) = linearAccelZ_corrected(i);
        else
            filteredAccelX(i) = alpha_filter * filteredAccelX(i-1) + (1-alpha_filter) * linearAccelX_corrected(i);
            filteredAccelY(i) = alpha_filter * filteredAccelY(i-1) + (1-alpha_filter) * linearAccelY_corrected(i);
            filteredAccelZ(i) = alpha_filter * filteredAccelZ(i-1) + (1-alpha_filter) * linearAccelZ_corrected(i);
        end
        
        if abs(filteredAccelX(i)) < ACCEL_DEADZONE
            filteredAccelX(i) = 0;
        end
        if abs(filteredAccelY(i)) < ACCEL_DEADZONE
            filteredAccelY(i) = 0;
        end
        if abs(filteredAccelZ(i)) < ACCEL_DEADZONE
            filteredAccelZ(i) = 0;
        end
    end
    
    % === 速度计算 ===
    velocityX = zeros(n,1);
    velocityY = zeros(n,1);
    velocityZ = zeros(n,1);
    
    for i = 1:n
        if i == 1
            velocityX(i) = prev_velX + filteredAccelX(i) * dt;
            velocityY(i) = prev_velY + filteredAccelY(i) * dt;
            velocityZ(i) = prev_velZ + filteredAccelZ(i) * dt;
        else
            velocityX(i) = velocityX(i-1) + (filteredAccelX(i-1) + filteredAccelX(i)) * 0.5 * dt;
            velocityY(i) = velocityY(i-1) + (filteredAccelY(i-1) + filteredAccelY(i)) * 0.5 * dt;
            velocityZ(i) = velocityZ(i-1) + (filteredAccelZ(i-1) + filteredAccelZ(i)) * 0.5 * dt;
        end
        
        % 轻微速度阻尼
        velocity_damping = 0.998;
        velocityX(i) = velocityX(i) * velocity_damping;
        velocityY(i) = velocityY(i) * velocity_damping;
        velocityZ(i) = velocityZ(i) * velocity_damping;
        
        if stationary(i)
            velocityX(i) = 0;
            velocityY(i) = 0;
            velocityZ(i) = 0;
        end
        
        if abs(velocityX(i)) < VELOCITY_DEADZONE
            velocityX(i) = 0;
        end
        if abs(velocityY(i)) < VELOCITY_DEADZONE
            velocityY(i) = 0;
        end
        if abs(velocityZ(i)) < VELOCITY_DEADZONE
            velocityZ(i) = 0;
        end
    end
    
    % === 关键改进：防止位移发散的积分策略 ===
    displacementX_array = zeros(n,1);
    displacementY_array = zeros(n,1);
    displacementZ_array = zeros(n,1);
    
    % 位移积分控制参数
    max_velocity_for_displacement = 0.5; % 最大有效速度 (m/s)
    displacement_gain = 0.1; % 位移增益（降低位移增长）
    
    for i = 1:n
        if i == 1
            displacementX_array(i) = prev_dx;
            displacementY_array(i) = prev_dy;
            displacementZ_array(i) = prev_dz; % 修复：移除重复的赋值
        else
            % 方法1：限制有效速度范围
            effective_velX = sign(velocityX(i)) * min(abs(velocityX(i)), max_velocity_for_displacement);
            effective_velY = sign(velocityY(i)) * min(abs(velocityY(i)), max_velocity_for_displacement);
            effective_velZ = sign(velocityZ(i)) * min(abs(velocityZ(i)), max_velocity_for_displacement);
            
            % 方法2：应用位移增益和限制
            delta_dx = (effective_velX + velocityX(i-1)) * 0.5 * dt * displacement_gain;
            delta_dy = (effective_velY + velocityY(i-1)) * 0.5 * dt * displacement_gain;
            delta_dz = (effective_velZ + velocityZ(i-1)) * 0.5 * dt * displacement_gain;
            
            % 方法3：限制单步位移增量
            max_step_displacement = 0.02; % 单步最大位移2cm
            delta_dx = sign(delta_dx) * min(abs(delta_dx), max_step_displacement);
            delta_dy = sign(delta_dy) * min(abs(delta_dy), max_step_displacement);
            delta_dz = sign(delta_dz) * min(abs(delta_dz), max_step_displacement);
            
            displacementX_array(i) = displacementX_array(i-1) + delta_dx;
            displacementY_array(i) = displacementY_array(i-1) + delta_dy;
            displacementZ_array(i) = displacementZ_array(i-1) + delta_dz;
        end
        
        % 在静止点保持位移不变（而不是归零）
        if stationary(i) && i > 1
            displacementX_array(i) = displacementX_array(i-1);
            displacementY_array(i) = displacementY_array(i-1);
            displacementZ_array(i) = displacementZ_array(i-1);
        end
        
        % 位移零值截断
        if abs(displacementX_array(i)) < DISPLACEMENT_DEADZONE
            displacementX_array(i) = 0;
        end
        if abs(displacementY_array(i)) < DISPLACEMENT_DEADZONE
            displacementY_array(i) = 0;
        end
        if abs(displacementZ_array(i)) < DISPLACEMENT_DEADZONE
            displacementZ_array(i) = 0;
        end
    end
    
    % === 位移发散检测和修正（不移除持久变量）===
    max_reasonable_displacement = 3.0;
    current_max_displacement = max([abs(displacementX_array); abs(displacementY_array); abs(displacementZ_array)]);
    
    if current_max_displacement > max_reasonable_displacement
        % 渐进式衰减而不是完全复位
        decay_factor = 0.8;
        displacementX_array = displacementX_array * decay_factor;
        displacementY_array = displacementY_array * decay_factor;
        displacementZ_array = displacementZ_array * decay_factor;
        
        displacement_decay_counter = displacement_decay_counter + 1;
        
        if mod(current_index, 10) == 0
            fprintf('位移衰减应用: 最大位移=%.3f, 衰减系数=%.2f, 计数=%d\n', ...
                current_max_displacement, decay_factor, displacement_decay_counter);
        end
    end
    
    % === 更新持久状态 ===
    prev_velX = velocityX(end);
    prev_velY = velocityY(end);
    prev_velZ = velocityZ(end);
    prev_dx = displacementX_array(end);
    prev_dy = displacementY_array(end);
    prev_dz = displacementZ_array(end);
    
    % === 返回结果 ===
    dx = displacementX_array(end);
    dy = displacementY_array(end);
    dz = displacementZ_array(end);
    
    linAccelX_full = filteredAccelX;
    linAccelY_full = filteredAccelY;
    linAccelZ_full = filteredAccelZ;
    velX_full = velocityX;
    velY_full = velocityY;
    velZ_full = velocityZ;
    
    % === 调试输出 ===
    if mod(current_index, 20) == 0
        stationary_percent = sum(stationary) / n * 100;
        max_vel = max([abs(velocityX); abs(velocityY); abs(velocityZ)]);
        max_disp = max([abs(displacementX_array); abs(displacementY_array); abs(displacementZ_array)]);
        
        fprintf('位移状态 [点%d]: 速度max=%.3f, 位移max=%.3f\n', current_index, max_vel, max_disp);
        fprintf('最终位移: X=%.3f, Y=%.3f, Z=%.3f\n', dx, dy, dz);
        fprintf('静止点: %.1f%%, 位移衰减计数: %d\n', stationary_percent, displacement_decay_counter);
        
        % 检查位移合理性
        if max_vel < 0.1 && max_disp > 0.5
            fprintf('警告：速度很小但位移很大，可能存在积分误差\n');
        end
        
        if max_disp > 1.0
            fprintf('注意：位移较大，可能需要调整积分参数\n');
        end
    end
end