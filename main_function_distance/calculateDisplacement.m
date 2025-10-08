function [dx, dy, dz, linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = calculateDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range)
    % 窗口位移计算函数 - 添加旋转运动补偿

    % 地球重力加速度参考值 (m/s²)
    G = 9.80665;

    % 数据转换
    accelX_g = double(accelX) / 32768 * accel_range;
    accelY_g = double(accelY) / 32768 * accel_range;
    accelZ_g = double(accelZ) / 32768 * accel_range;

    accelX_mps2 = accelX_g * G;
    accelY_mps2 = accelY_g * G;
    accelZ_mps2 = accelZ_g * G;

    gyroX_dps = double(gyroX) / 32768 * gyro_range;
    gyroY_dps = double(gyroY) / 32768 * gyro_range;
    gyroZ_dps = double(gyroZ) / 32768 * gyro_range;

    % 将角度转换为弧度
    pitchRad = pitchAngles * pi/180;
    rollRad = rollAngles * pi/180;

    n = length(accelX_mps2);

    % === 改进的旋转检测 ===
    is_rotating = false(n, 1);
    rotation_intensity = zeros(n, 1);  % 旋转强度指标

    for i = 1:n
        % 计算角速度幅值
        gyro_mag = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);

        % 计算角速度变化率（角加速度）
        if i > 1
            gyro_change = abs(gyro_mag - sqrt(gyroX_dps(i-1)^2 + gyroY_dps(i-1)^2 + gyroZ_dps(i-1)^2)) / dt;
        else
            gyro_change = 0;
        end

        % 综合旋转强度指标
        rotation_intensity(i) = gyro_mag * (1 + gyro_change * 0.1);

        % 动态旋转检测阈值
        rotation_threshold = 15.0;  % 基础阈值 15 dps

        % 如果角速度很大或者角加速度很大，认为是旋转运动
        is_rotating(i) = (gyro_mag > rotation_threshold) || (gyro_change > 50);
    end

    % === 使用已验证的重力消除方法 ===
    linearAccelX = zeros(n, 1);
    linearAccelY = zeros(n, 1);
    linearAccelZ = zeros(n, 1);

    for i = 1:n
        % 使用已验证的重力消除公式
        cosPitch = cos(pitchRad(i));
        sinPitch = sin(pitchRad(i));
        cosRoll = cos(rollRad(i));
        sinRoll = sin(rollRad(i));

        % 重力在设备坐标系中的分量
        gx = -G * sinPitch;
        gy = G * cosPitch * sinRoll;
        gz = G * cosPitch * cosRoll;

        % 从测量加速度中减去重力分量
        linearAccelX(i) = accelX_mps2(i) - gx;
        linearAccelY(i) = accelY_mps2(i) - gy;
        linearAccelZ(i) = accelZ_mps2(i) - gz;
    end

    % === 零速检测 ===
    stationary = false(n, 1);
    for i = 1:n
        % 计算加速度和角速度幅值
        accel_mag = sqrt(accelX_mps2(i)^2 + accelY_mps2(i)^2 + accelZ_mps2(i)^2);
        gyro_mag = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);

        % 零速检测条件
        condition1 = abs(accel_mag - G) < 0.08 * G;
        condition2 = gyro_mag < 2.0;
        stationary(i) = condition1 && condition2;
    end

    % === 零偏估计 ===
    stationary_indices = find(stationary);
    if length(stationary_indices) >= max(3, n * 0.2)
        bias_X = mean(linearAccelX(stationary_indices));
        bias_Y = mean(linearAccelY(stationary_indices));
        bias_Z = mean(linearAccelZ(stationary_indices));

        max_reasonable_bias = 0.15 * G;
        if abs(bias_X) > max_reasonable_bias || abs(bias_Y) > max_reasonable_bias
            bias_X = median(linearAccelX);
            bias_Y = median(linearAccelY);
            bias_Z = median(linearAccelZ);
        end
    else
        bias_X = median(linearAccelX);
        bias_Y = median(linearAccelY);
        bias_Z = median(linearAccelZ);
    end

    % 应用零偏补偿
    linearAccelX_corrected = linearAccelX - bias_X;
    linearAccelY_corrected = linearAccelY - bias_Y;
    linearAccelZ_corrected = linearAccelZ - bias_Z;

    % === 关键改进：基于旋转强度的自适应权重补偿 ===
    for i = 1:n
        if is_rotating(i)
            % 基于旋转强度的动态权重
            if rotation_intensity(i) < 25  % 轻微旋转
                rotation_factor = 0.7;  % 保留70%的线性加速度
            elseif rotation_intensity(i) < 50  % 中等旋转
                rotation_factor = 0.4;  % 保留40%的线性加速度
            else  % 强烈旋转
                rotation_factor = 0.1;  % 只保留10%的线性加速度
            end

            % 应用旋转补偿
            linearAccelX_corrected(i) = linearAccelX_corrected(i) * rotation_factor;
            linearAccelY_corrected(i) = linearAccelY_corrected(i) * rotation_factor;
            linearAccelZ_corrected(i) = linearAccelZ_corrected(i) * rotation_factor;

            % 调试输出旋转补偿信息
            if mod(i, 10) == 0
                fprintf('旋转补偿[点%d]: 强度=%.1f, 因子=%.1f, 补偿前=(%.3f,%.3f,%.3f), 补偿后=(%.3f,%.3f,%.3f)\n', ...
                    i, rotation_intensity(i), rotation_factor, ...
                    linearAccelX(i), linearAccelY(i), linearAccelZ(i), ...
                    linearAccelX_corrected(i), linearAccelY_corrected(i), linearAccelZ_corrected(i));
            end
        end

        % 在静止点强制加速度为零
        if stationary(i)
            linearAccelX_corrected(i) = 0;
            linearAccelY_corrected(i) = 0;
            linearAccelZ_corrected(i) = 0;
        end
    end

    % === 稳健滤波和死区处理 ===
    ACCEL_DEADZONE = 0.02 * G;
    VELOCITY_DEADZONE = 0.005;
    DISPLACEMENT_DEADZONE = 0.002;

    alpha_filter = 0.7;
    filteredAccelX = zeros(n, 1);
    filteredAccelY = zeros(n, 1);
    filteredAccelZ = zeros(n, 1);

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

        % 滤波后应用零值截断
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

    % === 改进的积分策略 ===
    velocityX = zeros(n, 1);
    velocityY = zeros(n, 1);
    velocityZ = zeros(n, 1);

    displacementX_array = zeros(n, 1);
    displacementY_array = zeros(n, 1);
    displacementZ_array = zeros(n, 1);

    velocity_damping = 0.995;

    for i = 1:n
        % 速度积分
        if i == 1
            velocityX(i) = filteredAccelX(i) * dt;
            velocityY(i) = filteredAccelY(i) * dt;
            velocityZ(i) = filteredAccelZ(i) * dt;
        else
            velocityX(i) = velocityX(i-1) + (filteredAccelX(i-1) + filteredAccelX(i)) * 0.5 * dt;
            velocityY(i) = velocityY(i-1) + (filteredAccelY(i-1) + filteredAccelY(i)) * 0.5 * dt;
            velocityZ(i) = velocityZ(i-1) + (filteredAccelZ(i-1) + filteredAccelZ(i)) * 0.5 * dt;
        end

        % 应用速度阻尼
        velocityX(i) = velocityX(i) * velocity_damping;
        velocityY(i) = velocityY(i) * velocity_damping;
        velocityZ(i) = velocityZ(i) * velocity_damping;

        % 在静止点强制速度为零
        if stationary(i)
            velocityX(i) = 0;
            velocityY(i) = 0;
            velocityZ(i) = 0;
        end

        % 速度零值截断
        if abs(velocityX(i)) < VELOCITY_DEADZONE
            velocityX(i) = 0;
        end
        if abs(velocityY(i)) < VELOCITY_DEADZONE
            velocityY(i) = 0;
        end
        if abs(velocityZ(i)) < VELOCITY_DEADZONE
            velocityZ(i) = 0;
        end

        % 位移积分
        if i == 1
            displacementX_array(i) = velocityX(i) * dt;
            displacementY_array(i) = velocityY(i) * dt;
            displacementZ_array(i) = velocityZ(i) * dt;
        else
            displacementX_array(i) = displacementX_array(i-1) + (velocityX(i-1) + velocityX(i)) * 0.5 * dt;
            displacementY_array(i) = displacementY_array(i-1) + (velocityY(i-1) + velocityY(i)) * 0.5 * dt;
            displacementZ_array(i) = displacementZ_array(i-1) + (velocityZ(i-1) + velocityZ(i)) * 0.5 * dt;
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

    % 返回结果
    linAccelX = filteredAccelX;
    linAccelY = filteredAccelY;
    linAccelZ = filteredAccelZ;
    velX = velocityX;
    velY = velocityY;
    velZ = velocityZ;

    dx = displacementX_array(end);
    dy = displacementY_array(end);
    dz = displacementZ_array(end);

    % 调试输出
    rotating_percent = sum(is_rotating) / n * 100;
    fprintf('窗口统计: 点数=%d, 旋转=%.1f%%, 位移=(%.6f, %.6f, %.6f)\n', ...
        n, rotating_percent, dx, dy, dz);
end


