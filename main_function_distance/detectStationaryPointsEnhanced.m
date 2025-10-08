function stationary = detectStationaryPointsEnhanced(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, G)
    n = length(accelX);
    stationary = false(n, 1);
    
    % 滑动窗口参数
    window_size = 7; % 增大窗口
    
    for i = 1:n
        % 计算滑动窗口
        start_idx = max(1, i - floor(window_size/2));
        end_idx = min(n, i + floor(window_size/2));
        window_indices = start_idx:end_idx;
        
        % 窗口内的统计量
        accel_mag_window = sqrt(accelX(window_indices).^2 + accelY(window_indices).^2 + accelZ(window_indices).^2);
        gyro_mag_window = sqrt(gyroX(window_indices).^2 + gyroY(window_indices).^2 + gyroZ(window_indices).^2);
        
        accel_var = var(accel_mag_window);
        gyro_mean = mean(gyro_mag_window);
        gyro_var = var(gyro_mag_window);
        
        % === 更严格的静止条件 ===
        condition1 = accel_var < 0.02 * G;      % 加速度方差
        condition2 = gyro_mean < 2.0;           % 角速度均值
        condition3 = gyro_var < 1.0;            % 新增：角速度方差
        condition4 = abs(mean(accel_mag_window) - G) < 0.08 * G; % 接近重力
        
        % 检查加速度变化率
        if i > 1
            accel_change = sqrt((accelX(i)-accelX(i-1))^2 + (accelY(i)-accelY(i-1))^2 + (accelZ(i)-accelZ(i-1))^2);
            condition5 = accel_change < 0.08 * G;
        else
            condition5 = true;
        end
        
        % 需要满足大部分条件
        conditions = [condition1, condition2, condition3, condition4, condition5];
        stationary(i) = sum(conditions) >= 4; % 5个条件中至少满足4个
    end
    
    % 加强形态学处理
    stationary = morphologicalCleanEnhanced(stationary, 5); % 增大窗口
end