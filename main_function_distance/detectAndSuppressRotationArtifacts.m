function [linAccelX_out, linAccelY_out, linAccelZ_out] = detectAndSuppressRotationArtifacts(...
    linAccelX, linAccelY, linAccelZ, gyroX_dps, gyroY_dps, gyroZ_dps, window_count)
    
    % 检测纯旋转运动并抑制虚假位移
    
    n = length(linAccelX);
    linAccelX_out = linAccelX;
    linAccelY_out = linAccelY;
    linAccelZ_out = linAccelZ;
    
    % 计算角速度能量
    gyro_energy = gyroX_dps.^2 + gyroY_dps.^2 + gyroZ_dps.^2;
    
    % 计算线性加速度能量
    accel_energy = linAccelX.^2 + linAccelY.^2 + linAccelZ.^2;
    
    % 使用滑动窗口检测旋转主导的运动
    window_size = 5; % 5个采样点的窗口
    
    % 初始化旋转检测计数器
    rotation_detected_count = 0;
    
    for i = 1:n
        % 计算窗口
        start_idx = max(1, i - floor(window_size/2));
        end_idx = min(n, i + floor(window_size/2));
        window_indices = start_idx:end_idx;
        
        % 计算窗口内的平均角速度能量和加速度能量
        avg_gyro_energy = mean(gyro_energy(window_indices));
        avg_accel_energy = mean(accel_energy(window_indices));
        
        % 如果角速度能量远大于加速度能量，则可能是纯旋转
        rotation_ratio = avg_gyro_energy / (avg_accel_energy + 0.001); % 避免除零
        
        % 检测条件：高角速度 + 低线性加速度 = 可能是纯旋转
        if rotation_ratio > 8 && avg_gyro_energy > 10 % 经验阈值
            % 纯旋转：抑制线性加速度
            suppression_factor = 0.2; % 抑制到20%
            linAccelX_out(i) = linAccelX(i) * suppression_factor;
            linAccelY_out(i) = linAccelY(i) * suppression_factor;
            linAccelZ_out(i) = linAccelZ(i) * suppression_factor;
            
            rotation_detected_count = rotation_detected_count + 1;
            
            % 可选：输出调试信息
            if mod(i, 20) == 0 && window_count > 1
                fprintf('检测到旋转运动，窗口 %d 点 %d: 角速度能量=%.2f, 加速度能量=%.2f, 比例=%.2f\n', ...
                    window_count, i, avg_gyro_energy, avg_accel_energy, rotation_ratio);
            end
        end
    end
    
    % 输出旋转检测统计
    if rotation_detected_count > 0 && window_count > 1 && mod(window_count, 5) == 0
        fprintf('窗口 %d: 检测到 %d/%d 个旋转点 (%.1f%%)\n', ...
            window_count, rotation_detected_count, n, (rotation_detected_count/n)*100);
    end
end