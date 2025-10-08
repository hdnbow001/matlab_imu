function [fusedPitch, fusedRoll, accel_weight] = adaptiveSensorFusion(...
        accPitch, accRoll, gyroX_dps, gyroY_dps, ...
        prevPitch, prevRoll, dt, i, convergence_samples, ...
        accel_magnitude, motion_threshold)
    
    % === 修正1：使用更合理的窗口大小（1秒，10个点）===
    window_size = 10; % 1秒窗口
    window_start = max(1, i - window_size + 1);
    current_window = accel_magnitude(window_start:i);
    accel_variance = var(current_window);
    
    % === 修正2：改进的权重分配策略 ===
    if i <= convergence_samples
        % 收敛阶段：平滑过渡，初始更信任加速度计
        convergence_factor = (i-1) / convergence_samples;
        accel_weight = 0.95 - 0.75 * convergence_factor; % 从0.95线性降到0.2
    else
        % 稳定阶段：基于运动状态，但保持足够的加速度计权重来修正漂移
        motion_level = min(1.0, accel_variance / motion_threshold);
        
        if motion_level < 0.3
            % 静止状态：较高加速度计权重修正漂移
            accel_weight = 0.3;
        elseif motion_level < 0.7
            % 轻度运动：平衡融合
            accel_weight = 0.2;
        else
            % 强烈运动：降低加速度计权重，但不能太低
            accel_weight = 0.1;
        end
    end
    
    gyro_weight = 1.0 - accel_weight;
    
    % === 修正3：陀螺仪积分 ===
    gyro_pitch = prevPitch + gyroX_dps * dt;
    gyro_roll = prevRoll + gyroY_dps * dt;
    
    % === 修正4：关键改进 - 角度连续性处理 ===
    % 处理加速度计角度与当前陀螺仪积分角度的一致性
    accPitch_adj = accPitch;
    accRoll_adj = accRoll;
    
    % 检查是否需要调整加速度计角度到合适的360°周期
    pitch_diff = accPitch - gyro_pitch;
    roll_diff = accRoll - gyro_roll;
    
    if pitch_diff > 180
        accPitch_adj = accPitch - 360;
    elseif pitch_diff < -180
        accPitch_adj = accPitch + 360;
    end
    
    if roll_diff > 180
        accRoll_adj = accRoll - 360;
    elseif roll_diff < -180
        accRoll_adj = accRoll + 360;
    end
    
    % === 修正5：互补滤波融合 ===
    fusedPitch = gyro_weight * gyro_pitch + accel_weight * accPitch_adj;
    fusedRoll = gyro_weight * gyro_roll + accel_weight * accRoll_adj;
    
    % === 修正6：防止角度发散的机制 ===
    % 如果融合角度与陀螺仪积分角度偏差过大，强制使用加速度计修正
    pitch_deviation = abs(fusedPitch - gyro_pitch);
    roll_deviation = abs(fusedRoll - gyro_roll);
    
    if pitch_deviation > 45 || roll_deviation > 45
        % 角度偏差过大，增加加速度计权重进行强制修正
        emergency_weight = min(0.5, accel_weight * 2);
        fusedPitch = (1 - emergency_weight) * gyro_pitch + emergency_weight * accPitch_adj;
        fusedRoll = (1 - emergency_weight) * gyro_roll + emergency_weight * accRoll_adj;
    end
    
    % === 修正7：最终角度范围处理 - 使用-180到180度范围 ===
    % 将角度限制在-180到180度范围内，避免0度边界问题
    if fusedPitch > 180
        fusedPitch = fusedPitch - 360;
    elseif fusedPitch < -180
        fusedPitch = fusedPitch + 360;
    end
    
    if fusedRoll > 180
        fusedRoll = fusedRoll - 360;
    elseif fusedRoll < -180
        fusedRoll = fusedRoll + 360;
    end
    
    % === 调试输出 ===
    if mod(i, 50) == 0
        fprintf('点 %d: Accel权重=%.2f, 运动水平=%.3f, Pitch: Acc=%.1f, Gyro=%.1f, Fused=%.1f\n', ...
            i, accel_weight, accel_variance/motion_threshold, accPitch, gyro_pitch, fusedPitch);
    end
end