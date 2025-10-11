% === 修复后的辅助函数 ===
function gesture_active = detectGestureActivity(accel_data, gyro_data, threshold)
    % 基于运动能量检测手势活动
    
    [~, N] = size(accel_data);
    gesture_active = false(1, N);
    
    % 确保输入为double类型
    accel_data = double(accel_data);
    gyro_data = double(gyro_data);
    threshold = double(threshold);
    
    % 计算运动能量（加速度和角速度的模值）
    accel_magnitude = sqrt(sum(accel_data.^2, 1));
    gyro_magnitude = sqrt(sum(gyro_data.^2, 1));
    
    % 归一化运动能量
    motion_energy = 0.7 * (accel_magnitude - 9.81) + 0.3 * gyro_magnitude;
    
    % 滑动窗口平滑
    window_size = 5;
    if N >= window_size
        smoothed_energy = movmean(motion_energy, window_size);
    else
        smoothed_energy = motion_energy;
    end
    
    % 阈值检测
    activity_threshold = threshold;
    gesture_active = smoothed_energy > activity_threshold;
    
    % 形态学操作去除噪声（需要图像处理工具箱）
    if exist('bwareaopen', 'file')
        min_gesture_duration = 5; % 最小手势持续时间（采样点数）
        gesture_active = bwareaopen(gesture_active, min_gesture_duration);
    end
end