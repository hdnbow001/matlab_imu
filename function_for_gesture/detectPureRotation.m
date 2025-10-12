function rotation_segments = detectPureRotation(gyro_data, active_indices)
    % 检测纯旋转运动段
    
    rotation_segments = [];
    segment_start = -1;
    min_segment_length = 5;
    
    for i = 1:length(active_indices)
        idx = active_indices(i);
        gyro_mag = norm(gyro_data(:, idx));
        
        % 旋转检测条件
        is_rotating = gyro_mag > 15; % 较低的阈值
        
        if is_rotating && segment_start == -1
            % 开始新的旋转段
            segment_start = idx;
        elseif ~is_rotating && segment_start ~= -1
            % 结束当前旋转段
            if (idx - segment_start) >= min_segment_length
                rotation_segments = [rotation_segments; segment_start, idx-1];
            end
            segment_start = -1;
        end
    end
    
    % 处理最后一个段
    if segment_start ~= -1
        if (active_indices(end) - segment_start) >= min_segment_length
            rotation_segments = [rotation_segments; segment_start, active_indices(end)];
        end
    end
end