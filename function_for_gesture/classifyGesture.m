function gesture_type = classifyGesture(trajectory, velocity, dt)
    % 基于轨迹特征进行手势分类
    % 输入添加dt参数
    
    if size(trajectory, 2) < 10
        gesture_type = '数据不足';
        return;
    end
    
    % 计算轨迹特征
    features = extractGestureFeatures(trajectory, velocity, dt);
    
    % 改进的分类逻辑
    if features.linearity > 0.85 && features.duration < 1.5
        gesture_type = '直线手势';
    elseif features.circularity > 0.6 && features.is_closed
        gesture_type = '圆形手势';
    elseif features.energy_z > features.energy_xy * 1.5
        gesture_type = '垂直手势';
    elseif features.total_distance < 0.1
        gesture_type = '静止';
    else
        gesture_type = '复杂手势';
    end
end

