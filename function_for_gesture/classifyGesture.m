function gesture_type = classifyGesture(trajectory, velocity)
    % 基于轨迹特征进行手势分类
    
    % 计算轨迹特征
    features = extractGestureFeatures(trajectory, velocity);
    
    % 简单基于阈值的分类
    if features.linearity > 0.8 && features.duration < 2.0
        gesture_type = '直线手势';
    elseif features.circularity > 0.7
        gesture_type = '圆形手势';
    elseif features.energy_z > features.energy_xy * 2
        gesture_type = '垂直手势';
    else
        gesture_type = '未知手势';
    end
end

function features = extractGestureFeatures(trajectory, velocity)
    % 提取手势特征
    
    [~, N] = size(trajectory);
    
    features.duration = N * 0.01; % 假设100Hz采样
    
    % 轨迹长度
    total_distance = sum(sqrt(sum(diff(trajectory, 1, 2).^2, 1)));
    features.total_distance = total_distance;
    
    % 线性度（起点到终点的距离与总路径长度之比）
    direct_distance = norm(trajectory(:,end) - trajectory(:,1));
    features.linearity = direct_distance / (total_distance + eps);
    
    % 圆形度（基于轨迹的紧凑性）
    centroid = mean(trajectory, 2);
    distances = sqrt(sum((trajectory - centroid).^2, 1));
    features.circularity = mean(distances) / (std(distances) + eps);
    
    % 运动能量分布
    features.energy_xy = mean(sqrt(velocity(1,:).^2 + velocity(2,:).^2));
    features.energy_z = mean(abs(velocity(3,:)));
end