function features = extractGestureFeatures(trajectory, velocity, dt)
    % 提取手势特征，添加dt参数
    
    [~, N] = size(trajectory);
    
    features.duration = (N - 1) * dt; % 使用实际时间
    
    % 轨迹长度
    diff_traj = diff(trajectory, 1, 2);
    total_distance = sum(sqrt(sum(diff_traj.^2, 1)));
    features.total_distance = total_distance;
    
    % 线性度
    direct_distance = norm(trajectory(:,end) - trajectory(:,1));
    features.linearity = direct_distance / (total_distance + eps);
    
    % 改进的圆形度
    features.circularity = calculateCircularity(trajectory);
    
    % 轨迹是否闭合
    features.is_closed = direct_distance < (total_distance * 0.2);
    
    % 运动能量分布
    features.energy_xy = mean(sqrt(velocity(1,:).^2 + velocity(2,:).^2));
    features.energy_z = mean(abs(velocity(3,:)));
    
    % 速度变化特征
    speed = sqrt(sum(velocity.^2, 1));
    features.max_speed = max(speed);
    features.speed_variance = var(speed);
end