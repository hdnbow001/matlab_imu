function processed_trajectory = postProcessTrajectory_(raw_trajectory, active_indices)
    % 轨迹后处理以减少积分误差
    
    [dim, N] = size(raw_trajectory);
    processed_trajectory = raw_trajectory;
    
    if length(active_indices) < 2
        return;
    end
    
    % 简单的去漂移处理：减去线性趋势
    for d = 1:dim
        trajectory_1d = raw_trajectory(d, active_indices);
        
        % 计算线性趋势
        x = 1:length(trajectory_1d);
        p = polyfit(x, trajectory_1d, 1);
        trend = polyval(p, x);
        
        % 减去趋势
        processed_trajectory(d, active_indices) = trajectory_1d - trend;
    end
    
    % 轨迹平滑（简单移动平均）
    smooth_window = 3;
    for d = 1:dim
        trajectory_1d = processed_trajectory(d, active_indices);
        if length(trajectory_1d) > smooth_window
            smoothed = movmean(trajectory_1d, smooth_window);
            processed_trajectory(d, active_indices) = smoothed;
        end
    end
end