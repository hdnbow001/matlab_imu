function processed_trajectory = enhancedPostProcessTrajectory(raw_trajectory, active_indices, velocity)
    % 增强的轨迹后处理，解决漂移问题
    
    [dim, N] = size(raw_trajectory);
    processed_trajectory = raw_trajectory;
    
    if length(active_indices) < 2
        return;
    end
    
    % 1. 强力的去漂移处理
    for d = 1:dim
        trajectory_1d = raw_trajectory(d, active_indices);
        
        % 使用高阶多项式拟合去除趋势
        x = 1:length(trajectory_1d);
        if length(x) >= 4
            p = polyfit(x, trajectory_1d, 3); % 三次多项式
            trend = polyval(p, x);
            processed_trajectory(d, active_indices) = trajectory_1d - trend;
        else
            % 数据点太少，使用线性去趋势
            p = polyfit(x, trajectory_1d, 1);
            trend = polyval(p, x);
            processed_trajectory(d, active_indices) = trajectory_1d - trend;
        end
    end
    
    % 2. 基于速度的自适应平滑
    for d = 1:dim
        trajectory_1d = processed_trajectory(d, active_indices);
        velocity_1d = abs(velocity(d, active_indices));
        
        % 根据速度动态调整平滑窗口
        avg_velocity = mean(velocity_1d);
        if avg_velocity > 0.5
            smooth_window = 2; % 高速运动，少平滑
        else
            smooth_window = 5; % 低速运动，多平滑
        end
        
        if length(trajectory_1d) > smooth_window
            smoothed = movmean(trajectory_1d, smooth_window);
            processed_trajectory(d, active_indices) = smoothed;
        end
    end
    
    % 3. 强制轨迹从原点开始
    start_offset = processed_trajectory(:, active_indices(1));
    processed_trajectory(:, active_indices) = processed_trajectory(:, active_indices) - start_offset;
end