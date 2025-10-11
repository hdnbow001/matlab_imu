% === 新增：手势轨迹显示更新函数 ===
function updateGestureDisplay(traj_plot, points_plot, start_plot, end_plot, ...
                             traj_x, traj_y, traj_z, active_mask, current_index, subplot_ax)
    % 更新手势轨迹显示
    
    % 提取有效轨迹点（手势活动期间）
    valid_indices = find(active_mask(1:current_index));
    
    if length(valid_indices) < 2
        % 没有足够的手势数据，清除显示
        set(traj_plot, 'XData', 0, 'YData', 0, 'ZData', 0);
        set(points_plot, 'XData', 0, 'YData', 0, 'ZData', 0);
        set(start_plot, 'XData', 0, 'YData', 0, 'ZData', 0);
        set(end_plot, 'XData', 0, 'YData', 0, 'ZData', 0);
        title(subplot_ax, '实时手势轨迹 - 等待手势...');
        return;
    end
    
    % 更新轨迹线
    set(traj_plot, 'XData', traj_x(valid_indices), ...
                   'YData', traj_y(valid_indices), ...
                   'ZData', traj_z(valid_indices));
    
    % 更新轨迹点
    set(points_plot, 'XData', traj_x(valid_indices), ...
                     'YData', traj_y(valid_indices), ...
                     'ZData', traj_z(valid_indices));
    
    % 更新起点和终点标记
    set(start_plot, 'XData', traj_x(valid_indices(1)), ...
                    'YData', traj_y(valid_indices(1)), ...
                    'ZData', traj_z(valid_indices(1)));
    set(end_plot, 'XData', traj_x(valid_indices(end)), ...
                  'YData', traj_y(valid_indices(end)), ...
                  'ZData', traj_z(valid_indices(end)));
    
    % 更新标题显示轨迹信息
    trajectory_length = length(valid_indices);
    title(subplot_ax, sprintf('实时手势轨迹 (%d点)', trajectory_length));
    
    % 自动调整坐标轴范围
    if length(valid_indices) >= 2
        margin = 0.1; % 10厘米边距
        x_limits = [min(traj_x(valid_indices))-margin, max(traj_x(valid_indices))+margin];
        y_limits = [min(traj_y(valid_indices))-margin, max(traj_y(valid_indices))+margin];
        z_limits = [min(traj_z(valid_indices))-margin, max(traj_z(valid_indices))+margin];
        
        % 确保所有轴有相同的范围以保持比例
        all_limits = [x_limits, y_limits, z_limits];
        axis_range = [min(all_limits), max(all_limits)];
        
        xlim(subplot_ax, axis_range);
        ylim(subplot_ax, axis_range);
        zlim(subplot_ax, axis_range);
    end
    
    drawnow;
end