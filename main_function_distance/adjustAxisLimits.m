function adjustAxisLimits(ax, timeData, xData, yData, zData)
    % 调整坐标轴范围以自适应数据显示
    % 参数:
    %   ax - 坐标轴句柄
    %   timeData - 时间数据
    %   xData, yData, zData - 三个方向的数据
    
    if length(timeData) > 1
        % 计算当前数据的范围
        data_min = min([min(xData), min(yData), min(zData)]);
        data_max = max([max(xData), max(yData), max(zData)]);
        time_min = min(timeData);
        time_max = max(timeData);
        
        % 设置坐标轴范围，添加10%的边距
        if data_max > data_min
            data_margin = (data_max - data_min) * 0.1;
            y_limits = [data_min - data_margin, data_max + data_margin];
        else
            % 如果数据范围很小，使用默认范围
            y_limits = [data_min - 0.1, data_max + 0.1];
        end
        
        time_margin = (time_max - time_min) * 0.1;
        x_limits = [time_min - time_margin, time_max + time_margin];
        
        % 设置坐标轴范围
        xlim(ax, x_limits);
        ylim(ax, y_limits);
    end
end