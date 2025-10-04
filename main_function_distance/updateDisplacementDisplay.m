function updateDisplacementDisplay(h_displacement, h_displacement_text, displacementX, displacementY, displacementZ, currentIndex, displacementAxes, window_start, window_count)
    % 设置当前坐标轴
    axes(displacementAxes);
    
    % 确保索引有效
    if window_start > currentIndex
        window_start = max(1, currentIndex - 50);
    end
    
    window_indices = window_start:min(currentIndex, length(displacementX));
    
    % 更新位移轨迹 - 将米转换为毫米
    if ~isempty(window_indices)
        set(h_displacement, 'XData', displacementX(window_indices) * 1000, ...  % 米→毫米
                            'YData', displacementY(window_indices) * 1000, ...  % 米→毫米
                            'ZData', displacementZ(window_indices) * 1000, ...  % 米→毫米
                            'Visible', 'on');
    end
    
    % 更新位移文本信息 - 将米转换为毫米
    if currentIndex > 0 && currentIndex <= length(displacementX)
        text_cell = {
            sprintf('位移: (%.2f, %.2f, %.2f) mm', ...
                    displacementX(currentIndex) * 1000, ...  % 米→毫米
                    displacementY(currentIndex) * 1000, ...  % 米→毫米
                    displacementZ(currentIndex) * 1000), ... % 米→毫米
            sprintf('窗口: %d', window_count)
        };
        set(h_displacement_text, 'String', text_cell, 'Visible', 'on');
    end
    
    % 刷新图形
    drawnow limitrate;
end