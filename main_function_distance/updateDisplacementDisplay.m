% 更新位移显示函数
function updateDisplacementDisplay(h_displacement, h_displacement_text, displacementX, displacementY, displacementZ, currentIndex, displacementAxes, window_start, window_count)
    % 设置当前坐标轴
    axes(displacementAxes);
    
    % 确保索引有效
    if window_start > currentIndex
        window_start = max(1, currentIndex - 50);
    end
    
    window_indices = window_start:min(currentIndex, length(displacementX));
    
    % 更新位移轨迹（只显示当前5秒窗口内的数据）
    if ~isempty(window_indices)
        set(h_displacement, 'XData', displacementX(window_indices), ...
                            'YData', displacementY(window_indices), ...
                            'ZData', displacementZ(window_indices), ...
                            'Visible', 'on');
    end
    
    % 更新位移文本信息
    if currentIndex > 0 && currentIndex <= length(displacementX)
        text_str = {sprintf('位移: (%.2f, %.2f, %.2f) m', ...
                    displacementX(currentIndex), displacementY(currentIndex), displacementZ(currentIndex)), ...
                    sprintf('窗口: %d', window_count)};
        set(h_displacement_text, 'String', text_str, 'Visible', 'on');
    end
    
    % 刷新图形
    drawnow limitrate;
end
