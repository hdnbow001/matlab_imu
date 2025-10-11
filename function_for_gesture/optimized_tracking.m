% 使用滑动窗口减少计算量
function optimized_tracking()
    % 只处理最近的手势数据，而不是全部历史数据
    window_size = 50; % 50个采样点（0.5秒）
    
    % 在缓冲区中处理数据
    if current_index > window_size
        process_indices = (current_index - window_size + 1):current_index;
    else
        process_indices = 1:current_index;
    end
end