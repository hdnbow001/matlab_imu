% =========================================================================
% 辅助函数：查找旋转区间
% =========================================================================
function segments = findRotationSegments(is_rotating)
    % 找到连续的旋转区间
    
    n = length(is_rotating);
    segments = [];
    
    i = 1;
    while i <= n
        if is_rotating(i)
            start_idx = i;
            % 找到旋转结束点
            while i <= n && is_rotating(i)
                i = i + 1;
            end
            end_idx = i - 1;
            
            % 只记录长度超过阈值的区间
            if (end_idx - start_idx) >= 2
                segments = [segments; start_idx, end_idx];
            end
        else
            i = i + 1;
        end
    end
    
    % 如果没有找到旋转区间，返回空矩阵
    if isempty(segments)
        segments = zeros(0, 2);
    end
end