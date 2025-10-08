function cleaned = morphologicalCleanEnhanced(stationary, min_duration)
    % 形态学清理：去除短于min_duration的孤立区间
    % 输入：
    %   stationary - 逻辑数组，true表示静止状态
    %   min_duration - 最小持续时间（采样点数）
    % 输出：
    %   cleaned - 清理后的逻辑数组
    
    n = length(stationary);
    cleaned = stationary;
    
    i = 1;
    while i <= n
        if stationary(i)
            % 找到连续静止段的结束位置
            j = i;
            while j < n && stationary(j+1)
                j = j + 1;
            end
            
            % 如果段长度太短，标记为非静止
            if (j - i + 1) < min_duration
                cleaned(i:j) = false;
            end
            
            i = j + 1;
        else
            i = i + 1;
        end
    end
end