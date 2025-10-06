% =========================================================================
% 辅助函数：自适应高通滤波
% =========================================================================
function filtered = adaptiveHighPassFilter(data, stationary, dt, alpha)
    % 自适应高通滤波器
    
    n = length(data);
    filtered = zeros(size(data));
    
    prev_filtered = 0;
    
    for i = 1:n
        if i == 1
            filtered(i) = (1 - alpha) * data(i);
        else
            % 根据运动状态调整滤波强度
            if stationary(i)
                current_alpha = min(alpha * 1.2, 0.99); % 静止时更强滤波
            else
                current_alpha = alpha;
            end
            
            filtered(i) = current_alpha * prev_filtered + (1 - current_alpha) * (data(i) - data(i-1));
        end
        
        prev_filtered = filtered(i);
    end
end