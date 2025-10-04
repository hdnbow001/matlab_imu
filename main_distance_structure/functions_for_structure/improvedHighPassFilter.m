% 标准高通滤波器实现
function [filteredData, new_prev] = improvedHighPassFilter(data, prev,dt)
    if nargin < 3
        prev = 0;
    end
    
    % 计算滤波器系数（固定截止频率0.1Hz）
    RC = 1 / (2 * pi * 0.1); % 截止频率0.1Hz
    alpha_val = dt / (RC + dt);
    
    filteredData = zeros(size(data));
    
    if ~isempty(data)
        % 处理第一个数据点
        filteredData(1) = alpha_val * (prev + data(1));
        
        % 对后续数据点应用改进的高通滤波器
        for i = 2:length(data)
            filteredData(i) = alpha_val * (filteredData(i-1) + data(i) - data(i-1));
        end
        
        new_prev = filteredData(end);
    else
        new_prev = prev;
        filteredData = [];
    end
end