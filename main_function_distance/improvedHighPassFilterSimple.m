% 简化的改进高通滤波器函数
function [filteredData, new_prev] = improvedHighPassFilterSimple(data, alpha, dt, prev)
    % 初始化滤波后的数据数组
    filteredData = zeros(size(data));
    
    % 计算滤波器系数
    RC = 1 / (2 * pi * 0.1); % 截止频率0.1Hz
    alpha_val = dt / (RC + dt);
    
    % 处理第一个数据点
    if ~isempty(data)
        filteredData(1) = alpha_val * (prev + data(1));
    end
    
    % 对后续数据点应用改进的高通滤波器
    for i = 2:length(data)
        filteredData(i) = alpha_val * (filteredData(i-1) + data(i) - data(i-1));
    end
    
    % 更新前一个值
    if ~isempty(data)
        new_prev = filteredData(end);
    else
        new_prev = prev;
    end
end