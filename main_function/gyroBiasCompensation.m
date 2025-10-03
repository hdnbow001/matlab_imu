% 零偏补偿函数
function [gyroX_final, gyroY_final, gyroZ_final, dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ] = ...
    gyroBiasCompensation(gyroX_int, gyroY_int, gyroZ_int, ...
    bias_gyroX, bias_gyroY, bias_gyroZ, ...
    dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ, alpha)
    
    % 应用静态零偏补偿
    gyroX_comp = double(gyroX_int) - bias_gyroX;
    gyroY_comp = double(gyroY_int) - bias_gyroY;
    gyroZ_comp = double(gyroZ_int) - bias_gyroZ;
    
    % 动态零偏估计
    % 检测传感器是否静止（简单阈值法）
    stationary_threshold = 50; % 阈值，可根据实际情况调整
    if abs(gyroX_comp) < stationary_threshold && ...
       abs(gyroY_comp) < stationary_threshold && ...
       abs(gyroZ_comp) < stationary_threshold
        
        % 使用低通滤波器更新零偏估计
        dynamic_bias_gyroX = (1 - alpha) * dynamic_bias_gyroX + alpha * gyroX_comp;
        dynamic_bias_gyroY = (1 - alpha) * dynamic_bias_gyroY + alpha * gyroY_comp;
        dynamic_bias_gyroZ = (1 - alpha) * dynamic_bias_gyroZ + alpha * gyroZ_comp;
    end
    
    % 应用动态零偏补偿
    gyroX_final = gyroX_comp - dynamic_bias_gyroX;
    gyroY_final = gyroY_comp - dynamic_bias_gyroY;
    gyroZ_final = gyroZ_comp - dynamic_bias_gyroZ;
end
