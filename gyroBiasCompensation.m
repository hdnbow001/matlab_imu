function [gyroX_final, gyroY_final, gyroZ_final, dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ] = ...
    gyroBiasCompensation(gyroX, gyroY, gyroZ, bias_gyroX, bias_gyroY, bias_gyroZ, ...
    dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ, alpha)
    
    % ¾²Ì¬ÁãÆ«²¹³¥
    gyroX_static = gyroX - bias_gyroX;
    gyroY_static = gyroY - bias_gyroY;
    gyroZ_static = gyroZ - bias_gyroZ;
    
    % ¶¯Ì¬ÁãÆ«¹À¼Æ
    dynamic_bias_gyroX = alpha * gyroX_static + (1 - alpha) * dynamic_bias_gyroX;
    dynamic_bias_gyroY = alpha * gyroY_static + (1 - alpha) * dynamic_bias_gyroY;
    dynamic_bias_gyroZ = alpha * gyroZ_static + (1 - alpha) * dynamic_bias_gyroZ;
    
    % ×îÖÕ²¹³¥
    gyroX_final = gyroX_static - dynamic_bias_gyroX;
    gyroY_final = gyroY_static - dynamic_bias_gyroY;
    gyroZ_final = gyroZ_static - dynamic_bias_gyroZ;
end