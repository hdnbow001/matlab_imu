% ��ƫ��������
function [gyroX_final, gyroY_final, gyroZ_final, dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ] = ...
    gyroBiasCompensation(gyroX_int, gyroY_int, gyroZ_int, ...
    bias_gyroX, bias_gyroY, bias_gyroZ, ...
    dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ, alpha)
    
    % Ӧ�þ�̬��ƫ����
    gyroX_comp = double(gyroX_int) - bias_gyroX;
    gyroY_comp = double(gyroY_int) - bias_gyroY;
    gyroZ_comp = double(gyroZ_int) - bias_gyroZ;
    
    % ��̬��ƫ����
    % ��⴫�����Ƿ�ֹ������ֵ����
    stationary_threshold = 50; % ��ֵ���ɸ���ʵ���������
    if abs(gyroX_comp) < stationary_threshold && ...
       abs(gyroY_comp) < stationary_threshold && ...
       abs(gyroZ_comp) < stationary_threshold
        
        % ʹ�õ�ͨ�˲���������ƫ����
        dynamic_bias_gyroX = (1 - alpha) * dynamic_bias_gyroX + alpha * gyroX_comp;
        dynamic_bias_gyroY = (1 - alpha) * dynamic_bias_gyroY + alpha * gyroY_comp;
        dynamic_bias_gyroZ = (1 - alpha) * dynamic_bias_gyroZ + alpha * gyroZ_comp;
    end
    
    % Ӧ�ö�̬��ƫ����
    gyroX_final = gyroX_comp - dynamic_bias_gyroX;
    gyroY_final = gyroY_comp - dynamic_bias_gyroY;
    gyroZ_final = gyroZ_comp - dynamic_bias_gyroZ;
end
