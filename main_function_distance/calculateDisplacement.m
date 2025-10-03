% ����λ�ƺ��� - �Ľ��汾�������������ٶ�Ӱ������ٶȼ��
function [dx, dy, dz] = calculateDisplacement(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, pitchAngles, rollAngles, dt, accel_range, gyro_range, window_count)
    % �����������ٶȲο�ֵ (m/s?)
    G = 9.80665;
    
    % �����ٶ�����ת��ΪG��λ��Ȼ��ת��Ϊm/s?
    accelX_g = (accelX / 32768) * accel_range;
    accelY_g = (accelY / 32768) * accel_range;
    accelZ_g = (accelZ / 32768) * accel_range;
    
    accelX_mps2 = accelX_g * G;
    accelY_mps2 = accelY_g * G;
    accelZ_mps2 = accelZ_g * G;
    
    % ������������ת��Ϊ��/��
    gyroX_dps = (gyroX / 32768) * gyro_range;
    gyroY_dps = (gyroY / 32768) * gyro_range;
    gyroZ_dps = (gyroZ / 32768) * gyro_range;
    
    % ���Ƕ�ת��Ϊ����
    pitchRad = pitchAngles * pi/180;
    rollRad = rollAngles * pi/180;
    
    % ��ʼ�����Լ��ٶ�����
    linearAccelX = zeros(size(accelX_mps2));
    linearAccelY = zeros(size(accelY_mps2));
    linearAccelZ = zeros(size(accelZ_mps2));
    
    % ���ٶȼ���־
    stationary = false(size(accelX_mps2));
    
    % ��ÿ�������㴦��
    for i = 1:length(accelX_mps2)
        % ������ת���󣨴���������ϵ������������ϵ��
        R_x = [1, 0, 0;
               0, cos(rollRad(i)), -sin(rollRad(i));
               0, sin(rollRad(i)), cos(rollRad(i))];
           
        R_y = [cos(pitchRad(i)), 0, sin(pitchRad(i));
               0, 1, 0;
               -sin(pitchRad(i)), 0, cos(pitchRad(i))];
           
        % �����ת����
        R_ws = R_x * R_y; % �ȸ������ת
        
        % ��������ϵ�е��������� [0; 0; -G] (Z������)
        gravity_world = [0; 0; -G];
        
        % ������������ת������������ϵ
        gravity_sensor = R_ws * gravity_world;
        
        % �Ӵ����������м�ȥ��������
        linearAccelSensor = [accelX_mps2(i); accelY_mps2(i); accelZ_mps2(i)] - gravity_sensor;
        
        % �����Լ��ٶȴӴ���������ϵ��ת����������ϵ
        linearAccelWorld = R_ws' * linearAccelSensor;
        
        % �洢���Լ��ٶ�
        linearAccelX(i) = linearAccelWorld(1);
        linearAccelY(i) = linearAccelWorld(2);
        linearAccelZ(i) = linearAccelWorld(3);
        
        % ���ٶȼ��
        accelNorm = sqrt(accelX_mps2(i)^2 + accelY_mps2(i)^2 + accelZ_mps2(i)^2);
        gyroNorm = sqrt(gyroX_dps(i)^2 + gyroY_dps(i)^2 + gyroZ_dps(i)^2);
        
        % ����Ƿ�ֹ�����ٶ�ģ�ӽ�G�����ٶ�ģ�ӽ�0��
        if abs(accelNorm - G) < 0.2 && gyroNorm < 2
            stationary(i) = true;
            % ����Ǿ�ֹ�㣬�����Լ��ٶ�����
            linearAccelX(i) = 0;
            linearAccelY(i) = 0;
            linearAccelZ(i) = 0;
        end
    end
    
    % ��ÿ�����ڿ�ʼʱ����ȥ��ʼƫ�ã�ǰ�������ƽ��ֵ��
    if length(linearAccelX) > 5
        initial_bias_X = mean(linearAccelX(1:5));
        initial_bias_Y = mean(linearAccelY(1:5));
        initial_bias_Z = mean(linearAccelZ(1:5));
    else
        initial_bias_X = mean(linearAccelX);
        initial_bias_Y = mean(linearAccelY);
        initial_bias_Z = mean(linearAccelZ);
    end
    
    linearAccelX = linearAccelX - initial_bias_X;
    linearAccelY = linearAccelY - initial_bias_Y;
    linearAccelZ = linearAccelZ - initial_bias_Z;
    
    % Ӧ�ø�ͨ�˲���ȥ��ֱ�������͵�Ƶ����
    % ʹ�ó־ñ����������˲���״̬����ÿ����������
    persistent prev_window_count prev_filteredAccelX prev_filteredAccelY prev_filteredAccelZ;
    
    % ����Ƿ����µĴ���
    if isempty(prev_window_count) || prev_window_count ~= window_count
        % �´��ڣ������˲���״̬
        prev_filteredAccelX = 0;
        prev_filteredAccelY = 0;
        prev_filteredAccelZ = 0;
        prev_window_count = window_count;
    end
    
    % Ӧ�ø�ͨ�˲���
    alpha = 0.95; % �˲�ϵ��
    [filteredAccelX, prev_filteredAccelX] = improvedHighPassFilter(linearAccelX, alpha, dt, prev_filteredAccelX);
    [filteredAccelY, prev_filteredAccelY] = improvedHighPassFilter(linearAccelY, alpha, dt, prev_filteredAccelY);
    [filteredAccelZ, prev_filteredAccelZ] = improvedHighPassFilter(linearAccelZ, alpha, dt, prev_filteredAccelZ);
    
    % ˫�ػ��ּ���λ��
    % ��һ�λ��֣����ٶ� -> �ٶ�
    velocityX = cumtrapz(filteredAccelX) * dt;
    velocityY = cumtrapz(filteredAccelY) * dt;
    velocityZ = cumtrapz(filteredAccelZ) * dt;
    
    % �ھ�ֹ�������ٶ�
    for i = 1:length(velocityX)
        if stationary(i)
            velocityX(i:end) = velocityX(i:end) - velocityX(i);
            velocityY(i:end) = velocityY(i:end) - velocityY(i);
            velocityZ(i:end) = velocityZ(i:end) - velocityZ(i);
        end
    end
    
    % Ӧ�ø�ͨ�˲���ȥ���ٶ�Ư��
    [velocityX, ~] = improvedHighPassFilter(velocityX, alpha, dt, 0);
    [velocityY, ~] = improvedHighPassFilter(velocityY, alpha, dt, 0);
    [velocityZ, ~] = improvedHighPassFilter(velocityZ, alpha, dt, 0);
    
    % �ڶ��λ��֣��ٶ� -> λ��
    dx = cumtrapz(velocityX) * dt;
    dy = cumtrapz(velocityY) * dt;
    dz = cumtrapz(velocityZ) * dt;
    
    % �ھ�ֹ������λ��
    for i = 1:length(dx)
        if stationary(i)
            dx(i:end) = dx(i:end) - dx(i);
            dy(i:end) = dy(i:end) - dy(i);
            dz(i:end) = dz(i:end) - dz(i);
        end
    end
    
    % Ӧ�ø�ͨ�˲���ȥ��λ��Ư��
    [dx, ~] = improvedHighPassFilter(dx, alpha, dt, 0);
    [dy, ~] = improvedHighPassFilter(dy, alpha, dt, 0);
    [dz, ~] = improvedHighPassFilter(dz, alpha, dt, 0);
    
    % �������һ��λ��ֵ
    dx = dx(end);
    dy = dy(end);
    dz = dz(end);
end