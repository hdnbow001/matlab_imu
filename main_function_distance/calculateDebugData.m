% ����������ݺ���
function [linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = calculateDebugData(accelX, accelY, accelZ, pitchAngles, rollAngles, dt, accel_range)
    % �����������ٶȲο�ֵ (m/s?)
    G = 9.80665;
    
    % �����ٶ�����ת��ΪG��λ��Ȼ��ת��Ϊm/s?
    accelX_g = (accelX / 32768) * accel_range;
    accelY_g = (accelY / 32768) * accel_range;
    accelZ_g = (accelZ / 32768) * accel_range;
    
    accelX_mps2 = accelX_g * G;
    accelY_mps2 = accelY_g * G;
    accelZ_mps2 = accelZ_g * G;
    
    % ���Ƕ�ת��Ϊ����
    pitchRad = pitchAngles * pi/180;
    rollRad = rollAngles * pi/180;
    
    % ��ʼ�����Լ��ٶ�����
    linearAccelX = zeros(size(accelX_mps2));
    linearAccelY = zeros(size(accelY_mps2));
    linearAccelZ = zeros(size(accelZ_mps2));
    
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
    alpha = 0.95; % �˲�ϵ��
    % filteredAccelX = improvedHighPassFilterSimple(linearAccelX, alpha, dt);
    % filteredAccelY = improvedHighPassFilterSimple(linearAccelY, alpha, dt);
    % filteredAccelZ = improvedHighPassFilterSimple(linearAccelZ, alpha, dt);
    filteredAccelX = improvedHighPassFilter(linearAccelX, alpha, dt);
    filteredAccelY = improvedHighPassFilter(linearAccelY, alpha, dt);
    filteredAccelZ = improvedHighPassFilter(linearAccelZ, alpha, dt);
    
    % ��һ�λ��֣����ٶ� -> �ٶ�
    velX = cumtrapz(filteredAccelX) * dt;
    velY = cumtrapz(filteredAccelY) * dt;
    velZ = cumtrapz(filteredAccelZ) * dt;
    
    % Ӧ�ø�ͨ�˲���ȥ���ٶ�Ư��
    % velX = improvedHighPassFilterSimple(velX, alpha, dt);
    % velY = improvedHighPassFilterSimple(velY, alpha, dt);
    % velZ = improvedHighPassFilterSimple(velZ, alpha, dt);
    velX = improvedHighPassFilter(velX, alpha, dt);
    velY = improvedHighPassFilter(velY, alpha, dt);
    velZ = improvedHighPassFilter(velZ, alpha, dt);
    
    % �������Լ��ٶȺ��ٶ�
    linAccelX = filteredAccelX;
    linAccelY = filteredAccelY;
    linAccelZ = filteredAccelZ;
end