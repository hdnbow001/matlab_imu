% ������̬��ʾ����
function updateAttitudeDisplay(attitudeQuiver, attitudeText, ax, ay, az, gx, gy, gz)
    % �����������ٶȲο�ֵ (m/s?)
    G = 9.80665;
    
    % ���㸩����(pitch)�͹�ת��(roll) - �Ӽ��ٶȼ�
    % ���ڴ�����z�������������෴����Ҫ��������
    pitch_acc = atan2(-ax, sqrt(ay^2 + az^2)) * 180/pi;
    roll_acc = atan2(-ay, -az) * 180/pi;
    
    % ���Ƕ�ת��Ϊ0-360�ȷ�Χ
    pitch_acc = mod(pitch_acc, 360);
    roll_acc = mod(roll_acc, 360);
    
    % �򻯵�ƫ����(yaw)���� - �������ǻ��֣��ǳ��򻯣�
    % ע�⣺����һ���ǳ��򻯵ķ�����ʵ��Ӧ������Ҫ�����ӵĴ������ں��㷨
    persistent yaw_angle;
    if isempty(yaw_angle)
        yaw_angle = 0;
    end
    
    % �򵥵Ļ��ּ���ƫ����
    dt = 0.2; % �������������5Hz�����ʣ�
    yaw_angle = yaw_angle + gz * dt; % ʹ��Z������������
    
    % ��ƫ����������0-360�ȷ�Χ��
    yaw_angle = mod(yaw_angle, 360);
    
    % ��һ�����ٶ�����
    normAccel = sqrt(ax^2 + ay^2 + az^2);
    if normAccel > 0
        ux = ax / normAccel;
        uy = ay / normAccel;
        uz = az / normAccel;
    else
        ux = 0; uy = 0; uz = 0;
    end
    
    % ���¼��ٶ�ʸ��
    set(attitudeQuiver, 'UData', ux, 'VData', uy, 'WData', uz);
    
    % ���½Ƕ���Ϣ�ı�
    set(attitudeText, 'String', sprintf('������: %.2f��\n��ת��: %.2f��\nƫ����: %.2f��', ...
        pitch_acc, roll_acc, yaw_angle));
end