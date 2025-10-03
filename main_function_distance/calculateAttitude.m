% ������̬�ǶȺ���
function [pitch, roll] = calculateAttitude(ax, ay, az, accel_range)
    % ��ԭʼ����ת��ΪG��λ
    ax_g = (ax / 32768) * accel_range;
    ay_g = (ay / 32768) * accel_range;
    az_g = (az / 32768) * accel_range;
    
    % ���㸩����(pitch)�͹�ת��(roll)
    % ���ڴ�����z�������������෴����Ҫ��������
    pitch = atan2(-ax_g, sqrt(ay_g^2 + az_g^2)) * 180/pi;
    roll = atan2(-ay_g, -az_g) * 180/pi;
end