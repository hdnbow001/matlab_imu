% ������̬��ʾ����
function updateAttitudeDisplay(attitudeQuiver, attitudeText, attitudeSphere, ax, ay, az, gx, gy, gz, attitudeAxes, accel_range, gyro_range, pitch, roll, yaw)
    % ��ԭʼ����ת��ΪG��λ
    ax_g = (ax / 32768) * accel_range;
    ay_g = (ay / 32768) * accel_range;
    az_g = (az / 32768) * accel_range;
    
    % ��һ�����ٶ�����
    normAccel = sqrt(ax_g^2 + ay_g^2 + az_g^2);
    if normAccel > 0
        ux = ax_g / normAccel;
        uy = ay_g / normAccel;
        uz = az_g / normAccel;
    else
        ux = 0; uy = 0; uz = 0;
    end
    
    % ���õ�ǰ������
    axes(attitudeAxes);
    
    % ���¼��ٶ�ʸ��
    set(attitudeQuiver, 'UData', ux, 'VData', uy, 'WData', uz);
    
    % �����ı����� - ʹ��sprintf���ɴ����е��ı�
    angleText = sprintf('������: %.2f��\n��ת��: %.2f��\nƫ����: %.2f��', pitch, roll, yaw);
    set(attitudeText, 'String', angleText);
    
    % ȷ�����������ͼ��Ԫ�ر��ֿɼ�
    set(attitudeSphere, 'Visible', 'on');
    
    % ˢ��ͼ��
    drawnow limitrate;
end