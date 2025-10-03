function main()
    % ��������
    fs = 10;   % ������ 10Hz
    T = 100;   % ʱ�� (100��)
    n = round(T*fs);  % ��������� (1000��)
    %%s = serial('COM7'); % �������ڶ���-��UIH��������
    s = serial('COM3'); % �������ڶ���-��E5��������
    set(s, 'BaudRate', 115200); % ���ò�����
    
    % ����������
    accel_range = 2; % ��2G
    gyro_range = 250; % ��250dps
    window_size = 5 * fs; % 5�봰�ڴ�С (50��������)
    
    flag = false;
    try
        fopen(s);
        flag = true;
        disp('���ڴ򿪳ɹ�');
    catch
        error('���ڴ�ʧ��');
    end

    if flag
        % ��ʼ�����ݴ洢
        maxPoints = n; % ����������Ϊ1000
        
        % GYRO�������ݣ�ԭʼ�Ͳ�����
        gyroXData = zeros(1, maxPoints, 'int16');
        gyroYData = zeros(1, maxPoints, 'int16');
        gyroZData = zeros(1, maxPoints, 'int16');
        gyroXCompensated = zeros(1, maxPoints);
        gyroYCompensated = zeros(1, maxPoints);
        gyroZCompensated = zeros(1, maxPoints);
        
        % ACCEL��������
        accelXData = zeros(1, maxPoints, 'int16');
        accelYData = zeros(1, maxPoints, 'int16');
        accelZData = zeros(1, maxPoints, 'int16');
        
        % λ������
        displacementX = zeros(1, maxPoints);
        displacementY = zeros(1, maxPoints);
        displacementZ = zeros(1, maxPoints);
        
        % ��̬�Ƕ�����
        pitchAngles = zeros(1, maxPoints);
        rollAngles = zeros(1, maxPoints);
        yawAngles = zeros(1, maxPoints); % ����ƫ���Ǵ洢
        
        % ��̬��ƫ���Ʋ���
        alpha = 0.001; % �˲�ϵ����ԽСԽƽ������ӦԽ����
        dynamic_bias_gyroX = 0;
        dynamic_bias_gyroY = 0;
        dynamic_bias_gyroZ = 0;
        
        % ��̬��ƫУ׼����
        calibrationSamples = 200; % У׼��������
        bias_gyroX = 0;
        bias_gyroY = 0;
        bias_gyroZ = 0;
        
        % �����˲�����
        compAlpha = 0.98; % �����˲�ϵ��
        fusedPitch = 0; % �ںϺ�ĸ�����
        fusedRoll = 0;  % �ںϺ�Ĺ�ת��
        fusedYaw = 0;   % �ںϺ��ƫ����
        
        % ���ٶȼ�����
        stationaryThreshold = [0.05, 2]; % [���ٶȷ�����ֵ, ���ٶȾ�ֵ��ֵ]
        
        % ����ͼ�δ��� - 1x2����
        fig = figure('Position', [50, 50, 1400, 600], 'Name', 'IMU��������̬��λ����ʾ');
        
        % ��̬��ʾ��ͼ
        attitude_subplot = subplot(1, 2, 1);
        % ��ʼ����̬��ʾ
        [attitudeQuiver, attitudeText, attitudeSphere, attitudeAxes] = initAttitudeDisplay(attitude_subplot);
        
        % λ����ʾ��ͼ
        displacement_subplot = subplot(1, 2, 2);
        % ��ʼ��λ����ʾ
        [h_displacement, h_displacement_text, displacementAxes] = initDisplacementDisplay(displacement_subplot);
        
        % ��������ͼ��
        debugFig = figure('Position', [100, 100, 1200, 800], 'Name', '���������ݵ���');
        subplot(3,1,1);
        h_accel_x = plot(NaN, NaN, 'r-');
        hold on;
        h_accel_y = plot(NaN, NaN, 'g-');
        h_accel_z = plot(NaN, NaN, 'b-');
        title('���Լ��ٶ� (m/s?)');
        legend('X', 'Y', 'Z');
        grid on;
        
        subplot(3,1,2);
        h_velocity_x = plot(NaN, NaN, 'r-');
        hold on;
        h_velocity_y = plot(NaN, NaN, 'g-');
        h_velocity_z = plot(NaN, NaN, 'b-');
        title('�ٶ� (m/s)');
        legend('X', 'Y', 'Z');
        grid on;
        
        subplot(3,1,3);
        h_displacement_x = plot(NaN, NaN, 'r-');
        hold on;
        h_displacement_y = plot(NaN, NaN, 'g-');
        h_displacement_z = plot(NaN, NaN, 'b-');
        title('λ�� (m)');
        legend('X', 'Y', 'Z');
        grid on;
        
        % ֡ͬ��
        syncBytes = [170, 85]; % AA 55
        syncFound = false;
        
        % λ�Ƽ������
        dt = 1/fs; % �������
        current_window_start = 1; % ��ǰ������ʼ����
        window_count = 1; % ���ڼ�����
        
        % ��ʾ����
        fprintf('������\tGYRO_X(HEX)\tGYRO_X(dps)\tGYRO_Y(HEX)\tGYRO_Y(dps)\tGYRO_Z(HEX)\tGYRO_Z(dps)\tACCEL_X(HEX)\tACCEL_X(G)\tACCEL_Y(HEX)\tACCEL_Y(G)\tACCEL_Z(HEX)\tACCEL_Z(G)\n');
        fprintf('------\t-----------\t-----------\t-----------\t-----------\t-----------\t-----------\t------------\t----------\t------------\t----------\t------------\t----------\n');
        
        % ��ƫУ׼�׶�
        disp('��ʼ��ƫУ׼���뱣�ִ�������ֹ...');
        gyroX_calib = zeros(1, calibrationSamples);
        gyroY_calib = zeros(1, calibrationSamples);
        gyroZ_calib = zeros(1, calibrationSamples);
        
        for calib_i = 1:calibrationSamples
            % ��ȡ�������ݰ� (17�ֽ�)
            dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
            
            % ��ȡԭʼHEXֵ
            gyroX_hex = sprintf('%02X%02X', dataPacket(5), dataPacket(6));
            gyroY_hex = sprintf('%02X%02X', dataPacket(7), dataPacket(8));
            gyroZ_hex = sprintf('%02X%02X', dataPacket(9), dataPacket(10));
            
            % ת��Ϊint16�з�������
            gyroX_calib(calib_i) = double(typecast(uint16(hex2dec(gyroX_hex)), 'int16'));
            gyroY_calib(calib_i) = double(typecast(uint16(hex2dec(gyroY_hex)), 'int16'));
            gyroZ_calib(calib_i) = double(typecast(uint16(hex2dec(gyroZ_hex)), 'int16'));
            
            % �����һ֡��ͬ���ֽ�
            nextBytes = fread(s, 2, 'uint8');
            if ~(length(nextBytes) == 2 && isequal(nextBytes', syncBytes))
                % ����ͬ��
                syncFound = false;
                while ~syncFound
                    byte1 = fread(s, 1, 'uint8');
                    if ~isempty(byte1) && byte1 == syncBytes(1)
                        byte2 = fread(s, 1, 'uint8');
                        if ~isempty(byte2) && byte2 == syncBytes(2)
                            syncFound = true;
                            disp('����ͬ���ɹ�');
                        end
                    end
                end
            end
        end
        
        % ������ƫֵ��ƽ��ֵ��
        bias_gyroX = mean(gyroX_calib);
        bias_gyroY = mean(gyroY_calib);
        bias_gyroZ = mean(gyroZ_calib);
        
        fprintf('��ƫУ׼��ɣ�\n');
        fprintf('GYRO X��ƫ: %.2f\n', bias_gyroX);
        fprintf('GYRO Y��ƫ: %.2f\n', bias_gyroY);
        fprintf('GYRO Z��ƫ: %.2f\n', bias_gyroZ);
        fprintf('��ʼ���ݲɼ�...\n');
        
        % ���ݲɼ�ѭ��
        for i = 1:maxPoints
            try
                % ��ȡ�������ݰ� (17�ֽ�)
                dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
                
                % ��ȡԭʼHEXֵ
                gyroX_hex = sprintf('%02X%02X', dataPacket(5), dataPacket(6));
                gyroY_hex = sprintf('%02X%02X', dataPacket(7), dataPacket(8));
                gyroZ_hex = sprintf('%02X%02X', dataPacket(9), dataPacket(10));
                accelX_hex = sprintf('%02X%02X', dataPacket(11), dataPacket(12));
                accelY_hex = sprintf('%02X%02X', dataPacket(13), dataPacket(14));
                accelZ_hex = sprintf('%02X%02X', dataPacket(15), dataPacket(16));
                
                % ת��Ϊint16�з�������
                gyroX_int = typecast(uint16(hex2dec(gyroX_hex)), 'int16');
                gyroY_int = typecast(uint16(hex2dec(gyroY_hex)), 'int16');
                gyroZ_int = typecast(uint16(hex2dec(gyroZ_hex)), 'int16');
                accelX_int = typecast(uint16(hex2dec(accelX_hex)), 'int16');
                accelY_int = typecast(uint16(hex2dec(accelY_hex)), 'int16');
                accelZ_int = typecast(uint16(hex2dec(accelZ_hex)), 'int16');
                
                % ʹ����ƫ��������
                [gyroX_final, gyroY_final, gyroZ_final, dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ] = ...
                    gyroBiasCompensation(...
                    gyroX_int, gyroY_int, gyroZ_int, ...
                    bias_gyroX, bias_gyroY, bias_gyroZ, ...
                    dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ, ...
                    alpha);
                
                % �洢����
                gyroXData(i) = gyroX_int;
                gyroYData(i) = gyroY_int;
                gyroZData(i) = gyroZ_int;
                gyroXCompensated(i) = gyroX_final;
                gyroYCompensated(i) = gyroY_final;
                gyroZCompensated(i) = gyroZ_final;
                accelXData(i) = accelX_int;
                accelYData(i) = accelY_int;
                accelZData(i) = accelZ_int;
                
                % ������ٶȼ���̬�Ƕ�
                [accPitch, accRoll] = calculateAttitude(...
                    double(accelXData(i)), double(accelYData(i)), double(accelZData(i)), accel_range);
                
                % �����˲��ں���̬
                if i == 1
                    % ��ʼ���ںϽǶ�
                    fusedPitch = accPitch;
                    fusedRoll = accRoll;
                    fusedYaw = 0;
                else
                    % ������������ת��Ϊ��/��
                    gyroX_dps = (gyroXCompensated(i) / 32768) * gyro_range;
                    gyroY_dps = (gyroYCompensated(i) / 32768) * gyro_range;
                    gyroZ_dps = (gyroZCompensated(i) / 32768) * gyro_range;
                    
                    % ʹ�������ǻ��ָ��½Ƕ�
                    fusedPitch = fusedPitch + gyroX_dps * dt;
                    fusedRoll = fusedRoll + gyroY_dps * dt;
                    fusedYaw = fusedYaw + gyroZ_dps * dt;
                    
                    % ʹ�û����˲��ںϼ��ٶȼƵĽǶ�
                    fusedPitch = compAlpha * fusedPitch + (1 - compAlpha) * accPitch;
                    fusedRoll = compAlpha * fusedRoll + (1 - compAlpha) * accRoll;
                end
                
                % �洢�ںϺ�ĽǶ�
                pitchAngles(i) = fusedPitch;
                rollAngles(i) = fusedRoll;
                yawAngles(i) = fusedYaw;
                
                % ����Ƿ���Ҫ��ʼ�µ�5�봰��
                if mod(i-1, window_size) == 0
                    current_window_start = i;
                    window_count = window_count + 1;
                    % ����λ������
                    displacementX(i) = 0;
                    displacementY(i) = 0;
                    displacementZ(i) = 0;
                end
                
                % ����λ�ƣ���ǰ5�봰���ڣ�
                if i > current_window_start
                    window_indices = current_window_start:i;
                    [displacementX(i), displacementY(i), displacementZ(i)] = ...
                        calculateDisplacement(...
                        double(accelXData(window_indices)), ...
                        double(accelYData(window_indices)), ...
                        double(accelZData(window_indices)), ...
                        double(gyroXCompensated(window_indices)), ...
                        double(gyroYCompensated(window_indices)), ...
                        double(gyroZCompensated(window_indices)), ...
                        pitchAngles(window_indices), ...
                        rollAngles(window_indices), ...
                        dt, accel_range, gyro_range, window_count);
                else
                    % ���ڵĵ�һ���㣬λ��Ϊ0
                    displacementX(i) = 0;
                    displacementY(i) = 0;
                    displacementZ(i) = 0;
                end
                
                % ÿ10�����������һ������
                if mod(i, 10) == 0 || i == 1
                    % ת��Ϊ����λ
                    gyroX_dps = (gyroX_int / 32768) * gyro_range;
                    gyroY_dps = (gyroY_int / 32768) * gyro_range;
                    gyroZ_dps = (gyroZ_int / 32768) * gyro_range;
                    accelX_g = (accelX_int / 32768) * accel_range;
                    accelY_g = (accelY_int / 32768) * accel_range;
                    accelZ_g = (accelZ_int / 32768) * accel_range;
                    
                    fprintf('%d\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\n', ...
                        i, ...
                        gyroX_hex, gyroX_dps, ...
                        gyroY_hex, gyroY_dps, ...
                        gyroZ_hex, gyroZ_dps, ...
                        accelX_hex, accelX_g, ...
                        accelY_hex, accelY_g, ...
                        accelZ_hex, accelZ_g);
                    
                    % ��ʾ��ǰ��̬��ƫ����ֵ
                    if i > 1
                        fprintf('��̬��ƫ����: X=%.2f, Y=%.2f, Z=%.2f\n', ...
                            dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ);
                    end
                end
                
                % ����λ����ʾ
                updateDisplacementDisplay(h_displacement, h_displacement_text, ...
                    displacementX, displacementY, displacementZ, i, displacementAxes, current_window_start, window_count);
                
                % ������̬��ʾ��ÿ10�������һ����������ܣ�
                if mod(i, 10) == 0 || i == 1
                    updateAttitudeDisplay(attitudeQuiver, attitudeText, attitudeSphere, ...
                        double(accelXData(i)), double(accelYData(i)), double(accelZData(i)), ...
                        gyroXCompensated(i), gyroYCompensated(i), gyroZCompensated(i), attitudeAxes, accel_range, gyro_range, pitchAngles(i), rollAngles(i), yawAngles(i));
                end
                
                % ���µ���ͼ��
                if i > 1
                    % �������Լ��ٶȡ��ٶȺ�λ�����ڵ���
                    window_indices = max(1, i-50):i;
                    [linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = ...
                        calculateDebugData(...
                        double(accelXData(window_indices)), ...
                        double(accelYData(window_indices)), ...
                        double(accelZData(window_indices)), ...
                        pitchAngles(window_indices), ...
                        rollAngles(window_indices), ...
                        dt, accel_range);
                    
                    % �ֱ����ÿ������
                    set(h_accel_x, 'XData', window_indices, 'YData', linAccelX);
                    set(h_accel_y, 'XData', window_indices, 'YData', linAccelY);
                    set(h_accel_z, 'XData', window_indices, 'YData', linAccelZ);
                    
                    set(h_velocity_x, 'XData', window_indices, 'YData', velX);
                    set(h_velocity_y, 'XData', window_indices, 'YData', velY);
                    set(h_velocity_z, 'XData', window_indices, 'YData', velZ);
                    
                    set(h_displacement_x, 'XData', window_indices, 'YData', displacementX(window_indices));
                    set(h_displacement_y, 'XData', window_indices, 'YData', displacementY(window_indices));
                    set(h_displacement_z, 'XData', window_indices, 'YData', displacementZ(window_indices));
                end
                
                drawnow limitrate; % ���Ƹ���Ƶ�����������
                
                % �����һ֡��ͬ���ֽ�
                nextBytes = fread(s, 2, 'uint8');
                if length(nextBytes) == 2 && isequal(nextBytes', syncBytes)
                    % ͬ����ȷ������
                    continue;
                else
                    % ����ͬ��
                    syncFound = false;
                    while ~syncFound
                        byte1 = fread(s, 1, 'uint8');
                        if ~isempty(byte1) && byte1 == syncBytes(1)
                            byte2 = fread(s, 1, 'uint8');
                            if ~isempty(byte2) && byte2 == syncBytes(2)
                                syncFound = true;
                                disp('����ͬ���ɹ�');
                            end
                        end
                    end
                end
                
            catch ME
                warning('���ݶ�ȡ����: %s', ME.message);
                %fprintf('���ݶ�ȡ����: %s\n', ME.message);
                % ��������ͬ��
                syncFound = false;
                while ~syncFound
                    byte1 = fread(s, 1, 'uint8');
                    if ~isempty(byte1) && byte1 == syncBytes(1)
                        byte2 = fread(s, 1, 'uint8');
                        if ~isempty(byte2) && byte2 == syncBytes(2)
                            syncFound = true;
                            disp('���������ͬ���ɹ�');
                        end
                    end
                end
                % ���»�ȡͼ�ξ��
                fig = gcf;
                attitude_subplot = subplot(1, 2, 1);
                displacement_subplot = subplot(1, 2, 2);
            end
        end
        
        % ���ݲɼ���ɺ���ʾ�����Ϣ
        fprintf('\n���ݲɼ����\n');
    end
    
    fclose(s); % �رմ�������
    delete(s); % ɾ�����ڶ���
end