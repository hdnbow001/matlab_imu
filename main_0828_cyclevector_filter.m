clc;
clear;
close all;

fs = 5;   % ������
T = 20;   % ʱ��
n = round(T*fs);  % ���������
s = serial('COM7'); % �������ڶ���
set(s, 'BaudRate', 115200); % ���ò�����
dataSwitch = 0;     % 0 �ɼ����� 1 ���� 2 ֹͣ

flag = false;
try
    fopen(s);
    flag = true;
    disp('���ڴ򿪳ɹ�');
catch
    error('���ڴ�ʧ��');
end

% ����/�رշ���
if flag
    if dataSwitch == 1
        hexdata = 'aa 55 05 00 00 00 00 00 00 00 00 00 00 00 00 CA 0D'; % ����
        asciiValues = hex2dec(split(hexdata));
        fwrite(s, asciiValues);
        data = fread(s, 17);
    elseif dataSwitch == 2
        hexdata = 'aa 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0D'; % �ر�
        asciiValues = hex2dec(split(hexdata));
        fwrite(s, asciiValues);
    else
        % ��ʼ�����ݴ洢
        maxPoints = 2000;
        timeData = zeros(1, maxPoints);
        pitchData1 = zeros(1, maxPoints);  % ��7-8�ֽڵĽǶ�
        pitchData2 = zeros(1, maxPoints);  % ��9-10�ֽڵĽǶ�
        byte4Data = zeros(1, maxPoints);
        diffByte4Data = zeros(1, maxPoints); % �洢��ֵ
        
        % ����ͼ�δ��ں���ͼ - 2x2����
        figure('Position', [100, 100, 1200, 900]); % ���ô��ڴ�С
        
        % ��һ����ͼ����һ���Ƕȵ�Բ��ʸ����ʾ
        subplot(2, 2, 1);
        % ���Ʋο�Բ
        theta = linspace(0, 2*pi, 100);
        plot(cos(theta), sin(theta), 'k--');
        hold on;
        axis equal;
        grid on;
        xlim([-1.2, 1.2]);
        ylim([-1.2, 1.2]);
        title('�Ƕ�1 (7-8�ֽ�) ʸ����ʾ');
        xlabel('X��');
        ylabel('Y��');
        
        % ��ʼ����һ��������ʸ������ɫ��
        pitchVector1 = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % ��ӽǶ��ı�
        currentAngleText1 = text(-1.1, 1.0, '�Ƕ�1: 0��', 'FontSize', 10, 'Color', 'r');
        
        % ��ӽǶȱ��
        text(1.05, 0, '0��', 'FontSize', 8);
        text(0, 1.05, '90��', 'FontSize', 8);
        text(-1.05, 0, '180��', 'FontSize', 8);
        text(0, -1.05, '270��', 'FontSize', 8);
        
        % �ڶ�����ͼ���ڶ����Ƕȵ�Բ��ʸ����ʾ
        subplot(2, 2, 2);
        % ���Ʋο�Բ
        plot(cos(theta), sin(theta), 'k--');
        hold on;
        axis equal;
        grid on;
        xlim([-1.2, 1.2]);
        ylim([-1.2, 1.2]);
        title('�Ƕ�2 (9-10�ֽ�) ʸ����ʾ');
        xlabel('X��');
        ylabel('Y��');
        
        % ��ʼ���ڶ���������ʸ������ɫ��
        pitchVector2 = quiver(0, 0, 0, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % ��ӽǶ��ı�
        currentAngleText2 = text(-1.1, 1.0, '�Ƕ�2: 0��', 'FontSize', 10, 'Color', 'b');
        
        % ��ӽǶȱ��
        text(1.05, 0, '0��', 'FontSize', 8);
        text(0, 1.05, '90��', 'FontSize', 8);
        text(-1.05, 0, '180��', 'FontSize', 8);
        text(0, -1.05, '270��', 'FontSize', 8);
        
        % ��������ͼ�����ĸ��ֽڵ�ֵ
        subplot(2, 2, 3);
        h2 = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
        title('���ĸ��ֽ�ʵʱ����');
        xlabel('������');
        ylabel('��ֵ');
        ylim([0 255]); % �����ֽ�ֵ��Χ��0-255
        grid on;
        
        % ���ĸ���ͼ�����ĸ��ֽڵĲ�ֵ
        subplot(2, 2, 4);
        h3 = plot(NaN, NaN, 'm-', 'LineWidth', 1.5);
        title('���ĸ��ֽڱ仯��');
        xlabel('������');
        ylabel('��ֵ');
        grid on;
        
        % ֡ͬ��
        syncBytes = [170, 85];
        syncFound = false;
        
        while ~syncFound
            byte1 = fread(s, 1, 'uint8');
            if ~isempty(byte1) && byte1 == syncBytes(1)
                byte2 = fread(s, 1, 'uint8');
                if ~isempty(byte2) && byte2 == syncBytes(2)
                    syncFound = true;
                    disp('֡ͬ���ɹ�');
                end
            end
        end
        
        % ��ʼ��ǰһֵ
        prevByte4Value = 0;
        
        % ���ݲɼ�ѭ��
        for i = 1:maxPoints
            try
                % ��ȡ�������ݰ�
                dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
                
                % ��ȡ����������
                % 1. ��һ�������� (��7-8�ֽ�)
                pitchValue1 = fun_hex2dec(dataPacket(7), dataPacket(8));
                
                % 2. �ڶ��������� (��9-10�ֽ�)
                pitchValue2 = fun_hex2dec(dataPacket(9), dataPacket(10));
                
                % 3. ���ĸ��ֽڵ�ֵ
                byte4Value = double(dataPacket(4)); % ֱ��ת��Ϊ��ֵ
                
                % 4. ������ĸ��ֽڵĲ�ֵ
                if i == 1
                    diffValue = 0; % ��һ����û��ǰһ���㣬��ֵΪ0
                else
                    diffValue = byte4Value - prevByte4Value;
                end
                
                % ����ǰһֵ
                prevByte4Value = byte4Value;
                
                % �洢����
                pitchData1(i) = pitchValue1;
                pitchData2(i) = pitchValue2;
                byte4Data(i) = byte4Value;
                diffByte4Data(i) = diffValue;
                timeData(i) = i;
                
                % ���µ�һ���Ƕ�ʸ����ʾ
                subplot(2, 2, 1);
                angle_rad1 = deg2rad(pitchValue1);
                x_end1 = cos(angle_rad1);
                y_end1 = sin(angle_rad1);
                set(pitchVector1, 'UData', x_end1, 'VData', y_end1);
                set(currentAngleText1, 'String', sprintf('�Ƕ�1: %.1f��', pitchValue1));
                
                % ���µڶ����Ƕ�ʸ����ʾ
                subplot(2, 2, 2);
                angle_rad2 = deg2rad(pitchValue2);
                x_end2 = cos(angle_rad2);
                y_end2 = sin(angle_rad2);
                set(pitchVector2, 'UData', x_end2, 'VData', y_end2);
                set(currentAngleText2, 'String', sprintf('�Ƕ�2: %.1f��', pitchValue2));
                
                % ���µ��ĸ��ֽ�ͼ��
                subplot(2, 2, 3);
                set(h2, 'XData', timeData(1:i), 'YData', byte4Data(1:i));
                
                % ���µ��ĸ��ֽڲ�ֵͼ��
                subplot(2, 2, 4);
                set(h3, 'XData', timeData(1:i), 'YData', diffByte4Data(1:i));
                
                % ��̬����X�᷶Χ
                if i > 50
                    subplot(2, 2, 3);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    subplot(2, 2, 4);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    
                    % �Զ�������ֵͼ��Y�᷶Χ
                    visibleDiffData = diffByte4Data(max(1, i-50):i);
                    if ~isempty(visibleDiffData)
                        yRange = max(abs(visibleDiffData)) * 1.1;
                        if yRange == 0
                            yRange = 1; % ����yRangeΪ0�����
                        end
                        subplot(2, 2, 4);
                        ylim([-yRange, yRange]);
                    end
                end
                
                % ���ͼ��
                if i == 1
                    subplot(2, 2, 3);
                    legend('�ֽ�4ֵ', 'Location', 'best');
                    subplot(2, 2, 4);
                    legend('�仯��', 'Location', 'best');
                end
                
                % ʵʱ��ʾ��ǰֵ
                if mod(i, 10) == 0 || i == 1
                    fprintf('������ %d: �Ƕ�1=%.2f��, �Ƕ�2=%.2f��, �ֽ�4ֵ=%d, �仯��=%d\n', ...
                        i, pitchValue1, pitchValue2, byte4Value, diffValue);
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
            end
        end
        
        % ���ݲɼ���ɺ���ʾͳ����Ϣ
        fprintf('\n���ݲɼ����\n');
        fprintf('�Ƕ�1ͳ��: ��Сֵ=%.2f��, ���ֵ=%.2f��, ƽ��ֵ=%.2f��\n', ...
            min(pitchData1), max(pitchData1), mean(pitchData1));
        fprintf('�Ƕ�2ͳ��: ��Сֵ=%.2f��, ���ֵ=%.2f��, ƽ��ֵ=%.2f��\n', ...
            min(pitchData2), max(pitchData2), mean(pitchData2));
        fprintf('�ֽ�4ͳ��: ��Сֵ=%d, ���ֵ=%d, ƽ��ֵ=%.2f\n', ...
            min(byte4Data), max(byte4Data), mean(byte4Data));
        
        % ��ѡ���������ݵ��ļ�
        save('sensor_data.mat', 'timeData', 'pitchData1', 'pitchData2', 'byte4Data', 'diffByte4Data');
        disp('�����ѱ��浽 sensor_data.mat');
    end
end

fclose(s); % �رմ�������
delete(s); % ɾ�����ڶ���