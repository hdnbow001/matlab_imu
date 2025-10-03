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
        pitchData = zeros(1, maxPoints);
        byte4Data = zeros(1, maxPoints);
        diffByte4Data = zeros(1, maxPoints); % �洢��ֵ
        
        % ����ͼ�δ��ں���ͼ
        figure('Position', [100, 100, 1000, 900]); % ���ô��ڴ�С
        
        % ��һ����ͼ��Բ�����ϵĸ�����ʸ����ʾ
        subplot(3, 1, 1);
        % ���Ʋο�Բ
        theta = linspace(0, 2*pi, 100);
        plot(cos(theta), sin(theta), 'k--');
        hold on;
        axis equal;
        grid on;
        xlim([-1.2, 1.2]);
        ylim([-1.2, 1.2]);
        title('������ʸ����ʾ');
        xlabel('X��');
        ylabel('Y��');
        
        % ��ʼ��������ʸ��
        pitchVector = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        currentAngleText = text(-1.1, 1.0, '�Ƕ�: 0��', 'FontSize', 10);
        
        % ��ӽǶȱ��
        text(1.05, 0, '0��', 'FontSize', 8);
        text(0, 1.05, '90��', 'FontSize', 8);
        text(-1.05, 0, '180��', 'FontSize', 8);
        text(0, -1.05, '270��', 'FontSize', 8);
        
        % �ڶ�����ͼ�����ĸ��ֽڵ�ֵ
        subplot(3, 1, 2);
        h2 = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
        title('���ĸ��ֽ�ʵʱ����');
        xlabel('������');
        ylabel('��ֵ');
        ylim([0 255]); % �����ֽ�ֵ��Χ��0-255
        grid on;
        
        % ��������ͼ�����ĸ��ֽڵĲ�ֵ
        subplot(3, 1, 3);
        h3 = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
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
                % 1. ������ (��7-8�ֽ�)
                pitchValue = fun_hex2dec(dataPacket(7), dataPacket(8));
                
                % 2. ���ĸ��ֽڵ�ֵ
                byte4Value = double(dataPacket(4)); % ֱ��ת��Ϊ��ֵ
                
                % 3. ������ĸ��ֽڵĲ�ֵ
                if i == 1
                    diffValue = 0; % ��һ����û��ǰһ���㣬��ֵΪ0
                else
                    diffValue = byte4Value - prevByte4Value;
                end
                
                % ����ǰһֵ
                prevByte4Value = byte4Value;
                
                % �洢����
                pitchData(i) = pitchValue;
                byte4Data(i) = byte4Value;
                diffByte4Data(i) = diffValue;
                timeData(i) = i;
                
                % ���¸�����ʸ����ʾ
                subplot(3, 1, 1);
                % ����ʸ�����յ�����
                angle_rad = deg2rad(pitchValue);
                x_end = cos(angle_rad);
                y_end = sin(angle_rad);
                
                % ����ʸ��
                set(pitchVector, 'UData', x_end, 'VData', y_end);
                
                % ���½Ƕ��ı�
                set(currentAngleText, 'String', sprintf('�Ƕ�: %.1f��', pitchValue));
                
                % ���µ��ĸ��ֽ�ͼ��
                subplot(3, 1, 2);
                set(h2, 'XData', timeData(1:i), 'YData', byte4Data(1:i));
                
                % ���µ��ĸ��ֽڲ�ֵͼ��
                subplot(3, 1, 3);
                set(h3, 'XData', timeData(1:i), 'YData', diffByte4Data(1:i));
                
                % ��̬����X�᷶Χ
                if i > 50
                    subplot(3, 1, 2);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    subplot(3, 1, 3);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    
                    % �Զ�������ֵͼ��Y�᷶Χ
                    visibleDiffData = diffByte4Data(max(1, i-50):i);
                    if ~isempty(visibleDiffData)
                        yRange = max(abs(visibleDiffData)) * 1.1;
                        if yRange == 0
                            yRange = 1; % ����yRangeΪ0�����
                        end
                        ylim([-yRange, yRange]);
                    end
                end
                
                % ���ͼ������ֵ��ʾ
                if i == 1
                    subplot(3, 1, 2);
                    legend('�ֽ�4ֵ', 'Location', 'best');
                    subplot(3, 1, 3);
                    legend('�仯��', 'Location', 'best');
                end
                
                % ʵʱ��ʾ��ǰֵ
                if mod(i, 10) == 0 || i == 1
                    fprintf('������ %d: ������=%.2f��, �ֽ�4ֵ=%d, �仯��=%d\n', ...
                        i, pitchValue, byte4Value, diffValue);
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
    end
end

fclose(s); % �رմ�������
delete(s); % ɾ�����ڶ���