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
    disp('UART open success��');
catch
    error('UART open fail��');
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
        maxPoints = 1000;
        timeData = zeros(1, maxPoints);
        pitchData = zeros(1, maxPoints);
        byte4Data = zeros(1, maxPoints);
        
        % ����ͼ�δ��ں���ͼ
        figure;
        
        % ��һ����ͼ��������
        subplot(2, 1, 1);
        h1 = plot(NaN, NaN, 'r-');
        title('pitch');
        xlabel('samples');
        ylabel('pitch angle (��)');
        ylim([0 360]);
        grid on;
        
        % �ڶ�����ͼ�����ĸ��ֽڵ�ֵ
        subplot(2, 1, 2);
        h2 = plot(NaN, NaN, 'b-');
        title('position change counter');
        xlabel('samples');
        ylabel('counter');
        ylim([0 255]); % �����ֽ�ֵ��Χ��0-255
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
                    disp('Fram synced');
                end
            end
        end
        
        % ���ݲɼ�ѭ��
        for i = 1:maxPoints
            % ��ȡ�������ݰ�
            dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
            
            % ��ȡ����������
            % 1. ������ (��7-8�ֽ�)
            pitchValue = fun_hex2dec(dataPacket(7), dataPacket(8));
            
            % 2. ���ĸ��ֽڵ�ֵ
            byte4Value = double(dataPacket(4)); % ֱ��ת��Ϊ��ֵ
            
            % �洢����
            pitchData(i) = pitchValue;
            byte4Data(i) = byte4Value;
            timeData(i) = i;
            
            % ���¸�����ͼ��
            set(h1, 'XData', timeData(1:i), 'YData', pitchData(1:i));
            
            % ���µ��ĸ��ֽ�ͼ��
            set(h2, 'XData', timeData(1:i), 'YData', byte4Data(1:i));
            
            % ��̬����X�᷶Χ
            if i > 50
                subplot(2, 1, 1);
                xlim([max(1, i-50), i+10]);
                subplot(2, 1, 2);
                xlim([max(1, i-50), i+10]);
            end
            
            drawnow;
            
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
        end
    end
end

fclose(s); % �رմ�������
delete(s); % ɾ�����ڶ���