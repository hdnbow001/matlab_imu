% === 新增：零偏校准函数 ===
function [bias_gyroX, bias_gyroY, bias_gyroZ] = gyroBiasCalibration(s, calibrationSamples, syncBytes)
    % 零偏校准函数
    % 输入：
    %   s - 串口对象
    %   calibrationSamples - 校准采样点数
    %   syncBytes - 同步字节 [170, 85]
    % 输出：
    %   bias_gyroX, bias_gyroY, bias_gyroZ - 三轴陀螺仪零偏值
    
    disp('开始零偏校准，请保持传感器静止...');
    
    % 初始化校准数据存储
    gyroX_calib = zeros(1, calibrationSamples);
    gyroY_calib = zeros(1, calibrationSamples);
    gyroZ_calib = zeros(1, calibrationSamples);
    
    syncFound = false;
    
    for calib_i = 1:calibrationSamples
        % 读取完整数据包 (17字节)
        dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
        
        % 提取原始HEX值
        gyroX_hex = sprintf('%02X%02X', dataPacket(5), dataPacket(6));
        gyroY_hex = sprintf('%02X%02X', dataPacket(7), dataPacket(8));
        gyroZ_hex = sprintf('%02X%02X', dataPacket(9), dataPacket(10));
        
        % 转换为int16有符号整数
        gyroX_calib(calib_i) = double(typecast(uint16(hex2dec(gyroX_hex)), 'int16'));
        gyroY_calib(calib_i) = double(typecast(uint16(hex2dec(gyroY_hex)), 'int16'));
        gyroZ_calib(calib_i) = double(typecast(uint16(hex2dec(gyroZ_hex)), 'int16'));
        
        % 检查下一帧的同步字节
        nextBytes = fread(s, 2, 'uint8');
        if ~(length(nextBytes) == 2 && isequal(nextBytes', syncBytes))
            % 重新同步
            syncFound = false;
            while ~syncFound
                byte1 = fread(s, 1, 'uint8');
                if ~isempty(byte1) && byte1 == syncBytes(1)
                    byte2 = fread(s, 1, 'uint8');
                    if ~isempty(byte2) && byte2 == syncBytes(2)
                        syncFound = true;
                        disp('重新同步成功');
                    end
                end
            end
        end
        
        % 显示校准进度
        if mod(calib_i, 10) == 0
            fprintf('校准进度: %d/%d\n', calib_i, calibrationSamples);
        end
    end
    
    % 计算零偏值（平均值）
    bias_gyroX = mean(gyroX_calib);
    bias_gyroY = mean(gyroY_calib);
    bias_gyroZ = mean(gyroZ_calib);
    
    fprintf('零偏校准完成：\n');
    fprintf('GYRO X零偏: %.2f\n', bias_gyroX);
    fprintf('GYRO Y零偏: %.2f\n', bias_gyroY);
    fprintf('GYRO Z零偏: %.2f\n', bias_gyroZ);
    fprintf('开始数据采集...\n');