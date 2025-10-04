function [sensorData, hexStrings, syncValid] = readAndParseDataFrame(serialObj, sysParams)
    % 读取并解析一帧传感器数据
    
    syncValid = true;  % 初始化为逻辑值
    
    try
        % 读取完整数据帧 (17字节: 2同步字节 + 15数据字节)
        dataPacket = [sysParams.syncBytes'; fread(serialObj, 15, 'uint8')];
        
        % 确保数据包长度正确
        if length(dataPacket) ~= 17
            syncValid = false;
            fprintf('数据包长度错误: 期望 17 字节，实际 %d 字节\n', length(dataPacket));
            % 返回空数据
            sensorData = struct('gyroX', 0, 'gyroY', 0, 'gyroZ', 0, ...
                               'accelX', 0, 'accelY', 0, 'accelZ', 0);
            hexStrings = struct('gyroX', '0000', 'gyroY', '0000', 'gyroZ', '0000', ...
                               'accelX', '0000', 'accelY', '0000', 'accelZ', '0000');
            return;
        end
        
        % 提取各传感器数据的十六进制字符串
        hexStrings.gyroX = sprintf('%02X%02X', dataPacket(5), dataPacket(6));
        hexStrings.gyroY = sprintf('%02X%02X', dataPacket(7), dataPacket(8));
        hexStrings.gyroZ = sprintf('%02X%02X', dataPacket(9), dataPacket(10));
        hexStrings.accelX = sprintf('%02X%02X', dataPacket(11), dataPacket(12));
        hexStrings.accelY = sprintf('%02X%02X', dataPacket(13), dataPacket(14));
        hexStrings.accelZ = sprintf('%02X%02X', dataPacket(15), dataPacket(16));
        
        % 转换为有符号整数 (int16)
        sensorData.gyroX = typecast(uint16(hex2dec(hexStrings.gyroX)), 'int16');
        sensorData.gyroY = typecast(uint16(hex2dec(hexStrings.gyroY)), 'int16');
        sensorData.gyroZ = typecast(uint16(hex2dec(hexStrings.gyroZ)), 'int16');
        sensorData.accelX = typecast(uint16(hex2dec(hexStrings.accelX)), 'int16');
        sensorData.accelY = typecast(uint16(hex2dec(hexStrings.accelY)), 'int16');
        sensorData.accelZ = typecast(uint16(hex2dec(hexStrings.accelZ)), 'int16');
        
        % 验证下一帧同步字节
        syncValid = verifyNextFrameSync(serialObj, sysParams);
        
    catch ME
        fprintf('数据帧解析错误: %s\n', ME.message);
        syncValid = false;
        % 返回默认数据
        sensorData = struct('gyroX', 0, 'gyroY', 0, 'gyroZ', 0, ...
                           'accelX', 0, 'accelY', 0, 'accelZ', 0);
        hexStrings = struct('gyroX', '0000', 'gyroY', '0000', 'gyroZ', '0000', ...
                           'accelX', '0000', 'accelY', '0000', 'accelZ', '0000');
    end
end