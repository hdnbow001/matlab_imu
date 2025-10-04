function success = resyncSerialPort(serialObj, sysParams)
    % 重新同步串口数据流
    
    success = false;
    maxAttempts = 1000;
    attempts = 0;
    
    fprintf('正在重新同步串口数据流...\n');
    
    while attempts < maxAttempts
        try
            byte1 = fread(serialObj, 1, 'uint8');
            if ~isempty(byte1) && byte1 == sysParams.syncBytes(1)
                byte2 = fread(serialObj, 1, 'uint8');
                if ~isempty(byte2) && byte2 == sysParams.syncBytes(2)
                    success = true;
                    fprintf('串口重新同步成功 (尝试次数: %d)\n', attempts + 1);
                    break;
                end
            end
            attempts = attempts + 1;
            
            % 每100次尝试显示进度
            if mod(attempts, 100) == 0
                fprintf('重新同步进度: %d/%d 次尝试\n', attempts, maxAttempts);
            end
            
        catch ME
            fprintf('重新同步过程中出错: %s\n', ME.message);
        end
    end
    
    if ~success
        fprintf('串口重新同步失败，达到最大尝试次数 %d\n', maxAttempts);
    end
end