function syncValid = verifyNextFrameSync(serialObj, sysParams)
    % 验证下一帧数据的同步字节
    % 返回逻辑值 true-同步成功 false-同步失败
    
    syncValid = true;
    
    try
        % 读取下一帧的同步字节
        nextBytes = fread(serialObj, 2, 'uint8');
        
        % 检查是否成功读取2个字节且与同步字节匹配
        if length(nextBytes) == 2 && isequal(nextBytes', sysParams.syncBytes)
            % 同步成功
            syncValid = true;
        else
            % 同步失败
            syncValid = false;
            fprintf('同步字节验证失败\n');
            
            % 尝试重新同步
            resyncSuccess = resyncSerialPort(serialObj, sysParams);
            if resyncSuccess
                fprintf('重新同步成功\n');
                syncValid = true;
            end
        end
        
    catch ME
        fprintf('同步验证过程中出错: %s\n', ME.message);
        syncValid = false;
    end
end