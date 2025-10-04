    % 7. 系统清理与数据保存
function cleanupSystem(serialObj, dataStore)
% cleanup_serial.m - 串口清理脚本
fprintf('开始清理串口对象...\n');

% 获取所有串口对象
allSerial = instrfind;

if isempty(allSerial)
    fprintf('✓ 没有找到串口对象\n');
else
    fprintf('找到 %d 个串口对象:\n', length(allSerial));
    
    for i = 1:length(allSerial)
        obj = allSerial(i);
        fprintf('  %d: %s (状态: %s)\n', i, obj.Name, obj.Status);
        
        % 关闭打开的对象
        if strcmp(obj.Status, 'open')
            fclose(obj);
            fprintf('    → 已关闭\n');
        end
        
        % 删除对象
        delete(obj);
        fprintf('    → 已删除\n');
    end
end
end