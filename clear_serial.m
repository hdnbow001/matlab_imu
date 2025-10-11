% cleanup_serial.m - 串口清理脚本
%当程序意外退出没有关闭serial port对象时，手动执行此脚本用意释放serial port对象——无需再次启动matlab
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

% 最终确认
finalCheck = instrfind;
if isempty(finalCheck)
    fprintf('✓ 串口清理完成\n');
else
    fprintf('⚠ 仍有 %d 个对象残留\n', length(finalCheck));
end

clear allSerial finalCheck;