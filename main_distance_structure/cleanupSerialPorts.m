function cleanupSerialPorts(portName)
    % 清理串口对象函数
    % 输入参数:
    %   portName - 可选，指定要清理的端口号，如'COM3'
    %             如果不指定，则清理所有串口对象
    
    if nargin == 0
        % 清理所有串口对象
        serialObjects = instrfind;
        if ~isempty(serialObjects)
            fclose(serialObjects);
            delete(serialObjects);
            fprintf('已清理所有串口对象 (%d 个)\n', length(serialObjects));
        else
            fprintf('没有找到串口对象\n');
        end
    else
        % 清理指定端口
        serialObjects = instrfind('Port', portName);
        if ~isempty(serialObjects)
            fclose(serialObjects);
            delete(serialObjects);
            fprintf('已清理端口 %s 的对象 (%d 个)\n', portName, length(serialObjects));
        else
            fprintf('端口 %s 没有找到串口对象\n', portName);
        end
    end
    
    % 清理工作区变量
    clear serialObjects;
end