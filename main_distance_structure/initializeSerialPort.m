% 2. 串口初始化与连接
function serialObj = initializeSerialPort(sysParams)
    % 初始化并打开串口连接
    
    serialObj = [];
    try
        % 创建串口对象
        serialObj = serial(sysParams.comPort);
        set(serialObj, 'BaudRate', sysParams.baudRate);
        set(serialObj, 'Timeout', 10);
        
        % 打开串口
        fopen(serialObj);
        fprintf('串口 %s 打开成功，波特率 %d\n', sysParams.comPort, sysParams.baudRate);
        
    catch ME
        fprintf('串口初始化失败: %s\n', ME.message);
        if ~isempty(serialObj)
            delete(serialObj);
        end
        serialObj = [];
    end
end