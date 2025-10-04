% 3. 数据缓冲区初始化
function dataStore = initializeDataStorage(maxPoints)
    % 初始化所有数据存储结构
    
    % 原始传感器数据（int16类型）
    dataStore.raw.gyroX = zeros(1, maxPoints, 'int16');
    dataStore.raw.gyroY = zeros(1, maxPoints, 'int16');
    dataStore.raw.gyroZ = zeros(1, maxPoints, 'int16');
    dataStore.raw.accelX = zeros(1, maxPoints, 'int16');
    dataStore.raw.accelY = zeros(1, maxPoints, 'int16');
    dataStore.raw.accelZ = zeros(1, maxPoints, 'int16');
    
    % 零偏补偿后的陀螺仪数据
    dataStore.compensated.gyroX = zeros(1, maxPoints);
    dataStore.compensated.gyroY = zeros(1, maxPoints);
    dataStore.compensated.gyroZ = zeros(1, maxPoints);
    
    % 姿态角度数据
    dataStore.attitude.pitch = zeros(1, maxPoints);
    dataStore.attitude.roll = zeros(1, maxPoints);
    dataStore.attitude.yaw = zeros(1, maxPoints);
    
    % 位移数据
    dataStore.displacement.x = zeros(1, maxPoints);
    dataStore.displacement.y = zeros(1, maxPoints);
    dataStore.displacement.z = zeros(1, maxPoints);
    
    % 动态零偏估计值
    dataStore.bias.dynamicX = 0;
    dataStore.bias.dynamicY = 0;
    dataStore.bias.dynamicZ = 0;
    
    % 窗口管理参数
    dataStore.window.currentStart = 1;
    dataStore.window.count = 1;
    
    % 融合姿态角度（用于互补滤波）
    dataStore.fused.pitch = 0;
    dataStore.fused.roll = 0;
    dataStore.fused.yaw = 0;
    
    fprintf('数据存储结构初始化完成，预留 %d 个数据点\n', maxPoints);
end