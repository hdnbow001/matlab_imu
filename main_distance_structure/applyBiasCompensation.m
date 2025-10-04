function dataStore = applyBiasCompensation(sensorData, dataStore, gyroBiases, imuParams, pointIndex)
    % 应用静态和动态零偏补偿 - 使用现有的gyroBiasCompensation函数
    
    [gyroXComp, gyroYComp, gyroZComp, ...
     dataStore.bias.dynamicX, dataStore.bias.dynamicY, dataStore.bias.dynamicZ] = ...
        gyroBiasCompensation(...
        sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ, ...
        gyroBiases.x, gyroBiases.y, gyroBiases.z, ...
        dataStore.bias.dynamicX, dataStore.bias.dynamicY, dataStore.bias.dynamicZ, ...
        imuParams.dynamicBiasAlpha);
    
    % 存储补偿后的数据
    dataStore.compensated.gyroX(pointIndex) = gyroXComp;
    dataStore.compensated.gyroY(pointIndex) = gyroYComp;
    dataStore.compensated.gyroZ(pointIndex) = gyroZComp;
end
%%







%%

