function dataStore = updateDisplacementWindow(dataStore, imuParams, pointIndex)
    % 更新位移计算窗口
    
    if mod(pointIndex-1, imuParams.windowSize) == 0
        dataStore.window.currentStart = pointIndex;
        dataStore.window.count = dataStore.window.count + 1;
        % 新窗口开始，重置位移
        dataStore.displacement.x(pointIndex) = 0;
        dataStore.displacement.y(pointIndex) = 0;
        dataStore.displacement.z(pointIndex) = 0;
        
        fprintf('开始新的位移计算窗口 #%d\n', dataStore.window.count);
    end
end