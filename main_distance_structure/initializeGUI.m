function guiHandles = initializeGUI()
    % 初始化所有图形用户界面
    
    guiHandles = struct();
    
    % 主显示窗口
    guiHandles.mainFig = figure('Position', [50, 50, 1400, 600], ...
        'Name', 'IMU传感器实时数据显示', 'NumberTitle', 'off');
    
    % 姿态显示子图
    guiHandles.attitude.axes = subplot(1, 2, 1);
    [guiHandles.attitude.quiver, guiHandles.attitude.text, ...
     guiHandles.attitude.sphere, guiHandles.attitude.axes] = ...
        initAttitudeDisplay(guiHandles.attitude.axes);
    
    % 位移显示子图
    guiHandles.displacement.axes = subplot(1, 2, 2);
    [guiHandles.displacement.plot, guiHandles.displacement.text, guiHandles.displacement.axes] = ...
        initDisplacementDisplay(guiHandles.displacement.axes);
    
    % 调试数据显示窗口
    guiHandles.debugFig = createDebugFigure(guiHandles);
    
    fprintf('图形用户界面初始化完成\n');
end