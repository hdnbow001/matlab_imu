function guiHandles = refreshGUIHandles()
    % 重新获取图形句柄（当图形窗口意外关闭时使用）
    
    guiHandles = struct();
    
    try
        % 查找现有的图形窗口
        figHandles = findall(0, 'Type', 'figure');
        
        for i = 1:length(figHandles)
            figName = get(figHandles(i), 'Name');
            if contains(figName, 'IMU传感器实时数据显示')
                guiHandles.mainFig = figHandles(i);
                % 重新获取子图句柄
                axesHandles = findall(guiHandles.mainFig, 'Type', 'axes');
                if length(axesHandles) >= 2
                    guiHandles.attitude.axes = axesHandles(1);
                    guiHandles.displacement.axes = axesHandles(2);
                end
            elseif contains(figName, '传感器数据调试显示')
                guiHandles.debugFig = figHandles(i);
            end
        end
        
        fprintf('图形句柄已刷新\n');
        
    catch ME
        fprintf('刷新图形句柄失败: %s\n', ME.message);
    end
end