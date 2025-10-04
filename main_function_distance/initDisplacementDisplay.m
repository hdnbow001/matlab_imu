% 初始化位移显示函数
function [h_displacement, h_displacement_text, displacementAxes] = initDisplacementDisplay(parentAxes)
    % 设置当前坐标轴
    axes(parentAxes);
    
    % 创建3D位移轨迹图
    h_displacement = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 2);
    hold on;
    % 添加起点标记
    plot3(0, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    grid on;
    xlabel('X位移 (mm)');
    ylabel('Y位移 (mm)');
    zlabel('Z位移 (mm)');
    title('传感器位移轨迹 (5秒窗口)');
    
    % 设置固定坐标轴范围 [-100, 100] 毫米
    xlim([-100, 100]);
    ylim([-100, 100]);
    zlim([-100, 100]);
    
    % 启用3D旋转
    rotate3d on;
    
    % 添加坐标轴
    line([-100 100], [0 0], [0 0], 'Color', 'r', 'LineStyle', '--');
    line([0 0], [-100 100], [0 0], 'Color', 'g', 'LineStyle', '--');
    line([0 0], [0 0], [-100 100], 'Color', 'b', 'LineStyle', '--');
    
    % 添加刻度标记
    % X轴刻度
    for x = -100:50:100
        if x ~= 0
            plot3([x x], [0 0], [0 -5], 'k-', 'LineWidth', 1);
            text(x, -10, -10, sprintf('%d', x), 'FontSize', 8, 'HorizontalAlignment', 'center');
        end
    end
    
    % Y轴刻度
    for y = -100:50:100
        if y ~= 0
            plot3([0 0], [y y], [0 -5], 'k-', 'LineWidth', 1);
            text(-10, y, -10, sprintf('%d', y), 'FontSize', 8, 'HorizontalAlignment', 'center');
        end
    end
    
    % Z轴刻度
    for z = -100:50:100
        if z ~= 0
            plot3([0 -5], [0 0], [z z], 'k-', 'LineWidth', 1);
            text(-10, -10, z, sprintf('%d', z), 'FontSize', 8, 'HorizontalAlignment', 'center');
        end
    end
    
    % 添加文本信息 - 使用元胞数组
    text_cell = {
        '位移: (0.00, 0.00, 0.00) mm',
        '窗口: 1'
    };
    h_displacement_text = text(0, 0, 110, text_cell, ...  % Z坐标调整为110以匹配新的坐标范围
        'HorizontalAlignment', 'center', 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
        'Interpreter', 'none', 'FontSize', 10);
    
    % 保存坐标轴句柄
    displacementAxes = gca;
end