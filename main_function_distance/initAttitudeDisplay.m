% ��ʼ����̬��ʾ����
function [attitudeQuiver, attitudeText, attitudeSphere, attitudeAxes] = initAttitudeDisplay(attitude_subplot)
    % ���õ�ǰ������
    axes(attitude_subplot);
    attitudeAxes = attitude_subplot; % ������������
    
    % �������������
    cla(attitudeAxes);
    hold on;
    grid on;
    axis equal;
    xlim([-1.5, 1.5]);
    ylim([-1.5, 1.5]);
    zlim([-1.5, 1.5]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('��̬��ʾ');
    view(45, 25);
    
    % ���Ʋο�����ϵ
    quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % ������λ����
    [x, y, z] = sphere(20);
    attitudeSphere = surf(x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);
    
    % ������̬��ͷ
    attitudeQuiver = quiver3(0, 0, 0, 0, 0, 1, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.8);
    
    % �����ı����� - ʹ��sprintf���ɴ����е��ı�
    initialText = sprintf('������: 0.00��\n��ת��: 0.00��\nƫ����: 0.00��');
    % attitudeText = text(-1, -1, 1.2, initialText, ...
    %     'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
    %     'VerticalAlignment', 'top', 'Interpreter', 'none');
    attitudeText = text(3, -2, 3, initialText, ...
    'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'Interpreter', 'none');

    
    hold off;
end