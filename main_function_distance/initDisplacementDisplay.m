% ��ʼ��λ����ʾ����
function [h_displacement, h_displacement_text, displacementAxes] = initDisplacementDisplay(parentAxes)
    % ���õ�ǰ������
    axes(parentAxes);
    
    % ����3Dλ�ƹ켣ͼ
    h_displacement = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 2);
    hold on;
    % ��������
    plot3(0, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    grid on;
    xlabel('Xλ�� (m)');
    ylabel('Yλ�� (m)');
    zlabel('Zλ�� (m)');
    title('������λ�ƹ켣 (5�봰��)');
    
    % ���ù̶������᷶Χ [-1, 1] ��
    xlim([-1, 1]);
    ylim([-1, 1]);
    zlim([-1, 1]);
    
    % ����3D��ת
    rotate3d on;
    
    % ���������
    line([-1 1], [0 0], [0 0], 'Color', 'r', 'LineStyle', '--');
    line([0 0], [-1 1], [0 0], 'Color', 'g', 'LineStyle', '--');
    line([0 0], [0 0], [-1 1], 'Color', 'b', 'LineStyle', '--');
    
    % ����ı���Ϣ - ʹ��Ԫ�����飨����º�������һ�£�
    text_cell = {
        'λ��: (0.00, 0.00, 0.00) m', 
        '����: 1'
    };
    h_displacement_text = text(0, 0, 1.1, text_cell, ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
        'Interpreter', 'none'); % ��Ҫ������LaTeX������
    
    % ������������
    displacementAxes = gca;
end