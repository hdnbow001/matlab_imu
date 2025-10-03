% ����λ����ʾ����
function updateDisplacementDisplay(h_displacement, h_displacement_text, displacementX, displacementY, displacementZ, currentIndex, displacementAxes, window_start, window_count)
    % ���õ�ǰ������
    axes(displacementAxes);
    
    % ȷ��������Ч
    if window_start > currentIndex
        window_start = max(1, currentIndex - 50);
    end
    
    window_indices = window_start:min(currentIndex, length(displacementX));
    
    % ����λ�ƹ켣��ֻ��ʾ��ǰ5�봰���ڵ����ݣ�
    if ~isempty(window_indices)
        set(h_displacement, 'XData', displacementX(window_indices), ...
                            'YData', displacementY(window_indices), ...
                            'ZData', displacementZ(window_indices), ...
                            'Visible', 'on');
    end
    
    % ����λ���ı���Ϣ
    if currentIndex > 0 && currentIndex <= length(displacementX)
        text_str = {sprintf('λ��: (%.2f, %.2f, %.2f) m', ...
                    displacementX(currentIndex), displacementY(currentIndex), displacementZ(currentIndex)), ...
                    sprintf('����: %d', window_count)};
        set(h_displacement_text, 'String', text_str, 'Visible', 'on');
    end
    
    % ˢ��ͼ��
    drawnow limitrate;
end
