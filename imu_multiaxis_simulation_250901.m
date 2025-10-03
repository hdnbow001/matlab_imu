function imu_multiaxis_simulation()
    % IMU_MULTIAXIS_SIMULATION ����IMU���ٶ����ݷ�����λ�ƽ���
    % ģ����X��Y��Z��������ͬʱ���˶������
    
    close all; clear all; clc;
    
    %% ��������
    Fs = 100;           % ������ 100 Hz
    dt = 1/Fs;          % ����ʱ����
    T = 10;             % ��ʱ�� 10��
    t = (0:dt:T-dt)';   % ʱ������
    N = length(t);
    
    fprintf('����IMU������λ�ƽ���...\n');
    fprintf('������: %.0f Hz\n', Fs);
    fprintf('����ʱ��: %.1f ��\n', T);
    fprintf('���ݵ���: %d\n', N);
    
    %% 1. ģ����ά�˶��켣
    fprintf('\n1. ģ����ά�˶��켣...\n');
    
    % ������ά�˶����� - ���������˶�
    omega_x = 0.5 * pi;  % X���Ƶ�� (rad/s)
    omega_y = 0.8 * pi;  % Y���Ƶ�� (rad/s)
    omega_z = 0.3 * pi;  % Z���Ƶ�� (rad/s)
    
    A_x = 2.0;          % X����� (m/s?)
    A_y = 1.5;          % Y����� (m/s?)
    A_z = 1.2;          % Z����� (m/s?)
    
    % ���������ϵ��˶����ٶ� (��������ƫ��)
    accel_motion = zeros(N, 3);
    accel_motion(:, 1) = -A_x * omega_x^2 * sin(omega_x * t);  % X����ٶ�
    accel_motion(:, 2) = -A_y * omega_y^2 * cos(omega_y * t);  % Y����ٶ�
    accel_motion(:, 3) = -A_z * omega_z^2 * sin(omega_z * t);  % Z����ٶ�
    
    % ͨ�����ּ��������ٶȺ�λ�� (������֤)
    velocity_theoretical = cumtrapz(dt, accel_motion);
    position_theoretical = cumtrapz(dt, velocity_theoretical);
    
    %% 2. ģ��IMU��������
    fprintf('2. ģ��IMU��������...\n');
    
    % IMU����ֵ = �˶����ٶ� + �������ٶ� + ���� + ƫ��
    g = 9.81;
    gravity = repmat([0, 0, g], N, 1);  % NED����ϵ������Ϊ [0, 0, 9.81]
    
    % ���������ƫ��
    bias = [0.05, -0.03, 0.08];         % ���᲻ͬ��ƫ��
    noise_level = 0.1;                  % ����ˮƽ
    
    accel_bias = repmat(bias, N, 1);
    accel_noise = noise_level * randn(N, 3);
    
    % �ϳ�"������"�ļ��ٶȼ����� (��������ϵ)
    accel_body = accel_motion + gravity + accel_bias + accel_noise;
    
    % ģ������������ - С�������ת
    gyro_noise_level = 0.02; % ����������ˮƽ (rad/s)
    gyro_body = gyro_noise_level * randn(N, 3);
    
    % ���һЩС����������ת
    gyro_body(:, 1) = gyro_body(:, 1) + 0.1 * sin(0.5 * pi * t);
    gyro_body(:, 2) = gyro_body(:, 2) + 0.08 * cos(0.3 * pi * t);
    
    fprintf('IMU����ģ�����\n');
    
    %% 3. ��̬���㣨ʹ�û����˲���
    fprintf('3. ������̬���㣨�����˲���...\n');
    
    % ��ʼ����̬�ǣ���ת��������ƫ����
    pitch = zeros(N, 1);
    roll = zeros(N, 1);
    yaw = zeros(N, 1);
    
    % �����˲�ϵ��
    alpha = 0.98;
    
    for i = 2:N
        % �������ǻ��ֵõ���̬
        gyro_rates = gyro_body(i, :);
        pitch_gyro = pitch(i-1) + gyro_rates(1) * dt;
        roll_gyro  = roll(i-1)  + gyro_rates(2) * dt;
        yaw_gyro   = yaw(i-1)   + gyro_rates(3) * dt;

        % �ü��ٶȼƼ����ת�͸�����
        acc = accel_body(i, :);
        roll_acc = atan2(acc(2), sign(acc(3)) * sqrt(acc(1)^2 + acc(3)^2));
        pitch_acc = atan2(-acc(1), sqrt(acc(2)^2 + acc(3)^2));

        % �����˲��ں�
        roll(i) = alpha * roll_gyro + (1 - alpha) * roll_acc;
        pitch(i) = alpha * pitch_gyro + (1 - alpha) * pitch_acc;
        yaw(i) = yaw_gyro; % ���ٶȼ��޷��۲�ƫ������ȫ����������
    end
    
    fprintf('��̬�������\n');
    
    %% 4. ����任����������
    fprintf('4. ��������任����������...\n');
    
    accel_global = zeros(N, 3); % ȫ������ϵ�µļ��ٶ�
    
    for i = 1:N
        % ���ݵ�ǰ��̬�ǹ�����ת���� R���ӱ�������ϵ��ȫ������ϵ��
        % ʹ�� Z-Y-X (ƫ��-����-��ת) ��ת˳��
        cr = cos(roll(i));  sr = sin(roll(i));
        cp = cos(pitch(i)); sp = sin(pitch(i));
        cy = cos(yaw(i));   sy = sin(yaw(i));

        R = [cy*cp,   cy*sp*sr - sy*cr,   cy*sp*cr + sy*sr;
             sy*cp,   sy*sp*sr + cy*cr,   sy*sp*cr - cy*sr;
             -sp,     cp*sr,              cp*cr];

        % ����������ϵ�µļ��ٶ���ת��ȫ������ϵ
        accel_global_i = R * accel_body(i, :)';

        % ������������ȥȫ������ϵ�е��������� [0; 0; g]
        accel_global(i, :) = accel_global_i' - [0, 0, g];
    end
    
    fprintf('����任�������������\n');
    
    %% 5. �˲����һ�λ��֣��õ��ٶȣ�
    fprintf('5. ���м��ٶ��˲����ٶȻ���...\n');
    
    % ��ͨ�˲�ȥ�����ٶ��еĵ�Ƶƫ��
    fc = 0.1; % ��ֹƵ�� 0.1 Hz
    [b, a] = butter(2, fc/(Fs/2), 'high');
    accel_global_filt = filtfilt(b, a, accel_global);
    
    % ���˲����ȫ�ּ��ٶȽ��л��ֵõ��ٶ�
    velocity = cumtrapz(dt, accel_global_filt);
    
    % �򵥵�ZUPT - ���迪ʼ�ͽ���ʱ��ֹ
    zupt_start = 1:50;      % ǰ0.5��
    zupt_end = (N-49):N;    % ���0.5��
    velocity(zupt_start, :) = 0;
    velocity(zupt_end, :) = 0;
    
    fprintf('�ٶȼ������\n');
    
    %% 6. �ڶ��λ��֣��õ�λ�ƣ�
    fprintf('6. ����λ�ƻ���...\n');
    
    % ���ٶȽ��л��ֵõ�λ��
    position = cumtrapz(dt, velocity);
    
    fprintf('λ�Ƽ������\n');
    
    %% 7. ���ӻ����
    fprintf('7. ���ɽ�����ӻ�...\n');
    
    % ����һ����ͼ��
    figure('Position', [100, 100, 1400, 1000], 'Name', '����IMUλ�ƽ�����');
    
    % ����ԭʼ���ٶ�����
    subplot(3, 4, 1);
    plot(t, accel_body(:,1), 'r', t, accel_body(:,2), 'g', t, accel_body(:,3), 'b');
    title('ԭʼ���ٶ����� (��������ϵ)');
    xlabel('ʱ�� (s)');
    ylabel('���ٶ� (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % ������̬��
    subplot(3, 4, 2);
    plot(t, rad2deg(roll), 'r', t, rad2deg(pitch), 'g', t, rad2deg(yaw), 'b');
    title('�������̬��');
    xlabel('ʱ�� (s)');
    ylabel('�Ƕ� (��)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;
    
    % ����ȫ������ϵ�µļ��ٶ�
    subplot(3, 4, 3);
    plot(t, accel_global(:,1), 'r', t, accel_global(:,2), 'g', t, accel_global(:,3), 'b');
    title('ȫ������ϵ���ٶ� (��������ǰ)');
    xlabel('ʱ�� (s)');
    ylabel('���ٶ� (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % �����˲���ļ��ٶ�
    subplot(3, 4, 4);
    plot(t, accel_global_filt(:,1), 'r', t, accel_global_filt(:,2), 'g', t, accel_global_filt(:,3), 'b');
    title('�˲�������Լ��ٶ� (ȫ������ϵ)');
    xlabel('ʱ�� (s)');
    ylabel('���ٶ� (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % �����ٶ�
    subplot(3, 4, 5);
    plot(t, velocity(:,1), 'r', t, velocity(:,2), 'g', t, velocity(:,3), 'b');
    title('�ٶ�');
    xlabel('ʱ�� (s)');
    ylabel('�ٶ� (m/s)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % ����λ��
    subplot(3, 4, 6);
    plot(t, position(:,1), 'r', t, position(:,2), 'g', t, position(:,3), 'b');
    title('����λ��');
    xlabel('ʱ�� (s)');
    ylabel('λ�� (m)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % ��������λ��
    subplot(3, 4, 7);
    plot(t, position_theoretical(:,1), 'r', t, position_theoretical(:,2), 'g', t, position_theoretical(:,3), 'b');
    title('����λ��');
    xlabel('ʱ�� (s)');
    ylabel('λ�� (m)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % ����λ�����
    subplot(3, 4, 8);
    error_x = position(:,1) - position_theoretical(:,1);
    error_y = position(:,2) - position_theoretical(:,2);
    error_z = position(:,3) - position_theoretical(:,3);
    plot(t, error_x, 'r', t, error_y, 'g', t, error_z, 'b');
    title('λ�����');
    xlabel('ʱ�� (s)');
    ylabel('��� (m)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % ����3D���۹켣
    subplot(3, 4, 9);
    plot3(position_theoretical(:,1), position_theoretical(:,2), position_theoretical(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(position_theoretical(1,1), position_theoretical(1,2), position_theoretical(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(position_theoretical(end,1), position_theoretical(end,2), position_theoretical(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    hold off;
    title('����3D�켣');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('�켣', '���', '�յ�', 'Location', 'best');
    grid on;
    axis equal;
    
    % ����3D����켣
    subplot(3, 4, 10);
    plot3(position(:,1), position(:,2), position(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(position(1,1), position(1,2), position(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(position(end,1), position(end,2), position(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    hold off;
    title('����3D�켣');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('�켣', '���', '�յ�', 'Location', 'best');
    grid on;
    axis equal;
    
    % ����3D�켣�Ա�
    subplot(3, 4, [11, 12]);
    plot3(position_theoretical(:,1), position_theoretical(:,2), position_theoretical(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(position(:,1), position(:,2), position(:,3), 'r--', 'LineWidth', 1.5);
    hold off;
    title('3D�켣�Ա�');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('���۹켣', '����켣', 'Location', 'best');
    grid on;
    axis equal;
    
    % ����ͼ��
    saveas(gcf, 'imu_multiaxis_results.png');
    fprintf('����ѱ���Ϊ imu_multiaxis_results.png\n');
    
    %% 8. ��ʾ�ؼ����
    fprintf('\n=== ������ժҪ ===\n');
    
    % ��������λ�ƺ����
    final_position = position(end, :);
    final_position_theoretical = position_theoretical(end, :);
    error = final_position - final_position_theoretical;
    error_percentage = abs(error) ./ abs(final_position_theoretical) * 100;
    
    fprintf('��������λ��: X=%.3fm, Y=%.3fm, Z=%.3fm\n', final_position_theoretical);
    fprintf('��������λ��: X=%.3fm, Y=%.3fm, Z=%.3fm\n', final_position);
    fprintf('�������:     X=%.3fm, Y=%.3fm, Z=%.3fm\n', error);
    fprintf('������:     X=%.1f%%, Y=%.1f%%, Z=%.1f%%\n', error_percentage);
    
    % ����RMS���
    rms_error = sqrt(mean((position - position_theoretical).^2));
    fprintf('RMS���:      X=%.3fm, Y=%.3fm, Z=%.3fm\n', rms_error);
    
    fprintf('\n������ɣ�\n');
    
    % ��ʾ�û����ڶ����˶����ص�
    fprintf('\n�����˶��ص㣺\n');
    fprintf('1. ���ٶȡ��ٶȺ�λ������������ͬʱ����\n');
    fprintf('2. ����֮������ЧӦ��������\n');
    fprintf('3. ��̬�仯�Ը������ݵ�Ӱ����Ӹ���\n');
    fprintf('4. ��������������ͬʱ�ۻ��ʹ���\n');
end