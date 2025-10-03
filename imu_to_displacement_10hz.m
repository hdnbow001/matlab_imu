function imu_to_displacement_10hz()
    % IMU_TO_DISPLACEMENT_10HZ ��IMU���ٶ����ݼ���λ�� (10Hz������)
    % �˺���ģ��IMU���ݲ���ʾ���ͨ�����λ��ּ���λ��
    
    close all; clear all; clc;
    
    %% �������� (10Hz������)
    Fs = 10;            % ������ 10 Hz
    dt = 1/Fs;          % ����ʱ����
    T = 5;              % ��ʱ�� 5��
    t = (0:dt:T-dt)';   % ʱ������
    N = length(t);
    
    fprintf('��ʼIMUλ�ƽ��� (10Hz������)...\n');
    fprintf('������: %.0f Hz\n', Fs);
    fprintf('����ʱ��: %.1f ��\n', T);
    fprintf('���ݵ���: %d\n', N);
    
    %% 1. ģ��IMU����
    fprintf('\n1. ģ��IMU����...\n');
    
    % ģ��һ���˶�����ʼ��ֹ��Ȼ���м��ٶȣ�����پ�ֹ
    accel_motion = zeros(N, 3);
    % 10Hz�£�1-2���Ӧ��11����20���㣬3-4���Ӧ��31����40����
    accel_motion(11:20, 1) = 2;       % X�᷽����1-2������ 2 m/s? �ļ��ٶ�
    accel_motion(31:40, 1) = -1.5;    % X�᷽����3-4������ -1.5 m/s? �ļ��ٶ�
    
    % IMU����ֵ = �˶����ٶ� + �������ٶ� + ���� + ƫ��
    % ʹ��NED����ϵ������Ϊ [0, 0, 9.81]
    g = 9.81;
    gravity = repmat([0, 0, g], N, 1); 
    
    % ���������ƫ�� (���ڲ����ʽ��ͣ��������ˮƽ������Ҫ����)
    bias = 0.1;                         % ���ٶȼ�ƫ��
    noise_level = 0.08;                 % ����ˮƽ (�Ը��ڸ߲��������)
    accel_bias = bias * ones(N, 3);
    accel_noise = noise_level * randn(N, 3);
    
    % �ϳ�"������"�ļ��ٶȼ����� (��������ϵ)
    accel_body = accel_motion + gravity + accel_bias + accel_noise;
    
    % ģ�����������ݣ���������̬���䣬�ʽ��ٶ�Ϊ0������������
    gyro_body = noise_level * randn(N, 3);
    
    fprintf('IMU����ģ�����\n');
    
    %% 2. ��̬���㣨ʹ�û����˲���
    fprintf('2. ������̬���㣨�����˲���...\n');
    
    % ��ʼ����̬�ǣ���ת��������ƫ����
    pitch = zeros(N, 1);
    roll = zeros(N, 1);
    yaw = zeros(N, 1);
    
    % �����˲�ϵ�� (������Ҫ��������Ӧ�Ͳ�����)
    alpha = 0.95; % ��΢���������ǵ�Ȩ��
    
    for i = 2:N
        % �������ǻ��ֵõ���̬�����׻��֣�
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
    
    %% 3. ����任����������
    fprintf('3. ��������任����������...\n');
    
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
    
    %% 4. �˲����һ�λ��֣��õ��ٶȣ�
    fprintf('4. ���м��ٶ��˲����ٶȻ���...\n');
    
    % ��ͨ�˲�ȥ�����ٶ��еĵ�Ƶƫ��
    % ���ڲ����ʽ��ͣ���ֹƵ����Ҫ��Ӧ����
    fc = 0.05; % ��ֹƵ�� 0.05 Hz (�����10Hz������)
    [b, a] = butter(2, fc/(Fs/2), 'high');
    accel_global_filt = filtfilt(b, a, accel_global);
    
    % ���˲����ȫ�ּ��ٶȽ��л��ֵõ��ٶ�
    velocity = cumtrapz(dt, accel_global_filt);
    
    % �����˶���ʼ�ͽ������Ǿ�ֹ�ģ����Խ����ٶ����㣨һ�ּ򵥵�ZUPT��
    % 10Hz�£�ǰ0.5���Ӧǰ5����
    velocity(1:5, :) = 0;    % ����ǰ0.5�뾲ֹ
    velocity(end-4:end, :) = 0; % �������0.5�뾲ֹ
    
    fprintf('�ٶȼ������\n');
    
    %% 5. �ڶ��λ��֣��õ�λ�ƣ�
    fprintf('5. ����λ�ƻ���...\n');
    
    % ���ٶȽ��л��ֵõ�λ��
    position = cumtrapz(dt, velocity);
    
    fprintf('λ�Ƽ������\n');
    
    %% 6. ���ӻ����
    fprintf('6. ���ɽ�����ӻ�...\n');
    
    % ����һ����ͼ��
    figure('Position', [100, 100, 1200, 800], 'Name', 'IMUλ�ƽ����� (10Hz������)');
    
    % ����ԭʼ���ٶ�����
    subplot(3, 3, 1);
    plot(t, accel_body(:,1), 'r', t, accel_body(:,2), 'g', t, accel_body(:,3), 'b');
    title('ԭʼ���ٶ����� (��������ϵ)');
    xlabel('ʱ�� (s)');
    ylabel('���ٶ� (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % ������̬��
    subplot(3, 3, 2);
    plot(t, rad2deg(roll), 'r', t, rad2deg(pitch), 'g', t, rad2deg(yaw), 'b');
    title('�������̬��');
    xlabel('ʱ�� (s)');
    ylabel('�Ƕ� (��)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;
    
    % ����ȫ������ϵ�µļ��ٶ�
    subplot(3, 3, 3);
    plot(t, accel_global(:,1), 'r', t, accel_global(:,2), 'g', t, accel_global(:,3), 'b');
    title('ȫ������ϵ���ٶ� (��������ǰ)');
    xlabel('ʱ�� (s)');
    ylabel('���ٶ� (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % �����˲���ļ��ٶ�
    subplot(3, 3, 4);
    plot(t, accel_global_filt(:,1), 'r', t, accel_global_filt(:,2), 'g', t, accel_global_filt(:,3), 'b');
    title('�˲�������Լ��ٶ� (ȫ������ϵ)');
    xlabel('ʱ�� (s)');
    ylabel('���ٶ� (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % �����ٶ�
    subplot(3, 3, 5);
    plot(t, velocity(:,1), 'r', t, velocity(:,2), 'g', t, velocity(:,3), 'b');
    title('�ٶ�');
    xlabel('ʱ�� (s)');
    ylabel('�ٶ� (m/s)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % ����λ��
    subplot(3, 3, 6);
    plot(t, position(:,1), 'r', t, position(:,2), 'g', t, position(:,3), 'b');
    title('λ��');
    xlabel('ʱ�� (s)');
    ylabel('λ�� (m)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % ����3D�켣
    subplot(3, 3, [7, 8, 9]);
    plot3(position(:,1), position(:,2), position(:,3), 'b-', 'LineWidth', 1.5);
    hold on;
    plot3(position(1,1), position(1,2), position(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(position(end,1), position(end,2), position(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    hold off;
    title('3D�켣 (10Hz������)');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('�켣', '���', '�յ�', 'Location', 'best');
    grid on;
    axis equal;
    
    % ����ͼ��
    saveas(gcf, 'imu_displacement_results_10hz.png');
    fprintf('����ѱ���Ϊ imu_displacement_results_10hz.png\n');
    
    %% 7. ��ʾ�ؼ����
    fprintf('\n=== ������ժҪ (10Hz������) ===\n');
    fprintf('����λ��: X=%.3fm, Y=%.3fm, Z=%.3fm\n', ...
            position(end,1), position(end,2), position(end,3));
    fprintf('���λ��: X=%.3fm, Y=%.3fm, Z=%.3fm\n', ...
            max(abs(position(:,1))), max(abs(position(:,2))), max(abs(position(:,3))));
    
    % ����������ֵ������Ϊ����֪��ģ����˶���
    % ����10Hz�����ʣ�������Ҫ���¼�������ֵ
    theoretical_velocity_x = cumtrapz(dt, accel_motion(:,1));
    theoretical_x_displacement = cumtrapz(dt, theoretical_velocity_x);
    theoretical_x_displacement = theoretical_x_displacement(end);
    
    actual_x_displacement = position(end,1);
    error_percentage = abs(actual_x_displacement - theoretical_x_displacement) / abs(theoretical_x_displacement) * 100;
    
    fprintf('X��������λ��: %.3fm\n', theoretical_x_displacement);
    fprintf('X����ʵ��λ��: %.3fm\n', actual_x_displacement);
    fprintf('X�������: %.1f%%\n', error_percentage);
    
    fprintf('\n������ɣ�\n');
    
    % ��ʾ�û����ڵͲ����ʵ�Ӱ��
    fprintf('\nע�⣺10Hz�����������100Hz�����ʣ����ܻᵼ������Ӱ�죺\n');
    fprintf('1. ���־��Ƚ��ͣ��ر��Ƕ��ڿ��ٱ仯���˶�\n');
    fprintf('2. �˲��������½���������Ҫ�����˲�������\n');
    fprintf('3. ��̬�������Ӧ�ٶȱ���\n');
    fprintf('4. �˶�ϸ�ڿ��ܶ�ʧ���ر��Ǹ�Ƶ�ɷ�\n');
    fprintf('��ʵ��Ӧ���У�Ӧ�����˶����Ժ;���Ҫ��ѡ����ʵĲ����ʡ�\n');
end