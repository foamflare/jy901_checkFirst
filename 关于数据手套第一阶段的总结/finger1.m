clear;
clc;
%% ����Բ�Ļ滭������Բ��Ϊ0 0 0���뾶Ϊ1��
%% ��ȡ���������������ݣ�ʹ�����������ݿ���һ��Բ����ת��
figure(1)
h=0; % �߶�
t=0:0.1:(2*pi); 
t=[t,0];
xc=sin(t);
yc=cos(t);
zc=h*ones(size(t));
%% ��p1,p2,p3 ����ֱ�߽��л滭
x1=sqrt(2)/2;y1=sqrt(2)/2;z1=0;
x2=2;y2=2;z2=0;
x3=3;y3=3;z3=0;
x4=3.7;y4=3.7;z4=0;
a = 0;b = 0.6;c = 0;
getPic(a,b,c,xc,yc,zc,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4); %��ͼ����л滭
%% 
%% ��ȡ�ļ�����,��ȡ��һ���������ĳ�ʼ�����ݣ�ͬʱ�Ծ�ֹλ�ý��г�ʼ��
data = load('C:\Users\����\Desktop\acc_gy1515640006279.txt');
gyro_bias = [-0.00321799428461155,0.000250065354581674,-0.00152898838326691]';
timestamp = data(:,1)'/1000000000;
data_size = length(timestamp');
acc_s = data(:,2:4)';
gyro_s = data(:,5:7)';
g = 9.8; % Gravity.
% Initialise parameters.
% Orientation from accelerometers. Sensor is assumed to be stationary.
pitch = -asin(acc_s(1,1)/g);%���X��ĽǶ�
roll = atan(acc_s(2,1)/acc_s(3,1));%���z��ĽǶ�
yaw = 0;%���y��ĽǶ�
%��ʼ�����ڼ��ٶȴ������ķ������
%Initialize the orientation matrix C based on the accelerations
C = [cos(pitch)*cos(yaw) (sin(roll)*sin(pitch)*cos(yaw))-(cos(roll)*sin(yaw)) (cos(roll)*sin(pitch)*cos(yaw))+(sin(roll)*sin(yaw));
    cos(pitch)*sin(yaw)  (sin(roll)*sin(pitch)*sin(yaw))+(cos(roll)*cos(yaw))  (cos(roll)*sin(pitch)*sin(yaw))-(sin(roll)*cos(yaw));
    -sin(pitch) sin(roll)*cos(pitch) cos(roll)*cos(pitch)];
C_prev = C;

% Preallocate storage for heading estimate. Different from direction of
% travel, the heading indicates the direction that the sensor, and therefore
% the pedestrian, is facing.
heading = nan(1, data_size);%����1��n�еĵ�Nan������
heading(1) = yaw;

% Gyroscope bias, to be determined for each sensor.
%  -- Defined above so we don't forget to change for each dataset. --

% Preallocate storage for accelerations in navigation frame.
%�����ٶ�����תΪ��������ϵ
acc_n = nan(3, data_size);
acc_n(:,1) = C*acc_s(:,1);


% Preallocate storage for velocity (in navigation frame).
% Initial velocity assumed to be zero.
%��ʼ���ٶ�
vel_n = nan(3, data_size);
vel_n(:,1) = [0 0 0]';

% Preallocate storage for position (in navigation frame).
% Initial position arbitrarily set to the origin.
%��ʼ��λ��
pos_n = nan(3, data_size);
pos_n(:,1) = [0 0 0]';

% Preallocate storage for distance travelled used for altitude plots.
distance = nan(1,data_size-1);
distance(1) = 0;

% Error covariance matrix.
% ���Э�������
P = zeros(9); %9*9ȫ�����

% Process noise parameter, gyroscope and accelerometer noise.
% �������̶����
sigma_omega = 1e-2; sigma_a = 1e-2;

% ZUPT measurement matrix.
% ����zupt�����µĲ���ֵ
H = [zeros(3) zeros(3) eye(3)];%eye����n*n��λ���� h 

% ZUPT measurement noise covariance matrix.
% ��������
sigma_v = 1e-2;
R = diag([sigma_v sigma_v sigma_v]).^2;

% Gyroscope stance phase detection threshold.
% �����ֵ
gyro_threshold = 0.6;

% Main Loop
for t = 2:data_size
    %%% Start INS (transformation, double integration) %%%
    %��ȡʱ����
    dt = timestamp(t) - timestamp(t-1);
    
    % Remove bias from gyro measurements.
    % ���������ǵĹ̶����
    gyro_s1 = gyro_s(:,t) - gyro_bias;
    
    % Skew-symmetric matrix for angular rates
    % ���ٶȷ��Գƾ���
    ang_rate_matrix = [0   -gyro_s1(3)   gyro_s1(2);
        gyro_s1(3)  0   -gyro_s1(1);
        -gyro_s1(2)  gyro_s1(1)  0];
    
    % orientation estimation
    % �������
    C = C_prev*(2*eye(3)+(ang_rate_matrix*dt))/(2*eye(3)-(ang_rate_matrix*dt));%ͨ�����½��ƶԷ��������и���
    
    % Transforming the acceleration from sensor frame to navigation frame.
    % �����ٶ����ݴ��豸����ϵתΪ��������ϵ
    acc_n(:,t) = 0.5*(C + C_prev)*acc_s(:,t);
    
    % Velocity and position estimation using trapeze integration.
    % ͨ�����ֵõ��ٶȺ�λ��
    vel_n(:,t) = vel_n(:,t-1) + ((acc_n(:,t) - [0; 0; g] )+(acc_n(:,t-1) - [0; 0; g]))*dt/2;% v=v0+((at+a(t-1))/2)*t
    pos_n(:,t) = pos_n(:,t-1) + (vel_n(:,t) + vel_n(:,t-1))*dt/2;                           %s=s0+(vt+Vt-1)/2*t
    
    % Skew-symmetric cross-product operator matrix formed from the n-frame accelerations.
    S = [0  -acc_n(3,t)  acc_n(2,t);
        acc_n(3,t)  0  -acc_n(1,t);
        -acc_n(2,t) acc_n(1,t) 0];
    
    % State transition matrix.
    %״̬���ݾ���
    F = [eye(3)  zeros(3,3)    zeros(3,3);
        zeros(3,3)   eye(3)  dt*eye(3);
        -dt*S  zeros(3,3)    eye(3) ];
    
    % Compute the process noise covariance Q.
    %״̬ת��Э�������
    Q = diag([sigma_omega sigma_omega sigma_omega 0 0 0 sigma_a sigma_a sigma_a]*dt).^2;
    
    % Propagate the error covariance matrix.
    %��ȷ���Դ��ݣ�������һʱ�̵Ĳ�ȷ���� ������һʱ�̵Ĳ�ȷ����
    P = F*P*F' + Q;
    %%% End INS %%%
    
    % Stance phase detection and zero-velocity updates.
    if norm(gyro_s(:,t)) < gyro_threshold
        %%% Start Kalman filter zero-velocity update %%%
        % Kalman gain.
        % ����������ļ���        
        K = (P*(H)')/((H)*P*(H)' + R);
        
        % Update the filter state.
        delta_x = K*vel_n(:,t);
        
        % Update the error covariance matrix.
        %P = (eye(9) - K*(H)) * P * (eye(9) - K*(H))' + K*R*K'; % Joseph form to guarantee symmetry and positive-definiteness.
        P = (eye(9) - K*H)*P; % Simplified covariance update found in most books.
        
        % Extract errors from the KF state.
        attitude_error = delta_x(1:3);
        pos_error = delta_x(4:6);
        vel_error = delta_x(7:9);
        %%% End Kalman filter zero-velocity update %%%
        
        %%% Apply corrections to INS estimates. %%%
        % Skew-symmetric matrix for small angles to correct orientation.
        ang_matrix = -[0   -attitude_error(3,1)   attitude_error(2,1);
            attitude_error(3,1)  0   -attitude_error(1,1);
            -attitude_error(2,1)  attitude_error(1,1)  0];
        
        % Correct orientation.
        C = (2*eye(3)+(ang_matrix))/(2*eye(3)-(ang_matrix))*C;
        
        % Correct position and velocity based on Kalman error estimates.
        vel_n(:,t)=vel_n(:,t)-vel_error;
        pos_n(:,t)=pos_n(:,t)-pos_error;
    end
    heading(t) = atan2(C(2,1), C(1,1)); % Estimate and save the yaw of the sensor (different from the direction of travel). Unused here but potentially useful for orienting a GUI correctly.
    C_prev = C; % Save orientation estimate, required at start of main loop.
    %% ����ͼ��Ļ滭������ĳ����Ǿ���ZUPT����
    %% ����ͨ����ת���󣬸�����ָ��̬
    x = (C*[xc;yc;zc]);
    xc=x(1,:);yc=x(2,:);zc=x(3,:);  %�õ�Բ���ĸ���״̬
    x = (C*[a;b;c]);
     a=x(1);b=x(2);c=x(3);   %�۲�����һ����ı仯���鿴Բ�����˶�״̬
    hold off
    getPic(a,b,c,xc,yc,zc,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4);
end
