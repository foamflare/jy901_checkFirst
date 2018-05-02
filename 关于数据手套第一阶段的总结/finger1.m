clear;
clc;
%% 进行圆的绘画，其中圆心为0 0 0，半径为1，
%% 读取传感器陀螺仪数据，使用陀螺仪数据控制一个圆环的转动
figure(1)
h=0; % 高度
t=0:0.1:(2*pi); 
t=[t,0];
xc=sin(t);
yc=cos(t);
zc=h*ones(size(t));
%% 对p1,p2,p3 三条直线进行绘画
x1=sqrt(2)/2;y1=sqrt(2)/2;z1=0;
x2=2;y2=2;z2=0;
x3=3;y3=3;z3=0;
x4=3.7;y4=3.7;z4=0;
a = 0;b = 0.6;c = 0;
getPic(a,b,c,xc,yc,zc,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4); %对图像进行绘画
%% 
%% 读取文件数据,获取第一个传感器的初始化数据，同时对静止位置进行初始化
data = load('C:\Users\刘武\Desktop\acc_gy1515640006279.txt');
gyro_bias = [-0.00321799428461155,0.000250065354581674,-0.00152898838326691]';
timestamp = data(:,1)'/1000000000;
data_size = length(timestamp');
acc_s = data(:,2:4)';
gyro_s = data(:,5:7)';
g = 9.8; % Gravity.
% Initialise parameters.
% Orientation from accelerometers. Sensor is assumed to be stationary.
pitch = -asin(acc_s(1,1)/g);%获得X轴的角度
roll = atan(acc_s(2,1)/acc_s(3,1));%获得z轴的角度
yaw = 0;%获得y轴的角度
%初始化基于加速度传感器的方向矩阵
%Initialize the orientation matrix C based on the accelerations
C = [cos(pitch)*cos(yaw) (sin(roll)*sin(pitch)*cos(yaw))-(cos(roll)*sin(yaw)) (cos(roll)*sin(pitch)*cos(yaw))+(sin(roll)*sin(yaw));
    cos(pitch)*sin(yaw)  (sin(roll)*sin(pitch)*sin(yaw))+(cos(roll)*cos(yaw))  (cos(roll)*sin(pitch)*sin(yaw))-(sin(roll)*cos(yaw));
    -sin(pitch) sin(roll)*cos(pitch) cos(roll)*cos(pitch)];
C_prev = C;

% Preallocate storage for heading estimate. Different from direction of
% travel, the heading indicates the direction that the sensor, and therefore
% the pedestrian, is facing.
heading = nan(1, data_size);%产生1行n列的的Nan的数据
heading(1) = yaw;

% Gyroscope bias, to be determined for each sensor.
%  -- Defined above so we don't forget to change for each dataset. --

% Preallocate storage for accelerations in navigation frame.
%将加速度数据转为导航坐标系
acc_n = nan(3, data_size);
acc_n(:,1) = C*acc_s(:,1);


% Preallocate storage for velocity (in navigation frame).
% Initial velocity assumed to be zero.
%初始化速度
vel_n = nan(3, data_size);
vel_n(:,1) = [0 0 0]';

% Preallocate storage for position (in navigation frame).
% Initial position arbitrarily set to the origin.
%初始化位置
pos_n = nan(3, data_size);
pos_n(:,1) = [0 0 0]';

% Preallocate storage for distance travelled used for altitude plots.
distance = nan(1,data_size-1);
distance(1) = 0;

% Error covariance matrix.
% 误差协方差矩阵
P = zeros(9); %9*9全零矩阵

% Process noise parameter, gyroscope and accelerometer noise.
% 传感器固定误差
sigma_omega = 1e-2; sigma_a = 1e-2;

% ZUPT measurement matrix.
% 依靠zupt产生新的测量值
H = [zeros(3) zeros(3) eye(3)];%eye返回n*n单位矩阵 h 

% ZUPT measurement noise covariance matrix.
% 测量噪声
sigma_v = 1e-2;
R = diag([sigma_v sigma_v sigma_v]).^2;

% Gyroscope stance phase detection threshold.
% 检测阈值
gyro_threshold = 0.6;

% Main Loop
for t = 2:data_size
    %%% Start INS (transformation, double integration) %%%
    %获取时间间隔
    dt = timestamp(t) - timestamp(t-1);
    
    % Remove bias from gyro measurements.
    % 减掉陀螺仪的固定误差
    gyro_s1 = gyro_s(:,t) - gyro_bias;
    
    % Skew-symmetric matrix for angular rates
    % 角速度反对称矩阵
    ang_rate_matrix = [0   -gyro_s1(3)   gyro_s1(2);
        gyro_s1(3)  0   -gyro_s1(1);
        -gyro_s1(2)  gyro_s1(1)  0];
    
    % orientation estimation
    % 方向估计
    C = C_prev*(2*eye(3)+(ang_rate_matrix*dt))/(2*eye(3)-(ang_rate_matrix*dt));%通过帕德近似对方向矩阵进行更新
    
    % Transforming the acceleration from sensor frame to navigation frame.
    % 将加速度数据从设备坐标系转为导航坐标系
    acc_n(:,t) = 0.5*(C + C_prev)*acc_s(:,t);
    
    % Velocity and position estimation using trapeze integration.
    % 通过积分得到速度和位移
    vel_n(:,t) = vel_n(:,t-1) + ((acc_n(:,t) - [0; 0; g] )+(acc_n(:,t-1) - [0; 0; g]))*dt/2;% v=v0+((at+a(t-1))/2)*t
    pos_n(:,t) = pos_n(:,t-1) + (vel_n(:,t) + vel_n(:,t-1))*dt/2;                           %s=s0+(vt+Vt-1)/2*t
    
    % Skew-symmetric cross-product operator matrix formed from the n-frame accelerations.
    S = [0  -acc_n(3,t)  acc_n(2,t);
        acc_n(3,t)  0  -acc_n(1,t);
        -acc_n(2,t) acc_n(1,t) 0];
    
    % State transition matrix.
    %状态传递矩阵
    F = [eye(3)  zeros(3,3)    zeros(3,3);
        zeros(3,3)   eye(3)  dt*eye(3);
        -dt*S  zeros(3,3)    eye(3) ];
    
    % Compute the process noise covariance Q.
    %状态转移协方差矩阵
    Q = diag([sigma_omega sigma_omega sigma_omega 0 0 0 sigma_a sigma_a sigma_a]*dt).^2;
    
    % Propagate the error covariance matrix.
    %不确定性传递，根据上一时刻的不确定性 估计这一时刻的不确定性
    P = F*P*F' + Q;
    %%% End INS %%%
    
    % Stance phase detection and zero-velocity updates.
    if norm(gyro_s(:,t)) < gyro_threshold
        %%% Start Kalman filter zero-velocity update %%%
        % Kalman gain.
        % 卡尔曼增益的计算        
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
    %% 进行图像的绘画，上面的程序是经典ZUPT程序
    %% 下面通过旋转矩阵，更新手指姿态
    x = (C*[xc;yc;zc]);
    xc=x(1,:);yc=x(2,:);zc=x(3,:);  %得到圆环的更新状态
    x = (C*[a;b;c]);
     a=x(1);b=x(2);c=x(3);   %观察其中一个点的变化，查看圆环的运动状态
    hold off
    getPic(a,b,c,xc,yc,zc,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4);
end
