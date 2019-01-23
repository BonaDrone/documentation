clear
close all
clc

addpath('data');
addpath('functions');

sens_data = load('data/sensors_data.mat', 'a_s', 'w_s', 't_imu');
resu_data = load('data/quadodom_results.mat', 'robots');

imu_a = sens_data.a_s;
imu_w = sens_data.w_s;
t     = sens_data.t_imu;

% Ground truth
XT = resu_data.robots(2).state;

N = 7; % Number of states

XX = zeros(N, size(imu_a, 2) + 1); % State container
PP = zeros(N-1, N-1, size(imu_a, 2) + 1); % Covariance container

x = [1, 0, 0, 0, 0.001 0.001 0.001]'; % Initial state
P = 1*eye(N-1);

XX(:, 1)   = x;
PP(:, :, 1) = P; 

% Sensor parameter
noise_gyro      = 0.01;
noise_gyro_bias = 0.001;
noise_accel     = 2;

% Other parameters
gravity = 9.81007;

for ii=1:size(imu_a, 2)
    if ii==1
        dt = t(1);
    else
        dt = t(ii) - t(ii-1);
    end
    
    a = imu_a(:, ii);
    w = imu_w(:, ii);
    
    % Nominal Jacobian
    W = 0.5*[0, -transpose(w - x(5:7)); (w - x(5:7)), -skew(w - x(5:7))];
    Q = -0.5*Qmat(x(1:4));
    
    A_x = ...
        [W, Q; ...
        zeros(3, 7)];
    
    F_x = eye(size(A_x,1)) + A_x*dt;
    
    x = F_x*x;
    % Error-State Jacobian
    Fi = -skew(w-x(5:7));
    
    A_dx = ...
        [Fi, -eye(3);  ...
        zeros(3,6)];
    
    F_dx = eye(6) + A_dx*dt;
    
    % Covariance matrix
    F_i = eye(6);
    
    Fi_i = noise_gyro^2*dt^2*eye(3);
    Om_i = noise_gyro_bias^2*dt*eye(3);
    Q_i  = blkdiag(Fi_i, Om_i);
    
    % State update
    P = F_dx*P*F_dx' + F_i*Q_i*F_i';
    
    % Correction Accelerometer
    
    % Measurement Jacobians
    H_x = 2*[-x(3)    x(4)    -x(1)   x(2); ...
            x(2)    x(1)     x(4)   x(3); ...
            x(1)   -x(2)    -x(3)   x(4)];
    H_x = [H_x, zeros(3)];    
    X_dx = Qmat(x);
    X_dx = blkdiag(X_dx, eye(3));
    H_dx = H_x*X_dx;
    
    Na = noise_accel^2*eye(3);
    Z = H_dx*P*H_dx' + Na;
    K = P*H_dx'/Z;
    
    R = fromqtoR(x(1:4));
    a_est = -R'*[0;0;-gravity];
    z = a - a_est;
    dx = K*z;
    S = eye(6) - K*H_dx;
    P = S*P*S' + K*Z*K';
        
    % Reset operation
    x(1:4) = leftQuaternion(x(1:4))*[1; dx(1:3)/2];
    x(5:7) = x(5:7) + dx(4:6);    
        
    dx = zeros(3,1);
    
    % Fill containers
    XX(:, ii+1) = x;
    PP(:, :, ii+1) = P;    
    
end

[psi, theta, phi] = quat2angle(XX(1:4, :)');
[psiT, thetaT, phiT] = quat2angle(XT(7:10, :)');

figure
subplot(3, 1, 1);
plot(t, [psi(2:end), psiT]);
title('Orientation')
subplot(3, 1, 2);
plot(t, 180/pi*[theta(2:end), thetaT]);
subplot(3, 1, 3);
plot(t, 180/pi*[phi(2:end), phiT]);

figure
subplot(3, 1, 1);
plot(t, [XX(5,2:end); XT(14,:)]);
title('Bias')
subplot(3, 1, 2);
plot(t, [XX(6,2:end); XT(15,:)]);
subplot(3, 1, 3);
plot(t, [XX(7,2:end); XT(16,:)]);