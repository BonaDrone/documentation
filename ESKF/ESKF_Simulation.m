clear
close all
clc

addpath('data');
addpath('./ESKF_Jacobians/functions');

sens_data = load('data/sensors_data.mat', 'a_s', 'w_s', 'range_s', 't_imu');
resu_data = load('data/quadodom_results.mat', 'robots');

imu_a = sens_data.a_s;
imu_w = sens_data.w_s;
range = sens_data.range_s;
t     = sens_data.t_imu;

% Ground truth
XT = resu_data.robots(2).state;

N = 9; % Number of states

XX = zeros(N, size(imu_a, 2) + 1); % State container
PP = zeros(N-1, N-1, size(imu_a, 2) + 1); % Covariance container

% Initial nominal states
z0  = 0;
vz0 = 0;
q0  = [1, 0, 0, 0]';
wb0 = ones(3,1)*0.001;

x = [z0; vz0; q0; wb0]; % Initial state
P = 1*eye(N-1);

XX(:, 1)   = x;
PP(:, :, 1) = P;

% Sensor parameter
noise_gyro      = 0.1;
noise_gyro_bias = 0.0001;
noise_accel     = 10;
noise_accel_est = 0.1;
noise_range     = 0.1;

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
    pz = x(1);
    vz = x(2);
    qw = x(3);
    qv = x(4:6);
    wb = x(7:9);
    q = [qw; qv];
    
%     W = 0.5*[0, -transpose(w - wb); (w - wb), -skew(w - wb)];
%     Q = -0.5*Qmat(q);
%     V = 2*[qw*a + cross(qv, a), qv*a' - a*qv' + a'*qv*eye(3) - qw*skew(a)];
%     
%     A_x = ...
%         [0 1 zeros(1, N-2);...
%         0 0 V(3, :) zeros(1,3);...
%         zeros(4,2), W, Q; ...
%         zeros(3, N)];
%     
%     F_x = eye(size(A_x,1)) + A_x*dt;
%     
%     x = F_x*x;
    
    R = fromqtoR(q);
    aux    = R*a + [0;0;-gravity];
    q_aux  = [1;(w-wb)*dt/2];
    x(1)   = pz + vz*dt;
    x(2)   = vz + aux(3)*dt;
    x(3:6) = leftQuaternion(q)*q_aux;
    x(7:9) = wb;
    
    pz = x(1);
    vz = x(2);
    qw = x(3);
    qv = x(4:6);
    wb = x(7:9);
    q = [qw; qv];
    
    % Error-State Jacobian
    Fi = -skew(w-wb);
    V  = -fromqtoR(q)*skew(a);
    A_dx = ...
        [0, 1, zeros(1, N-3);...
        0, 0, V(3, :), zeros(1,3);...
        zeros(3,2), Fi, -eye(3);  ...
        zeros(3,N-1)];
    
    F_dx = eye(N-1) + A_dx*dt;
    
    % Covariance matrix
    F_i = [zeros(1,7); eye(7)];    
    
    Fi_i = noise_gyro^2*dt^2*eye(3);
    Om_i = noise_gyro_bias^2*dt*eye(3);
    a_i  = noise_accel_est^2*dt^2;
    Q_i  = blkdiag(a_i,Fi_i, Om_i);
    
    % State update
    P = F_dx*P*F_dx' + F_i*Q_i*F_i';
    
    % Correction Accelerometer
    
    % Measurement Jacobiansx(1:4)
    H_x = 2*[-qv(2)    qv(3)    -qw   qv(1); ...
        qv(1)    qw     qv(3)   qv(2); ...
        qw   -qv(1)    -qv(2)   qv(3)];
    H_x = [zeros(3, 2), H_x, zeros(3)];
    X_dx = Qmat(q);
    X_dx = blkdiag(eye(2), X_dx, eye(3));
    H_dx = H_x*X_dx;
    
    Na = noise_accel^2*eye(3);
    Z = H_dx*P*H_dx' + Na;
    K = P*H_dx'/Z;
    
    R = fromqtoR(q);
    a_est = -R'*[0;0;-gravity];
    z = a - a_est;
    dx = K*z;
    S = eye(N-1) - K*H_dx;
    P = S*P*S' + K*Z*K';
    
    % TODO: Implement
    % the altitude correction using the range
    
    % Reset operation
    x(1)   = pz + dx(1);
    x(2)   = vz + dx(2);
    x(3:6) = leftQuaternion(q)*[1; dx(3:5)/2];
    x(7:9) = wb + dx(6:8);
    
    dx = zeros(N-1,1);
    
    % Fill containers
    XX(:, ii+1) = x;
    PP(:, :, ii+1) = P;
    
end

[psi, theta, phi] = quat2angle(XX(3:6, :)');
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