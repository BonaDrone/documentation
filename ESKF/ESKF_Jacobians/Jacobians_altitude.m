clc; clear
%% Define States
syms pz
syms vz
syms qw qx qy qz
syms wbx wby wbz

syms dpz
syms dvz
syms dwx dwy dwz
syms dwbx dwby dwbz


syms g dt

% Nominal states
p  = pz;
v  = vz;
qv = [qx; qy; qz];
q  = [qw; qv];
wb = [wbx; wby; wbz];

%x = [p; v; q; ab; wb];
x = [p; v; q; wb];

gv = [0;0;-g];

% Error states
dp  = dpz;
dv  = dvz;
dw  = [dwx; dwy; dwz];
dwb = [dwbx; dwby; dwbz];

dx = [dp; dv; dw; dwb];

% Measurements - IMU
syms asx asy asz
syms wsx wsy wsz

as = [asx; asy; asz];
ws = [wsx; wsy; wsz];

%% Nominal Jacobian

%V = fromqtoR(q)*(as-ab);
V = fromqtoR(q)*as;
V = jacobian(V, q);
R = -fromqtoR(q);
W = 0.5*[0, -transpose(ws-wb); ws-wb, -skew(ws-wb)];
Q = -0.5*Qmat(q);

% gv = (as-ab);
% 2*[qw*gv+cross(qv,gv), qv*transpose(gv)-gv*transpose(qv)+transpose(gv)*qv*eye(3)-qw*skew(gv)]

%A_x = ...
%    [zeros(3) eye(3)   zeros(3, 4) zeros(3) zeros(3); ...
%    zeros(3) zeros(3) V           -R        zeros(3); ...
%    zeros(3) zeros(3) W           zeros(3) Q       ; ...
%    zeros(6, 16)];

A_x = ...
    [0               1    zeros(1, 4)  zeros(1, 3); ...
     0               0      V(3,:)     zeros(1,3); ...
    zeros(3,1)   zeros(3,1)    W             Q       ; ...
    zeros(3, 9)];

F_x = eye(9) + A_x*dt;

%% Error-State Jacobian

%V  = -fromqtoR(q)*skew(as-ab);
V = -fromqtoR(q)*skew(as);
R  = fromqtoR(q);
Fi = -skew(ws-wb);

A_dx = ...
    [0              1      zeros(1, 3)  zeros(1, 3); ...
     0              0         V(3,:)    zeros(1, 3); ...
    zeros(3, 1)  zeros(3,1)     Fi        -eye(3);  ...
    zeros(3, 8)];

F_dx = eye(8) + A_dx*dt;


%% h(x) - accel. Jacobian of h(x) w.r.t the quaternion

R_t = transpose(fromqtoR(q));
R_t_g = -R_t*gv;
H_x = jacobian(R_t_g, q);
H_x = blkdiag(zeros(2), H_x, zeros(3));
X_dx = Qmat(q);
X_dx = blkdiag(eye(2), X_dx, eye(3));
H_dx_a = H_x*X_dx;

%% h(x) - Range Finder

R_r_i = [1 0 0; 0 -1 0; 0 0 -1];
p_r_i = [0 0 0]; % measure distances
R = fromqtoR(q);

p_r_w = [0; 0; pz] + R*transpose(p_r_i);
R_r_w = R*R_r_i;

h_r = p_r_w(3)/R_r_w(3,3); 
H_r = jacobian(h_r, x);
X_dx = Qmat(q);
X_dx = blkdiag(eye(2), X_dx, eye(3));
H_dx_r = H_r*X_dx;