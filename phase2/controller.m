function [F, M, trpy, drpy] = controller(qd, ~, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw

% Define the gains
kp_1 = 3.6;  %6
kp_2 = 3.6;%6.5; %5.6
kp_3 = 70;%13;%6
kd_1 = 9;%6.0; %5.6
kd_2 = 9;%6.0; % 6.2
kd_3 = 30;%5; %9
kp_phi = 0.6;
kp_theta = 0.6;
kp_psi = 0.02; 
kd_phi = 0.06; %0.08
kd_theta = 0.06; %0.08
kd_psi = 0.02;


% Define p,q & r desired
p_des = 0;
q_des = 0;
r_des = qd{qn}.yawdot_des;

% Calculate r_1_com_acc
r_1_com_acc = qd{qn}.acc_des(1) + kd_1*(qd{qn}.vel_des(1) - qd{qn}.vel(1)) + kp_1*(qd{qn}.pos_des(1) - qd{qn}.pos(1));

% Calculate r_2_com_acc
r_2_com_acc = qd{qn}.acc_des(2) + kd_2*(qd{qn}.vel_des(2) - qd{qn}.vel(2)) + kp_2*(qd{qn}.pos_des(2) - qd{qn}.pos(2));

% Calculate r_2_com_acc
r_3_com_acc = qd{qn}.acc_des(3) + kd_3*(qd{qn}.vel_des(3) - qd{qn}.vel(3)) + kp_3*(qd{qn}.pos_des(3) - qd{qn}.pos(3));

% Find theta_des and phi_des
phi_des = (r_1_com_acc*sin(qd{qn}.yaw_des) - r_2_com_acc*cos(qd{qn}.yaw_des))/params.grav;
theta_des = (r_1_com_acc*cos(qd{qn}.yaw_des) + r_2_com_acc*sin(qd{qn}.yaw_des))/params.grav;
psi_des = qd{qn}.yaw_des;

% Thrust
F    = params.mass*params.grav + params.mass*r_3_com_acc; %paramss.mass*paramss.grav;

% Moment
M(1,1)    = kp_phi*(phi_des - qd{qn}.euler(1)) + kd_phi*(p_des - qd{qn}.omega(1));
M(2,1)    = kp_theta*(theta_des - qd{qn}.euler(2)) + kd_theta*(q_des - qd{qn}.omega(2));
M(3,1)    = kp_psi*(psi_des - qd{qn}.euler(3)) + kd_psi*(r_des - qd{qn}.omega(3)); % You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];
% current = [qd{qn}.pos_des(1) qd{qn}.pos_des(2) qd{qn}.pos_des(3)];
% actual = [qd{qn}.pos(1) qd{qn}.pos(2) qd{qn}.pos(3)];
% 
% error = error + pdist([current;actual])
end

