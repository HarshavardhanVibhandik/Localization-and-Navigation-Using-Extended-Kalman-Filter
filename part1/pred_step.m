function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%covarPrev and uPrev are the previous mean and covariance respectively
%angVel is the angular velocity
%acc is the acceleration
%dt is the sampling time


%To extract the previous state values
vel_x = uPrev(7,1);
vel_y = uPrev(8,1);
vel_z = uPrev(9,1);

roll = uPrev(4,1);
pitch = uPrev(5,1);
yaw = uPrev(6,1);

bias_gyro_x = uPrev(10,1);
bias_gyro_y = uPrev(11,1);
bias_gyro_z = uPrev(12,1);

bias_acc_x = uPrev(13,1);
bias_acc_y = uPrev(14,1);
bias_acc_z = uPrev(15,1);

wmx = angVel(1,1);
wmy = angVel(2,1);
wmz = angVel(3,1);

amx = acc(1,1);
amy = acc(2,1);
amz = acc(3,1);


%x_dot is explained in the Lecture 7 notes: page no: 34 
%x_dot = [x3 ; G_x2_inv_xyz*Rot_zyx*(wm - x4 - ng) ; g+(Rot_zyx)*(am - x5 - na); nbg ; nba];  T

%G is explained in the Lecture 7 notes: page no: 27
% G_x2_inv =[(cos(yaw)*sin(pitch))/cos(pitch) (sin(pitch)*sin(yaw))/cos(pitch) 1 ; -sin(yaw) cos(yaw) 0; cos(yaw)/cos(pitch) sin(yaw)/cos(pitch) 0];  

%We need to flip (G_x2_inv) this matrix as we need the state matrix in the form of XYZ and we have it in the form ZYX euler orientation
% G_x2_inv_xyz = flip(G_x2_inv);

%Rotation is explained in the Lecture 7 notes: page no: 25
% Rot_x = [1 0 0 ; 0 cos(roll) -sin(roll) ; 0 sin(roll) cos(roll)]
% Rot_y = [cos(pitch) 0 sin(pitch) ; 0 1 0 ; -sin(pitch) 0 cos(pitch)]
% Rot_z = [cos(yaw) -sin(yaw) 0 ; sin(yaw) cos(yaw) 0 ;0 0 1]

% Rot_zyx = simplify(Rz*Ry*Rx)

x_dot = [vel_x; vel_y; vel_z; 
         ((sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch) - (cos(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch))*(bias_gyro_z - wmz) - ((sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch) - (cos(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch))*(bias_gyro_y - wmy) - (bias_gyro_x - wmx)*(cos(yaw)^2 + sin(yaw)^2); 
         (bias_gyro_z - wmz)*(cos(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + sin(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))) - (bias_gyro_y - wmy)*(cos(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + sin(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))); 
         - (bias_gyro_z - wmz)*(cos(pitch)*cos(roll) + (cos(yaw)*sin(pitch)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch) - (sin(pitch)*sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch)) - (bias_gyro_y - wmy)*(cos(pitch)*sin(roll) - (cos(yaw)*sin(pitch)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch) + (sin(pitch)*sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch)) - (bias_gyro_x - wmx)*(sin(pitch)*cos(yaw)^2 + sin(pitch)*sin(yaw)^2 - sin(pitch)); 
         (amz - bias_acc_z)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - (amy - bias_acc_y)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + cos(pitch)*cos(yaw)*(amx - bias_acc_x); 
         (amy - bias_acc_y)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - (amz - bias_acc_z)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + cos(pitch)*sin(yaw)*(amx - bias_acc_x); 
         cos(pitch)*cos(roll)*(amz - bias_acc_z) - sin(pitch)*(amx - bias_acc_x) + cos(pitch)*sin(roll)*(amy - bias_acc_y) - (981/100); 
         (0); (0); (0); (0); (0); (0)];

uEst  = uPrev + dt* x_dot; %To update the estimated state


%Linearization: is explained in the Lecture 7 notes: page no: 40

%Jacobian of x_dot wrt state variable
%A = jacobian(x_dot,x)
A = [(0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0), (0), (0); (0), (0), (0), ((sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch) - (cos(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch))*(bias_gyro_y - wmy) + ((sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch) - (cos(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch))*(bias_gyro_z - wmz), - (bias_gyro_z - wmz)*(cos(roll)*cos(yaw)^2 + cos(roll)*sin(yaw)^2 + (cos(yaw)*sin(pitch)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch)^2 - (sin(pitch)*sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch)^2) - (bias_gyro_y - wmy)*(cos(yaw)^2*sin(roll) + sin(roll)*sin(yaw)^2 - (cos(yaw)*sin(pitch)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch)^2 + (sin(pitch)*sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch)^2), (0), (0), (0), (0), - cos(yaw)^2 - sin(yaw)^2, (cos(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch) - (sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch), ((sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch) - (cos(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch)), (0), (0), (0); (0), (0), (0), (bias_gyro_y - wmy)*(cos(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + sin(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))) + (bias_gyro_z - wmz)*(cos(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + sin(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))), (0), (0), (0), (0), (0), (0), - cos(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - sin(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)), (cos(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + sin(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))), (0), (0), (0); (0), (0), (0), (bias_gyro_z - wmz)*(cos(pitch)*sin(roll) - (cos(yaw)*sin(pitch)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch) + (sin(pitch)*sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch)) - (bias_gyro_y - wmy)*(cos(pitch)*cos(roll) + (cos(yaw)*sin(pitch)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch) - (sin(pitch)*sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch)), - (bias_gyro_z - wmz)*(cos(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - cos(roll)*sin(pitch) - sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + cos(roll)*cos(yaw)^2*sin(pitch) + cos(roll)*sin(pitch)*sin(yaw)^2 + (cos(yaw)*sin(pitch)^2*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch)^2 - (sin(pitch)^2*sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch)^2) - (bias_gyro_y - wmy)*(sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - cos(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - sin(pitch)*sin(roll) + cos(yaw)^2*sin(pitch)*sin(roll) + sin(pitch)*sin(roll)*sin(yaw)^2 - (cos(yaw)*sin(pitch)^2*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch)^2 + (sin(pitch)^2*sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch)^2) - (bias_gyro_x - wmx)*(cos(pitch)*cos(yaw)^2 + cos(pitch)*sin(yaw)^2 - cos(pitch)), (0), (0), (0), (0), - sin(pitch)*cos(yaw)^2 - sin(pitch)*sin(yaw)^2 + sin(pitch), (cos(yaw)*sin(pitch)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch) - cos(pitch)*sin(roll) - (sin(pitch)*sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch), (sin(pitch)*sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch) - (cos(yaw)*sin(pitch)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch) - cos(pitch)*cos(roll), (0), (0), (0); (0), (0), (0), (amy - bias_acc_y)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + (amz - bias_acc_z)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)), cos(pitch)*cos(roll)*cos(yaw)*(amz - bias_acc_z) - cos(yaw)*sin(pitch)*(amx - bias_acc_x) + cos(pitch)*cos(yaw)*sin(roll)*(amy - bias_acc_y), (amz - bias_acc_z)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - (amy - bias_acc_y)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - cos(pitch)*sin(yaw)*(amx - bias_acc_x), (0), (0), (0), (0), (0), (0), -cos(pitch)*cos(yaw), (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)), - sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch); (0), (0), (0), - (amy - bias_acc_y)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - (amz - bias_acc_z)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)), cos(pitch)*cos(roll)*sin(yaw)*(amz - bias_acc_z) - sin(pitch)*sin(yaw)*(amx - bias_acc_x) + cos(pitch)*sin(roll)*sin(yaw)*(amy - bias_acc_y), (amz - bias_acc_z)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - (amy - bias_acc_y)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + cos(pitch)*cos(yaw)*(amx - bias_acc_x), (0), (0), (0), (0), (0), (0), -cos(pitch)*sin(yaw), - cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw), (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)); (0), (0), (0), cos(pitch)*cos(roll)*(amy - bias_acc_y) - cos(pitch)*sin(roll)*(amz - bias_acc_z), - cos(pitch)*(amx - bias_acc_x) - cos(roll)*sin(pitch)*(amz - bias_acc_z) - sin(pitch)*sin(roll)*(amy - bias_acc_y), (0), (0), (0), (0), (0), (0), (0), sin(pitch), -cos(pitch)*sin(roll), -cos(pitch)*cos(roll); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0)];

%Transition Matrix
F = eye(15,15) + dt* A;

%Jacobian of x_dot wrt noise variable
%U = jacobian(x_dot,n)
U = [(0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); - cos(yaw)^2 - sin(yaw)^2, (cos(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch) - (sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch), (sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch) - (cos(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), - cos(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - sin(yaw)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)), cos(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + sin(yaw)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)), (0), (0), (0), (0), (0), (0), (0), (0), (0); - sin(pitch)*cos(yaw)^2 - sin(pitch)*sin(yaw)^2 + sin(pitch), (cos(yaw)*sin(pitch)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/cos(pitch) - cos(pitch)*sin(roll) - (sin(pitch)*sin(yaw)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/cos(pitch), (sin(pitch)*sin(yaw)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)))/cos(pitch) - (cos(yaw)*sin(pitch)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))/cos(pitch) - cos(pitch)*cos(roll), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), -cos(pitch)*cos(yaw), (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)), - sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch), (0), (0), (0), (0), (0), (0); (0), (0), (0), -cos(pitch)*sin(yaw), - cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw), (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)), (0), (0), (0), (0), (0), (0); (0), (0), (0), sin(pitch), -cos(pitch)*sin(roll), -cos(pitch)*cos(roll), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (1), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (1), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (1), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (1)];

V  = U; %Input matrix

Qd = eye(12,12) * dt*0.1; %To process the noise covariance matrix

covarEst = F* covarPrev * (F)' + V* Qd * V'; %To estimate the covariance matrix for the next time step

end

