function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively

% H: Identity matrix
H = [zeros(3,3),zeros(3,3),eye(3,3),zeros(3,3),zeros(3,3)];

%To predict the measurement by using predicted state estimate
z = H * uEst + zeros(3,1);

% To state the measurement Jacobian matrix (denoted as C) equal to Identity matrix(H)
C = H ;

% Measurement noise covariance R tune these values based on your system's noise characteristics 
R = eye(3,3)*0.05;

% The Kalman gain 
K = covarEst * C' /(C * covarEst* C' + R );

% Update state estimate
uCurr = uEst + K * (z_t - z );

% Update covariance estimate 
covar_curr = covarEst- K* C * covarEst;

end