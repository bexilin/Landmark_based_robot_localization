function [mu_1, Sigma_1] = ekfUpdate(mu, Sigma, u ,z , Q, R)
 
% NOTE: The header is not set in stone.  You may change it if you like.
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(z(2));
landmark_y = FIELDINFO.MARKER_Y_POS(z(2));

%stateDim=3;
%motionDim=3;
%observationDim=2;

% --------------------------------------------
% Prediction step
% --------------------------------------------

% EKF prediction of mean and covariance
mu_hat=prediction(mu, u);
F=[1 0 -u(2)*sin(mu(3)+u(1));0 1 u(2)*cos(mu(3)+u(1));0 0 1];
Sigma_hat=F*Sigma*F'+Q;

%--------------------------------------------------------------
% Correction step
%--------------------------------------------------------------

% Compute expected observation and Jacobian
z_hat=minimizedAngle(atan2(landmark_y-mu_hat(2),landmark_x-mu_hat(1))-mu_hat(3));
H=[(landmark_y-mu_hat(2))/((landmark_x-mu_hat(1))^2+(landmark_y-mu_hat(2))^2) -(landmark_x-mu_hat(1))/((landmark_x-mu_hat(1))^2+(landmark_y-mu_hat(2))^2) -1];
% Innovation / residual covariance
%v=z(1)-z_hat;
v=minimizedAngle(z(1)-z_hat);
S=H*Sigma_hat*H'+R;
% Kalman gain
K=Sigma_hat*H'*inv(S);
% Correction
mu_1=mu_hat+K*v;
mu_1(3)=minimizedAngle(mu_1(3));
Sigma_1=(eye(3)-K*H)*Sigma_hat;
%Sigma_1=(eye(3)-K*H)*Sigma_hat*(eye(3)-K*H)'+K*R*K';

