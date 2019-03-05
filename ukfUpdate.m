function [mu_1, Sigma_1] = ukfUpdate(mu, Sigma, u ,z , Q, R)
 
% NOTE: The header is not set in stone.  You may change it if you like.
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(z(2));
landmark_y = FIELDINFO.MARKER_Y_POS(z(2));

%stateDim=3;
%motionDim=3;
%observationDim=2;

% --------------------------------------------
% Setup UKF
% --------------------------------------------

% UKF params
alpha=0.001;
k=0;
beta=30;
lamda=alpha^2*(3+k)-3;
 
% Sigma points
L_p=chol((3+lamda)*Sigma,'lower');
X=zeros(3,7);
X(:,1)=mu;
X(3,1)=minimizedAngle(X(3,1));
for i=2:4
    X(:,i)=mu+L_p(:,i-1);
    X(3,i)=minimizedAngle(X(3,i));
    X(:,i+3)=mu-L_p(:,i-1);
    X(3,i+3)=minimizedAngle(X(3,i+3));
end
% Weights
wm=zeros(1,7);
wc=zeros(1,7);
wm(1)=lamda/(3+lamda);
wc(1)=wm(1)+(1-alpha^2+beta);
for i=2:7
    wm(i)=1/(2*(3+lamda));
    wc(i)=1/(2*(3+lamda));
end

% --------------------------------------------
% Prediction step
% --------------------------------------------

% UKF prediction of mean and covariance
mu_hat=zeros(3,1);
Sigma_hat=Q;
for i=1:7
    f=prediction(X(:,i),u);
    mu_hat=mu_hat+wm(i)*f;
end
mu_hat(3)=minimizedAngle(mu_hat(3));
for i=1:7
    f=prediction(X(:,i),u);
    Sigma_hat=Sigma_hat+wc(i)*(minimizedAngle(f-mu_hat))*(minimizedAngle(f-mu_hat))';
end
L_p_hat=chol((3+lamda)*Sigma_hat,'lower');
X_hat=zeros(3,7);
X_hat(:,1)=mu_hat;
X_hat(3,1)=minimizedAngle(X_hat(3,1));
for i=2:4
    X_hat(:,i)=mu_hat+L_p_hat(:,i-1);
    X_hat(3,i)=minimizedAngle(X_hat(3,i));
    X_hat(:,i+3)=mu_hat-L_p_hat(:,i-1);
    X_hat(3,i+3)=minimizedAngle(X_hat(3,i+3));
end

%--------------------------------------------------------------
% Correction step
%--------------------------------------------------------------

% UKF correction of mean and covariance
z_hat=0;
S=R;
Sigma_xzk=zeros(3,1);
for i=1:7
    h=minimizedAngle(atan2(landmark_y-X_hat(2,i),landmark_x-X_hat(1,i))-X_hat(3,i));
    z_hat=z_hat+wm(i)*h;
end
z_hat=minimizedAngle(z_hat);
for i=1:7
    h=minimizedAngle(atan2(landmark_y-X_hat(2,i),landmark_x-X_hat(1,i))-X_hat(3,i));
    S=S+wc(i)*(h-z_hat)*(h-z_hat)';
    Sigma_xzk=Sigma_xzk+wc(i)*(X_hat(:,i)-mu_hat)*(h-z_hat)';
end
K=Sigma_xzk*inv(S);
mu_1=mu_hat+K*(minimizedAngle(z(1)-z_hat));
mu_1(3)=minimizedAngle(mu_1(3));
Sigma_1=Sigma_hat-K*S*K';
