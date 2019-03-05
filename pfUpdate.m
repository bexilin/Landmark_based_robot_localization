function [samples_1, weight_1] = pfUpdate(samples, weight, n, alphas, u, z, R)
    
% NOTE: The header is not set in stone.  You may change it if you like.
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(z(2));
landmark_y = FIELDINFO.MARKER_Y_POS(z(2));

%stateDim=3;
%motionDim=3;
%observationDim=2;

samples_1=zeros(3,n);
weight_1=zeros(1,n);
for i=1:n
    % generate new samples with samples at last time step and new motion
    samples_1(:,i)=sampleOdometry(u, samples(:,i), alphas);
    z_hat=minimizedAngle(atan2(landmark_y-samples_1(2,i),landmark_x-samples_1(1,i))-samples_1(3,i));
    v=wrapToPi(z(1)-z_hat);
    % update new weight
    weight_1(i)=weight(i)*normpdf(v,0,sqrt(R));
end
weight_1=weight_1/sum(weight_1);
n_eff=1/sum(weight_1.^2);

% resample 
if n_eff<(n/1)
    [samples_1, weight_1] = resample(samples_1, weight_1);
end


