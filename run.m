function varargout = run(stepsOrData, pauseLen, filter, global_locolization, kidnapped, makeVideo)
% RUN  PS2 Feature-Based Localization Simulator
%   RUN(ARG)
%   RUN(ARG, PAUSLEN, MAKEVIDEO)
%      ARG - is either the number of time steps, (e.g. 100 is a complete
%            circuit) or a data array from a previous run.
%      PAUSELEN - set to `inf`, to manually pause, o/w # of seconds to wait
%                 (e.g., 0.3 is the default)
%      MAKEVIDEO - boolean specifying whether to record a video or not
%
%   DATA = RUN(ARG,PAUSELEN)
%      DATA - is an optional output and contains the data array generated
%             and/or used during the simulation.

%   (c) 2009-2015
%   Ryan M. Eustice
%   University of Michigan
%   eustice@umich.edu

if ~exist('pauseLen','var') || isempty(pauseLen)
    pauseLen = 0.3; % seconds
end
if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end

%--------------------------------------------------------------
% Graphics
%--------------------------------------------------------------

NOISEFREE_PATH_COL = 'green';
ACTUAL_PATH_COL = 'blue';

NOISEFREE_BEARING_COLOR = 'cyan';
OBSERVED_BEARING_COLOR = 'red';

GLOBAL_FIGURE = 1;

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(5, 1/pauseLen));
        open(vo);
    end
end

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------

initialStateMean = [180 50 0]';

% Motion noise (in odometry space, see Table 5.5, p.134 in book).
alphas = [0.05 0.001 0.05 0.01].^2; % variance of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
beta = deg2rad(20);

% Step size between filter updates, can be less than 1.
deltaT=0.1;

persistent data numSteps;
if isempty(stepsOrData) % use dataset from last time
    if isempty(data)
        numSteps = 100;
        data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
    end
elseif isscalar(stepsOrData)
    % Generate a dataset of motion and sensor info consistent with
    % noise models.
    numSteps = stepsOrData;
    data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
else
    % use a user supplied dataset from a previous run
    data = stepsOrData;
    numSteps = size(data, 1);
    global FIELDINFO;
    FIELDINFO = getfieldinfo;
end


% TODO: provide proper initialization for your filters here
% You can set the initial mean and variance of the EKF to the true mean and
% some uncertainty.
mu=zeros(3,numSteps+1);
Sigma=zeros(3,3,numSteps+1);

% if not deal with global locolization problem, set the known initial state
% value, otherwise sample from uniform distribution
if ~exist('global_locolization','var') || isempty(global_locolization) || global_locolization == 0
    % mu and Sigma initialization for EKF and UKF
    mu(:,1)=initialStateMean;
    Sigma(:,:,1)=[100 0 0;0 100 0;0 0 0.1];
    % samples and weight initialization for PF
    n=500;
    samples=mvnrnd(mu(:,1),Sigma(:,:,1),n)';
    weight=ones(1,n)/n;
else
    % mu and Sigma initialization for EKF and UKF
    mu(:,1)=[500*rand;300*rand;-pi+2*pi*rand];
    Sigma(:,:,1)=[10000 0 0;0 10000 0;0 0 10];
    % samples and weight initialization for PF
    n=5000;
    samples=[500*rand(1,n);300*rand(1,n);-pi+2*pi*rand(1,n)];
    weight=ones(1,n)/n;
end

% R is the sensor noise, its value can be changed here to study the influence
R=beta^2;

% if deal with the kinapped robot problem, let the robot jump at time
% step 50 directly to time step 200
if ~exist('kidnapped','var') || isempty(kidnapped) || kidnapped == 0
    0;
else
    data(51:199,:)=[];
    numSteps = size(data, 1);
end

% Call ekfUpdate, ukfUpdate and pfUpdate in every iteration of this loop.
% You might consider putting in a switch yard so you can select which
% algorithm does the update
results = [];
for t = 1:numSteps

    %=================================================
    % data available to your filter at this time step
    %=================================================
    motionCommand = data(t,3:5)'; % [drot1, dtrans, drot2]' noisefree control command
    observation = data(t,1:2)';   % [bearing, landmark_id]' noisy observation

    %=================================================
    % data *not* available to your filter, i.e., known
    % only by the simulator, useful for making error plots
    %=================================================
    % actual position (i.e., ground truth)
    x = data(t,8);
    y = data(t,9);
    theta = data(t,10);

    % noisefree observation
    noisefreeBearing = data(t, 6);

    %=================================================
    % graphics
    %=================================================
    figure(GLOBAL_FIGURE); clf; hold on; plotfield(observation(2));

    % draw actual path and path that would result if there was no noise in
    % executing the motion command
    plot([initialStateMean(1) data(1,8)], [initialStateMean(2) data(1,9)], 'Color', ACTUAL_PATH_COL);
    plot([initialStateMean(1) data(1,11)], [initialStateMean(2) data(1,12)], 'Color', NOISEFREE_PATH_COL);

    % draw actual path (i.e., ground truth)
    plot(data(1:t,8), data(1:t,9), 'Color', ACTUAL_PATH_COL);
    plotrobot( x, y, theta, 'black', 1, 'cyan');

    % draw noise free motion command path
    plot(data(1:t,11), data(1:t,12), 'Color', NOISEFREE_PATH_COL);
    plot(data(t,11), data(t,12), '*', 'Color', NOISEFREE_PATH_COL);

    % indicate observed angle relative to actual position
    plot([x x+cos(theta+observation(1))*100], [y y+sin(theta+observation(1))*100], 'Color', OBSERVED_BEARING_COLOR);

    % indicate ideal noise-free angle relative to actual position
    plot([x x+cos(theta+noisefreeBearing)*100], [y y+sin(theta+noisefreeBearing)*100], 'Color', NOISEFREE_BEARING_COLOR);

    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
    switch filter
        case 'ekf'
            % Q is the motion noise, its value can be changed here to study the influence
            Q=zeros(3,3);
            Q(1,1)=alphas(1)*motionCommand(1)^2+alphas(2)*motionCommand(2)^2;
            Q(2,2)=alphas(3)*motionCommand(2)^2+alphas(4)*(motionCommand(1)^2+motionCommand(3)^2);
            Q(3,3)=alphas(1)*motionCommand(3)^2+alphas(2)*motionCommand(2)^2;
            [mu(:,t+1),Sigma(:,:,t+1)]=ekfUpdate(mu(:,t),Sigma(:,:,t),motionCommand,observation,Q,R);
            g=plot(mu(1,1:t+1),mu(2,1:t+1),'r');
            h=draw_ellipse(mu(1:2,t+1), Sigma(1:2,1:2,t+1),9,'k');
            legend([g,h],{'EKF mean','EKF 3-sigma ellipsoid'})
        case 'ukf'
            % Q is the motion noise, its value can be changed here to study the influence
            Q=zeros(3,3);
            Q(1,1)=alphas(1)*motionCommand(1)^2+alphas(2)*motionCommand(2)^2;
            Q(2,2)=alphas(3)*motionCommand(2)^2+alphas(4)*(motionCommand(1)^2+motionCommand(3)^2);
            Q(3,3)=alphas(1)*motionCommand(3)^2+alphas(2)*motionCommand(2)^2;
            [mu(:,t+1),Sigma(:,:,t+1)]=ukfUpdate(mu(:,t),Sigma(:,:,t),motionCommand,observation,Q,R);
            g=plot(mu(1,1:t+1),mu(2,1:t+1),'r');
            h=draw_ellipse(mu(1:2,t+1), Sigma(1:2,1:2,t+1),9,'k');
            legend([g,h],{'UKF mean','UKF 3-sigma ellipsoid'})
        case 'pf'
            [samples, weight] = pfUpdate(samples, weight, n, alphas, motionCommand, observation, R);
            g=plot(samples(1,:),samples(2,:),'r.');
            [mu(:,t+1), Sigma(:,:,t+1)] = meanAndVariance(samples, length(samples));
            h=draw_ellipse(mu(1:2,t+1),Sigma(1:2,1:2,t+1),9,'k');
            legend([g,h],{'PF particles','PF 3-sigma ellipsoid'})
    end

    drawnow;
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end

    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
end

%=================================================
%TODO: plot and evaluate filter results here
%=================================================
% plots of pose error vs time
figure(2)
plot(1:numSteps,mu(1,2:numSteps+1)-data(:,8)','b',1:numSteps,3*sqrt(reshape(...
Sigma(1,1,2:numSteps+1),[1,numSteps])),'r',1:numSteps,-3*sqrt(reshape(...
Sigma(1,1,2:numSteps+1),[1,numSteps])),'r');
xlabel('t')
ylabel('x hat-x')
legend('x hat-x(error)','3-sigma bounds')
figure(3)
plot(1:numSteps,mu(2,2:numSteps+1)-data(:,9)','b',1:numSteps,3*sqrt(reshape(...
Sigma(2,2,2:numSteps+1),[1,numSteps])),'r',1:numSteps,-3*sqrt(reshape(...
Sigma(2,2,2:numSteps+1),[1,numSteps])),'r');
xlabel('t')
ylabel('y hat-y')
legend('y hat-y(error)','3-sigma bounds')
figure(4)
theta_d=zeros(1,numSteps);
for i=1:1:numSteps
    theta_d(i)=minimizedAngle(mu(3,i+1)-data(i,10)');
end
plot(1:numSteps,theta_d,'b',1:numSteps,3*sqrt(reshape(...
Sigma(3,3,2:numSteps+1),[1,numSteps])),'r',1:numSteps,-3*sqrt(reshape(...
Sigma(3,3,2:numSteps+1),[1,numSteps])),'r');
title('theta hat-theta vs. t')
xlabel('t')
ylabel('theta hat-theta')
legend('theta hat-theta(error)','3-sigma bounds')

switch filter
    case 'ekf'
        figure(2)
        title('EKF x hat-x vs. t')
        figure(3)
        title('EKF y hat-y vs. t')
        figure(4)
        title('EKF theta hat-theta vs. t')
    case 'ukf'
        figure(2)
        title('UKF x hat-x vs. t')
        figure(3)
        title('UKF y hat-y vs. t')
        figure(4)
        title('UKF theta hat-theta vs. t')
    case 'pf'
        figure(2)
        title('PF x hat-x vs. t')
        figure(3)
        title('PF y hat-y vs. t')
        figure(4)
        title('PF theta hat-theta vs. t')
end

if nargout >= 1
    varargout{1} = data;
end
if nargout >= 2
    varargout{2} = results;
end

if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end


