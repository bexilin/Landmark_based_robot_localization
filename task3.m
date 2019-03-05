close all
clear
clc
load('task3.mat')
% run EKF, UKF and PF given a known initial state
disp('press enter to perform the case of EKF')
pause
run(data,0,'ekf')
disp('press enter to perform the case of UKF')
pause
close all
run(data,0,'ukf')
disp('press enter to perform the case of PF')
pause
close all
run(data,0,'pf')

% run EKF, UKF and PF when initial state is unknown, hence dealing with a
% global localization problem
disp('press enter to perform the global localization case of EKF')
pause
close all
run(data,0,'ekf',1)
disp('press enter to perform the global localization case of UKF')
pause
close all
run(data,0,'ukf',1)
disp('press enter to perform the global localization case of PF')
pause
close all
run(data,0,'pf',1)

% run EKF, UKF and PF to deal with a kinapped robot problem, the robot's
% information between time step 51 and time step 199 is deleted, thus
% simulating the robot being kidnapped from time step 50 to time step 200
disp('press enter to perform the kinapped robot case of EKF')
pause
close all
run(500,0,'ekf',0,1)
disp('press enter to perform the kinapped robot case of UKF')
pause
close all
run(500,0,'ukf',0,1)
disp('press enter to perform the kinapped robot case of PF')
pause
close all
run(500,0,'pf',0,1)
disp('press enter to end')
pause
close all