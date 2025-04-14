clear, clc, close all;
addpath("C:\Users\admin\OneDrive - North Carolina State University\Dr. Nitin Sharma\visual servo control\mex-wrapper")
addpath("C:\Users\admin\OneDrive - North Carolina State University\Dr. Nitin Sharma\visual servo control")
Simulink.importExternalCTypes('C:\Users\admin\OneDrive - North Carolina State University\Dr. Nitin Sharma\matlab_simplified_api_2.2.1\matlab_simplified_api_2.2.1\mex-wrapper\include\kortex_wrapper_data.h');
gen3 = loadrobot("kinovaGen3");
gen3.DataFormat = 'column';

%% parameters  - use camera_me function or explicitParamerts
%[O,C,C_rotationMatrix,f,d,ang,Config,EEPose_Test_1]=camera_me(gen3);

%explicitParamerts
0=[0 0 0];
C=[-.3591; -.2983; .03403];
f=[276.2;5394.6];
ang=[-1.794, .1573, 1.248];
C_rotationMatrix=[.2191, -.9629, -.1575;.2763,-.216, .9265;-.9358,.617,.3134];

jointPositionsDeg_row = {jointConfiguration.data.jointAngles};
testPose_Deg= jointPositionsDeg_row(1,end)';
testPose_Deg=cell2mat(testPose_Deg);
Config=testPose_Deg

d= C-O; 
%end of explicitParamerts

initial_Config=Config*pi/180;
init_pose = [C ; ang']
% Config=Config*(180/pi)
Config_show=Config*pi/180
show(gen3,Config_show)
% pose=init_pose
frequency = 200;       % sampling frequency [Hz] 
dT = 1/frequency;      % sampling time [s] adjusting the camera position every 5ms


% % points' position (x,y,z) in world frame
p1 = [0.5; -.35; .05]; %%measure the strting coodinates of the object-edit-usedefined
p2 = [0.7; -.35; .05]; %%measure the strting coodinates of the object-edit-userdefined
p=[p1;p2];

% reference 
ref1 = [0; -0.0];%%refeence points in image coordinates
ref2 = [0;  -0.0];%%reference points in image coordinates
ref = [ref1; ref2];

% control
Kp = 10*eye(4); %proportional gain
Ki = 9*eye(4); %differential gain
Kd = 5*eye(4); %differential gain


%connect to robot
j=load("jntconfig.mat")

