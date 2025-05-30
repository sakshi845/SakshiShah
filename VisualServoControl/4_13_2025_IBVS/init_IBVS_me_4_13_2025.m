clear, clc, close all;
addpath("C:\Users\admin\OneDrive - North Carolina State University\Dr. Nitin Sharma\visual servo control\mex-wrapper")
addpath("C:\Users\admin\OneDrive - North Carolina State University\Dr. Nitin Sharma\visual servo control")
Simulink.importExternalCTypes('C:\Users\admin\OneDrive - North Carolina State University\Dr. Nitin Sharma\matlab_simplified_api_2.2.1\matlab_simplified_api_2.2.1\mex-wrapper\include\kortex_wrapper_data.h');
gen3 = loadrobot("kinovaGen3");
gen3.DataFormat = 'column';

%% parameters
[O,C,C_rotationMatrix,f,d,ang,Config]=camera_me_4_13_25(gen3);
initial_Config=Config*pi/180;
init_pose = [C ; ang']
Config_show=Config*pi/180%configuration in radians
show(gen3,Config_show)

% pose=init_pose
frequency = 200;       % sampling frequency [Hz] 
dT = 1/frequency;      % sampling time [s] adjusting the camera position every 5ms


% % points' position (x,y,z) in world frame
p1 = [0.34; -.65; .035]; %%measure the strting coodinates of the object-edit-usedefined
p2 = [0.44; -.64; .035]; %%measure the strting coodinates of the object-edit-userdefined
p=[p1;p2];

% reference 
ref1 = [0; -0.0];%%refeence points in image coordinates
ref2 = [0;  -0.0];%%reference points in image coordinates
ref = [ref1; ref2];

% control
Kp = 10*eye(4); %proportional gain
Ki = 9*eye(4); %differential gain
Kd = 5*eye(4); %differential gain
