function [O,C,C_rotationMatrix,f_meters,d,ang,Config,EEPose_Test_1]=camera_me(gen3)

% world origin frame
O = [0; 0; 0];        

jointConfiguration = load("kinova_pose.mat");
jointPositionsDeg_row = {jointConfiguration.data.jointAngles};
testPose_Deg= jointPositionsDeg_row(1,31)';
testPose_Deg=cell2mat(testPose_Deg);
% testPose_Radmat=deg2rad(cell2mat(testPose_Deg));
Config=testPose_Deg;
% EEPose_Test={jointConfiguration.data.endEffectorPose};
% EEPose_Test_1= EEPose_Test(1,16)';
% EEPose_Test_1=cell2mat(EEPose_Test);
EEPose_Test_1=[0 0 0]



% get the current position of camera in world coordinate frame
[C,C_rotationMatrix] = GetCurrentCameraCoordinates(gen3,Config)  
versor_origin = 0.4; %scaling factors
versor_camera = 0.2;%scaling factors
origin_axis = {'O';'X';'Y';'Z'};
camera_axis = {'oc','zc','xc','yc'};


%find image plane width and height
[~,~,f_pix,imgSize_pix] = cameraClliberation(); % Focal length in pixels

d = C - O        % distance between camera and world frame


%orientation of the camera in the world frame - Euler angles (yaw, pitch, roll) 
C_eulerAngles = rotm2eul(C_rotationMatrix, 'ZYX'); % 'ZYX' is the standard Euler angle sequence
yaw = C_eulerAngles(1);
pitch = C_eulerAngles(2);
roll = C_eulerAngles(3);
ang = [yaw pitch roll]

%calculating the field of view and focal lengths in in x and y direction, from diagonal field of
%view
FOV_d_degree=65;
w_pix=imgSize_pix(1);
h_pix=imgSize_pix(2);
f_d_pix=sqrt(w_pix^2 + h_pix^2)/(2*(tan(FOV_d_degree/2))); %diagonal focal length
FOV_x_degree=FOV_d_degree*(w_pix/(sqrt(w_pix^2+h_pix^2)));
FOV_y_degree=FOV_d_degree*(h_pix/(sqrt(w_pix^2+h_pix^2)));
f_x_pix=sqrt(w_pix^2 + h_pix^2)/(2*(tan(FOV_x_degree/2)));
f_y_pix=sqrt(w_pix^2 + h_pix^2)/(2*(tan(FOV_y_degree/2)));
FOV_x_rad=FOV_x_degree*(pi/180);
FOV_y_rad=FOV_y_degree*(pi/180);
FOV_d_rad=FOV_d_degree*(pi/180);
%calculating sensor width and height
S_w_meters=2*f_x_pix*tan(FOV_x_rad*(pi/180));
S_h_meters=2*f_y_pix*tan(FOV_y_rad*(pi/180));
%calculating the focal length in meters
f_x_meters=f_x_pix*S_w_meters/w_pix;
f_y_meters=f_y_pix*S_h_meters/h_pix;
f_meters=[f_x_meters;f_y_meters];
%calculating img size in meters
plane_y=S_h_meters;
plane_z=20;
plane_x=S_w_meters;


end

