clear
clc
addpath('/usr/local/share/openrave-0.9/matlab/')
addpath('/usr/local/share/openrave-0.9/matlab/examples')
addpath('openrave_mex')
%% Script for simulating a camera and displaying its results in real-time
global orConnectionParams
orConnectionParams.ip='127.0.0.1';
% orConnectionParams.port=11000;
orConnectionParams.port=4765;


more off; % turn off output paging
addopenravepaths()

if( ~exist('render','var') )
    render = [0];
end
sensorindex = 0;
%write full path


orEnvLoadScene('/home/aseem/aseem_camera_openrave/openrave_data/freecam_room1.xml',1); % reset the scene



bodies = orEnvGetBodies();
robots = orEnvGetRobots();
robot=robots{1};
robotid = robot.id;

% to turn on the rendering, send a command to the sensor
for i = 0:1
    orRobotSensorConfigure(robot.id, i, 'PowerOn');
    %orRobotSensorConfigure(robot.id, render(i), 'RenderDataOn');
    
end




%start scene1
Tinit = orBodyGetTransform(robots{1}.id);
Tinit =reshape(Tinit, [3 4]);
Tinit1=Tinit; 
Tfinal = Tinit;
% Tfinal(1,4)=Tinit(1,4)-10;
% Tfinal(2,4)=Tinit(2,4)-5;
% Tfinal(3,4)=Tinit(3,4)+15;
Tfinal(1,4)=Tinit(1,4);
Tfinal(2,4)=Tinit(2,4);
Tfinal(3,4)=Tinit(3,4);

% Rotation
%Tfinal(1:3,1:3)=eul2rotm([0.125 -0.125 -0.02])'*Tinit(1:3,1:3); %[0.5403   -0.8415         0;0.8415    0.5403         0 ;0 0 1];
Tfinal(1:3,1:3)=eul2rotm([0.5236 0 0])'*Tinit(1:3,1:3); %[0.5403   -0.8415         0;0.8415    0.5403         0 ;0 0 1];
%Tfinal(1:3,1:3)=Tinit(1:3,1:3);
orBodySetTransform(robots{1}.id,Tinit1);
pause(2)
data = orRobotSensorGetData(robotid, sensorindex);
pause(0.2)
img1=uint8(data.I*255);    %TINIT

orBodySetTransform(robots{1}.id,Tfinal);
pause(2);
data = orRobotSensorGetData(robotid, sensorindex);
pause(0.2)
imgd=uint8(data.I*255); %TFINAL
subplot(2,2,2),imshow(imgd);
imgwritepathinit=sprintf('results/s1_seq2/currentimg/s1_seq2_start.png');
%imwrite(img1,imgwritepathinit);
imgwritepathfinal=sprintf('results/s1_seq2/currentimg/s1_seq2_final.png');
%imwrite(imgd,imgwritepathfinal);
%end scene 1
net=initnet();
iter=1;
while iter<401,
    
    orBodySetTransform(robots{1}.id,Tinit);
    pause(2);
    data = orRobotSensorGetData(robotid, sensorindex);
    pause(0.2);
    img=uint8(data.I*255);    %TINIT
    
    imgdiff_arr(iter) = norm(single(rgb2gray(img)-rgb2gray(imgd))); %absolute value of image difference
    
    Tcurr_arr{iter}=orBodyGetTransform(robots{1}.id);
    Tcurr_arr{iter}=reshape(Tcurr_arr{iter}, [3 4]);
    
   % nrmimgdiff = norm(imgdiff);
    %if nrmimgdiff<thresh
    %    break
       
    
    subplot(2,2,1),imshow(img);
    subplot(2,2,2),imshow(imgd);
    %subplot(2,2,4),imshow(img1);
    pose = deploy_matlab(net,img, imgd); %this will give final wrt init but we want init wrt final
    

    q = myquatinv(pose(1:4)); %inverse to get opposite rotation
    R = quat2rotm(q);
    %q = pose(1:4); %inverse to get opposite rotation
    %Rinv = quat2rotm(q);
    %inverse to get opposite rotation
    %R = Rinv';
    utheta = quat2axang(q);
    t = -1 * pose(5:7); %negetive because our cnn gave opposite transformation
    
    %translation error
    %terr_arr(iter)=norm(t);
    
    
    % translation in millimeters, theta in radians
    
    %for initial room scene use
    lambda1 = 1;
    lambda2 = 0.06;
    
    %lambda1 = 0.25;
    %lambda2 = 0.5;
    
    tvel = -1 * lambda2 *  R' * t' ; %t is a row vector so took transpose, result is a row vector (3X3 3X1 , 3X1)
    tvel = tvel(:) ; % converting to column vector
    rvel = [utheta(1:3), -1*lambda1*utheta(4)]  ;
    %rvel = rvel' ; % converting to column vector
    tvel_arr(iter)=norm(tvel);
    rvel_arr(iter)=norm(lambda1*utheta(4));
    %rvel_arr(iter)=norm(rvel);
    subplot(2,2,3),plot(tvel_arr);
    subplot(2,2,4),plot(imgdiff_arr);
    % update step
    
    % check euler rotation 
    
    
    
    %tvel'  ;
    %tnew = t - tvel' ; %IT IS WORKING
    %uthetanew = [utheta(1:3) utheta(4)*(1+lambda2)] ; %IT IS WORKING
    %tnew = t + tvel' ; %something new
    %uthetanew = [utheta(1:3) utheta(4)*(1-lambda2)] ; %something new
    Tdelta= [    axang2rotm(rvel), tvel]; %calculations of initial wrt final converted to final wrt initial
    Tinit=[Tinit;0 0 0 1]*[Tdelta;0 0 0 1];
    Tinit=Tinit(1:3,:);
    
    Tdelta_arr{iter}=Tdelta;
    %tvel_arr(1:6,iter)=[tvel' rotm2eul([axang2rotm(utheta(1:3)
    %-utheta(4)*lambda2]))];
    imgwritepath=sprintf('results/s1_seq2/currentimg/s1_seq2_img%05d.png',iter);
    imwrite(img,imgwritepath);
    %imgwritepathdiff=sprintf('results/diffimg/dimg%05d.png',iter);
    %imwrite(imgdiff,imgwritepathdiff)
    %to write initial imge,
    
    iter=iter+1;
end

%[0.5403 -0.8415 0; 0 0 1;0.845 0.5403 0] interesting results, not much
%scene is overlapping

save('results/s1_seq2/s1_seq2.mat','lambda1','lambda2','Tdelta_arr','tvel_arr','rvel_arr','imgdiff_arr','Tinit1','Tfinal','Tcurr_arr','Tinit');
