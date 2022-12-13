clear all;
clc;


robot=rigidBodyTree('DataFormat','column','MaxNumBodies',4);
L1=6.5;L2=9;L3=22.5;

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, (trvec2tform([0,0,L1])*eul2tform([pi/2,0,0])));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([0,0,L2]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link2');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([0, 0, L3]));
body.Joint = joint;
addBody(robot, body, 'link3');

showdetails(robot)
show(robot)
axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
%% OWI rigid body tree without joint constraints

% t = (0:0.2:10)'; % Time
% count = length(t);
% center = [0.3 0.1 0];
% radius = 0.15;
% theta = t*(2*pi/t(end));
% points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];
%cam = webcam("Integrated Webcam");%% Laptop Camera
%%Start image processing
cam = webcam("HD Web Camera");%% External USB Camera

I = snapshot(cam);
tagFamily = ["tag36h11","tagCircle21h7","tagCircle49h12","tagCustom48h12","tagStandard41h12"];

[id,loc,detectedFamily] = readAprilTag(I,tagFamily);

for idx = 1:length(id)
        % Display the ID and tag family
        disp("Detected Tag ID, Family: " + id(idx) + ", " ...
            + detectedFamily(idx));
 
        % Insert markers to indicate the locations
        markerRadius = 8;
        numCorners = size(loc,1);
        polyin = polyshape(loc(:,:,1));
        [x,y] = centroid(polyin);
        polyin = polyshape(loc(:,:,2));
        [x1,y1] = centroid(polyin);
        plot([x,y],[x1,y1],'m*','markersize',32)
        markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
        markerPosition1 = [x,y,x1,y1];
        I = insertShape(I,"FilledCircle",markerPosition,Color="red",Opacity=1);
end
% for idx = 1:length(id)
%     polyin = polyshape(loc(:,:,1));
%     [x,y] = centroid(polyin);
%     polyin = polyshape(loc(:,:,2));
%     [x1,y1] = centroid(polyin);
%     %polyin = polyshape(loc(:,:,3));
%     %[x2,y2] = centroid(polyin);
%     plot([x,y],[x1,y1],'m*','markersize',32)  
%      %markerPosition1 = [[x,y,1],[x1,y1,2]];
%      %I = insertShape(I,"FilledRectangle",markerPosition1,Color="red",Opacity=1);
% end
imshow(I)

%% xy in real coordinates =Rot (camera w.r.t world frame) * Xc points in camera frame+ O (camera w.r.t world frame)
Rot= [-0.707 0 0.707;0 1 0; -0.707 0 0.707];% Rot in y by 135 deg
k=0.44/41;u =abs((x-949.32)*1^(-4)/k);v =abs((y-489.30)*1^(-4)/k); %%OC Or values for our camera %%considering 1 micron = e^-4 cm for w h of pixel
xc=[u v 41];
xw=Rot*xc+[0 36 41];
%% above not complete have to calculate Rot again

points = [x y;x1 y1;x2 y2 ]; %% convert these points from camera frame to robot base frame in loop again 
%% values for points incorrect do again
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
% Solve for the configuration satisfying the desired end effector
% position
point = points(i,:);
qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
% Store the configuration
qs(i,:) = qSol;
% Start from prior solution
qInitial = qSol;
end

%% not required to run
% figure
% show(robot,qs(1,:)');
% view(2)
% ax = gca;
% ax.Projection = 'orthographic';
% hold on
% plot(points(:,1),points(:,2),'k')
% axis([-0.1 0.7 -0.3 0.5])
% 
% framesPerSecond = 15;
% r = rateControl(framesPerSecond);
% for i = 1:count
% show(robot,qs(i,:)','PreservePlot',false);
% drawnow
% waitfor(r);
% end

%% convert angles to voltage and use potentiometer 


%%  Give Voltage for base, shoulder and elbow using poly fit, using potentiometer values
a = arduino('COM4','mega2560','Libraries','Adafruit/MotorShieldV2');
shield = addon(a,'Adafruit/MotorShieldV2');

configurePin(a, 'A8', 'AnalogInput');  
configurePin(a, 'A9', 'AnalogInput'); 
configurePin(a, 'A10', 'AnalogInput'); 
writeDigitalPin(a,'D52',1);
writeDigitalPin(a,'D53',0);
writeDigitalPin(a,'D50',1);
writeDigitalPin(a,'D51',0);
writeDigitalPin(a,'D48',1);
writeDigitalPin(a,'D49',0);

dcm1 = dcmotor(shield,4); %% changed from 1 to 4
dcm2 = dcmotor(shield,3);%% changed from 2 to 3
dcm3 = dcmotor(shield,2);%% changed from 3 to 2
pin1=readVoltage(a,'A8');
pin2=readVoltage(a,'A10');
pin3=readVoltage(a,'A9');


%% After point is reached open /close jaw
dcm1 = dcmotor(shield,1);
dcm1.Speed = -0.7;% jaw close
start(dcm1)
pause(3)
stop(dcm1)

clear cam;clear all
clc;