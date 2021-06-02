%% Delta Robot
% Annine
%%
clc
clear
%% values used in Commissioning
% set length

disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
    end
sim.simxAddStatusbarMessage(clientID,'Iniciada!',sim.simx_opmode_oneshot);
disp('iniciada');
    
    
[res,joint1]=sim.simxGetObjectHandle(clientID,'joint1',sim.simx_opmode_blocking)
[res,joint2]=sim.simxGetObjectHandle(clientID,'joint2',sim.simx_opmode_blocking)
[res,joint3]=sim.simxGetObjectHandle(clientID,'joint3',sim.simx_opmode_blocking)


Joint1=[joint1]
Joint2=[joint2]
Joint3=[joint3]

dof=3;
q0=[0.4968,0.4968,0.4968];
L=220%170; %  upper arm
l=334.92%300; % lower arm

f=311.17 %200*sqrt(3); % fixed baseSB
e=95.26%50*sqrt(3); % end-effectorSP

Wb = ((sqrt(3)) / 6) * f

Ub = ((sqrt(3)) / 3) * f

Wp = ((sqrt(3)) / 6) * e


Up = ((sqrt(3)) / 3) * e

a = (Wb - Up)

b = ((e * 0.5) - (((sqrt(3)) * 0.5) * Wb))

c = (Wp - (0.5 * Wb))


%a=(f/2-e)/sqrt(3);
%b=e/2-f/4;
%c=(e-f/2)/(2*sqrt(3));

len=[L,l,f,e,a,b,c];
%% values used in functions
% rotation around Z axis
R=[cos(-2*pi/3) -sin(-2*pi/3) 0;sin(-2*pi/3) cos(-2*pi/3) 0;0 0 1];
% three joints on fixed base
J1=[0 -f/(2*sqrt(3)) 0];
J2=J1*R;
J3=J2*R;
% calc and plot robot e plate
E1=[0 -e/sqrt(3) 0];
E2=E1*R;
E3=E2*R;
% other easy-cal values
wb=f/(2*sqrt(3));

con=[J1; J2; J3; E1; E2; E3];
%% inverse Inverse Position Kinematics (IPK) Solution

n=20; % divide path into n parts
dy=200; % distance
dz=1
dx=1
ax=0.0001;ay=1;az=0.0001;
pose=pathCalRec(n,dx,dy,dz,ax,ay,az);% position of end-effector
angle=zeros(n+1,3);

for i=1:n+1
    angle(i,:)=Inverse(pose(i,:),len);
end
%%
r=200; % radius of parth (path is a circle)
%x0 = pose(n+1,1); y0=pose(n+1,2); z0=pose(n+1,3)
x0 = 0; y0=0; z0=-350
s=1
pose2 = pathCalCIL(n,r,x0,y0,z0,s);% position of end-effector
angle2=zeros(n+1,3);

for i=1:n+1
    angle2(i,:)=Inverse(pose2(i,:),len);
end

%%
ax=-50;ay=50;az=0.0001;b=-200;a=0;
pose3 = pathCalPen(n,ax,ay,az,b,a)
angle3=zeros(n+1,3);


for i=1:n+1
    angle3(i,:)=Inverse(pose3(i,:),len);
end


%% 4
 % divide path into n parts
r2=150; % radius of parth (path is a circle)
x0 = -50; y0=0; z0=-350
s=-1
pose4=pathCalCIL(n,r2,x0,y0,z0,s);% position of end-effector
angle4=zeros(n+1,3);
for i=1:n+1
    angle4(i,:)=Inverse(pose4(i,:),len);
end

%% 5
ax=-50;ay=-50;az=0.0001;b=150;a=-50
pose5 = pathCalPen(n,ax,ay,az,b,a)
angle5=zeros(n+1,3);
for i=1:n+1
    angle5(i,:)=Inverse(pose5(i,:),len);
end


%% 6
r=100; % radius of parth (path is a circle)
%x0 = pose(n+1,1); y0=pose(n+1,2); z0=pose(n+1,3)
x0 = -100; y0=0; z0=-350
s=1
pose6 = pathCalCIL(n,r,x0,y0,z0,s);% position of end-effector
angle6=zeros(n+1,3);

for i=1:n+1
    angle6(i,:)=Inverse(pose6(i,:),len);
end

%% 7
ax=200;ay=200;az=0.0001;b=-100;a=-100
pose7 = pathCalPen(n,ax,ay,az,b,a)
angle7=zeros(n+1,3);
for i=1:n+1
    angle7(i,:)=Inverse(pose7(i,:),len);
end
%% 8
r=100; % radius of parth (path is a circle)
%x0 = pose(n+1,1); y0=pose(n+1,2); z0=pose(n+1,3)
x0 = +100; y0=0; z0=-350
s=1
pose8 = pathCalCIL(n,r,x0,y0,z0,s);% position of end-effector
angle8=zeros(n+1,3);

for i=1:n+1
    angle8(i,:)=Inverse(pose8(i,:),len);
end


%% 9
ax=-50;ay=-50;az=0.0001;b=-100;a=100;
pose9 = pathCalPen(n,ax,ay,az,b,a)
angle9=zeros(n+1,3);


for i=1:n+1
    angle9(i,:)=Inverse(pose9(i,:),len);
end


%% 10
 % divide path into n parts
r2=150; % radius of parth (path is a circle)
x0 = 50; y0=0; z0=-350
s=-1
pose10=pathCalCIL(n,r2,x0,y0,z0,s);% position of end-effector
angle10=zeros(n+1,3);
for i=1:n+1
    angle10(i,:)=Inverse(pose10(i,:),len);
end

%% 11
ax=-50;ay=-150;az=0.0001;b=150;a=50;
pose11 = pathCalPen(n,ax,ay,az,b,a)
angle11=zeros(n+1,3);


for i=1:n+1
    angle11(i,:)=Inverse(pose11(i,:),len);
end
%%

posef = [pose;pose2;pose3;pose4;pose5;pose6;pose7;pose8;pose9;pose10;pose11]
anglef = [angle;angle2;angle3;angle4;angle5;angle6;angle7;angle8;angle9;angle10;angle11]
%%
crr = 0.478220; %factor de correccion 
%for joint in UR51:
    sim.simxSetJointTargetPosition(clientID, Joint1, (q0(1)-crr), sim.simx_opmode_streaming)
    pause(1/100)
%for joint in UR52:
    sim.simxSetJointTargetPosition(clientID, Joint2, (q0(2)-crr), sim.simx_opmode_streaming)
    pause(1/100)
%for joint in UR53:
    sim.simxSetJointTargetPosition(clientID, Joint3, (q0(3)-crr), sim.simx_opmode_streaming)
    pause(1/100)
pause(10)

crr1 = 0.478220;
for t2=1:length(anglef)
    %for joint in UR51:
        sim.simxSetJointTargetPosition(clientID, Joint1, (anglef(t2,1)-crr1), sim.simx_opmode_streaming)
        pause(1/100)
    %for joint in UR52:
        sim.simxSetJointTargetPosition(clientID, Joint2, (anglef(t2,2)-crr1), sim.simx_opmode_streaming)
        pause(1/100)
    %for joint in UR53:
        sim.simxSetJointTargetPosition(clientID, Joint3, (anglef(t2,3)-crr1), sim.simx_opmode_streaming)
        pause(1/100)

end
%%
anim(len,posef,anglef,con,R,wb)
function []=anim(len,posef,anglef,con,R,wb)
n=length(posef(:,1));

for i=1:n
    clf;
    axis([-500 300 -500 300 -450 100])
    pause on % to enable pause function
    figureDrawing(len,posef(i,:),anglef(i,:),con,R,wb);
     for j=1:i
         plot3(posef(j,1),posef(j,2),posef(j,3),'.r')
     end
    pause(0.01); 
end
end


