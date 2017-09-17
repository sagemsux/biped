clc
clear
close all

u1 = -5;
u2 = 3;

u = [u1;u2];
r = 1;
m = 1;
Mh = 15;
Mt = 10;
L = 0.5;


%% kinematics test
close all
test_angles = [deg2rad(-30),deg2rad(30),deg2rad(0)];
[A,B,C,D] = kinematics(test_angles,r,L,[0 0]);
plotbot(A,B,C,D);

%% dynamics test
close all
clc
time = 1;
ic = [0 0 0 0 0 0];
options = odeset('RelTol',1e-4,'AbsTol',1e-8); % solver options

[T,Y] = ode45(@(t,y) dynamics(t,y,m,r,L,Mh,Mt,u),[0 time],ic,options);

figure
axis([-3 3 -3 3]);
walk(T,Y,r,L,[0 0]);

%% motion planning
tf = 1;
n = 100;
clc
close all
th1_ic = [0 0.5];
th1_fc = [deg2rad(15) 1];
th2_ic = [0 0.5];
th2_fc = [deg2rad(-15) 2];
th3_ic = [0 0.5];
th3_fc = [0 1];

coeff1 = motionplanning(th1_ic,th1_fc,0,1);
coeff2 = motionplanning(th2_ic,th2_fc,0,1);
coeff3 = motionplanning(th3_ic,th3_fc,0,1);

t = linspace(0,tf,n);

[th1,th1d,th1dd] = motioneval(coeff1,t);
[th2,th2d,th2dd] = motioneval(coeff2,t);
[th3,th3d,th3dd] = motioneval(coeff3,t);

%walk(t,[th1' th2' th3'],r,L,[0 0])

u = zeros(length(t),2);
for i = 1:size(t,2)
    [u(i,1),u(i,2)] = inversedynamics(L,m,Mh,Mt,r,th1(i),th2(i),th3(i),th1d(i),th2d(i),th3d(i),th1dd(i),th2dd(i),th3dd(i));
end
[A,B1,C,D] = kinematics([th1(end) th2(end) th3(end)],r,L,[0 0]);
% Plot torques
%plot(t,u(:,1),'k',t,u(:,2),'b')

%% Impact Model

%Once the robot touches the ground with impact, the joint variables change
[th1d(end),th2d(end),th3d(end)] = impact(th1(end),th2(end),th3(end),th1d(end),th2d(end),th3d(end));


%% Dynamic Model Transfer
clc
close all
tf = 1;
th1_ic2 = [th2(end) th2d(end)];
th1_fc2 = [deg2rad(15) 1];
th2_ic2 = [th1(end) th1d(end)];
th2_fc2 = [deg2rad(-15) 2];
th3_ic2 = [th3(end) th3d(end)];
th3_fc2 = [0 0.5];

coeff1 = motionplanning(th1_ic2,th1_fc2,0,1);
coeff2 = motionplanning(th2_ic2,th2_fc2,0,1);
coeff3 = motionplanning(th3_ic2,th3_fc2,0,1);

t = linspace(0,1,n);

[th1_2,th1d_2,th1dd_2] = motioneval(coeff1,t);
[th2_2,th2d_2,th2dd_2] = motioneval(coeff2,t);
[th3_2,th3d_2,th3dd_2] = motioneval(coeff3,t);

%walk(t,[th1_2' th2_2' th3_2'],r,L,[0 0])
u2 = zeros(length(t),2);
for i = 1:size(t,2)
    [u2(i,2),u2(i,1)] = inversedynamics(L,m,Mh,Mt,r,th1_2(i),th2_2(i),th3_2(i),th1d_2(i),th2d_2(i),th3d_2(i),th1dd_2(i),th2dd_2(i),th3dd_2(i));
end
[A,B2,C,D] = kinematics([th1_2(end) th2_2(end) th3_2(end)],r,L,B1);


%% Third Step
[th1d_2(end),th2d_2(end),th3d_2(end)] = impact(th1_2(end),th2_2(end),th3_2(end),th1d_2(end),th2d_2(end),th3d_2(end));

clc
close all
tf = 1;
th1_ic3 = [th2_2(end) th2d(end)];
th1_fc3 = [deg2rad(15) 1];
th2_ic3 = [th1_2(end) th1d(end)];
th2_fc3 = [deg2rad(-15) 0.5];
th3_ic3 = [th3_2(end) th3d(end)];
th3_fc3 = [deg2rad(0) 0.5];

coeff1 = motionplanning(th1_ic3,th1_fc3,0,1);
coeff2 = motionplanning(th2_ic3,th2_fc3,0,1);
coeff3 = motionplanning(th3_ic3,th3_fc3,0,1);

t = linspace(0,1,n);

[th1_3,th1d_3,th1dd_3] = motioneval(coeff1,t);
[th2_3,th2d_3,th2dd_3] = motioneval(coeff2,t);
[th3_3,th3d_3,th3dd_3] = motioneval(coeff3,t);

%walk(t,[th1_2' th2_2' th3_2'],r,L,[0 0])
u3 = zeros(length(t),2);
for i = 1:size(t,2)
    [u3(i,1),u3(i,2)] = inversedynamics(L,m,Mh,Mt,r,th1_3(i),th2_3(i),th3_3(i),th1d_3(i),th2d_3(i),th3d_3(i),th1dd_3(i),th2dd_3(i),th3dd_3(i));
end
[A,B2,C,D] = kinematics([th1_3(end) th2_3(end) th3_3(end)],r,L,B1);
%% Continuous Motion
close all
walk(t,[th1' th2' th3'],r,L,[0 0])
walk(t,[th1_2' th2_2' th3_2'],r,L,B1)
walk(t,[th1_3' th2_3' th3_3'],r,L,B2)


t2 = linspace(0,3,n*3);
u_1 = [u(:,1);u2(:,1);u3(:,1)]; u_2 = [u(:,2);u2(:,2);u3(:,2)];

close all
plot(t2,u_1,'k',t2,u_2,'b')

%%






