clc
clear all
close all 
clear preva2 
clear k
global preva2
global k 
k = 0;
xd = @(t)(1*t)+1/(1+exp(-(t-5)));
yd = @(t)(-3+4/(1+exp(-(t-5)))-2/(1+exp(-(t-15))));
%xd = @(t)(16*sin(t)^3);
%yd = @(t)(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t));
Wcar = 1.5;
Lcar = 4;
maxa = 14;
f  = 1/50;
%xd = @(t)(16*sin(2*pi*f*t));
%yd = @(t)(8*sin(4*pi*f*t));
x0 = [xd(0)-2,yd(0)+2,0.3,3];
%x0 = [xd(0),yd(0),0,0];
opts = odeset('MaxStep',10^-2);%,'OutputFcn',@odeplot,'OutputSel',[1]);
Ts = 0:0.01:10;
kx = 2.5;
ky = 2.2;
ktheta = 10;
kv = 10;
xemax = abs(x0(1) - xd(0))+10;
yemax = abs(x0(2) - yd(0))+10;
xeinf = 0.5;
yeinf = 0.5;
xl = 1;
yl = 1;
r1 = @(t)((xemax-xeinf)*exp(-xl*t)+xeinf)
r2 = @(t)((yemax-yeinf)*exp(-yl*t)+yeinf)
ksi_1 = (x0(1)-xd(0))/r1(0);
ksi_2 = (x0(2)-yd(0))/r2(0);
a1 = -kx*ppcTrans(ksi_1);
a2 = -ky*ppcTrans(ksi_2);
preva2 = a2;
a = 1;
gain = 10;
vd0 = getVd(a1,a2);
thetad0 = getThetad(a1,a2,a,gain);
%thetad0 = atan(a2/a1);
thetaemax = abs(x0(3) - thetad0) + 0.1;
vemax = abs(x0(4) -vd0) + 5;
thetaeinf = 0.5;
veinf = 5;
thetal = 0.8;
vl = 0.8;
%r3 = @(t)((thetaemax-thetaeinf)*exp(-thetal*t)+thetaeinf)
r3 = @(t)(thetaemax+0*t)
r4 = @(t)((vemax-veinf)*exp(-vl*t)+veinf)
e = 1;
[T,X] = ode15s(@(t,x)trajectorysystem(t,x,xd,yd,r1,r2,r3,r4,kx,ky,ktheta,kv,a,gain,Lcar),Ts,x0,opts);
xds = zeros(size(T));
yds = zeros(size(T));
for ts = 1 : length(T)
    xds(ts) = xd(T(ts));
end
for ts = 1 : length(T)
    yds(ts) = yd(T(ts));
end
xddot = diff(xds);
yddot = diff(yds);

%{
% Plot the output trajectory 
figure
curve = animatedline('Color','r');
axis([-0.1,30,-3,20])
for t = 1:length(T) 
        addpoints(curve,X(t,1),X(t,2));
        drawnow
        hold on 
end
title("Trajectories of car")
xlabel("X position")
ylabel("Y position")
%}
%Calculation of ksi and 
ksi1 = zeros(size(T));
ksi2 = zeros(size(T));
e1 = zeros(size(T));
e2 = zeros(size(T));
for i = 1 : length(ksi1)
    e1(i) = (X(i,1) - xd(T(i)));
    e2(i) = (X(i,2) - yd(T(i))); 
    ksi1(i) = (X(i,1) - xd(T(i)))/r1(T(i));
    ksi2(i) = (X(i,2) - yd(T(i)))/r2(T(i));    
end
%x xd and errorx 
figure
subplot(2,1,1)
plot(T,X(:,1))
hold on 
plot(T,xds)
legend("Car x","Desired x")
xlabel("Time in sec")
ylabel("X position in m")
title("X position in time plot")
subplot(2,1,2)
plot(T,e1)
hold on 
plot(T,r1(T))
hold on 
plot(T,-r1(T)) 
legend("Error x","rx","-rx")
xlabel("Time in sec")
ylabel("Error x")
title("ex plot")
%y yd and errorx 
figure
subplot(2,1,1)
plot(T,X(:,2))
hold on 
plot(T,yds)
legend("Car y","Desired y")
xlabel("Time in sec")
ylabel("Y position in m")
title("Y position in time plot")
subplot(2,1,2)
plot(T,e2)
hold on 
plot(T,r2(T))
hold on 
plot(T,-r2(T))
legend("Error y","ry","-ry")
xlabel("Time in sec")
ylabel("Error y")
title("ey plot")
figure 
subplot(2,1,1)
plot(T,ksi1)
hold on
plot(T,ones(size(T)))
hold on
plot(T,-ones(size(T)))
legend("ksi_x","1","-1")
xlabel("Time in sec")
ylabel("Normalized error x")
title("ξx plot")
subplot(2,1,2)
plot(T,ksi2)
hold on
plot(T,ones(size(T)))
hold on
plot(T,-ones(size(T)))
legend("ksi_y","1","-1")
xlabel("Time in sec")
ylabel("Normalized error y")
title("ξy plot")
k = 0;
preva2 = a2;
ksi_1 = (x0(1)-xd(0))/r1(0);
ksi_2 = (x0(2)-yd(0))/r2(0);
a1 = -kx*ppcTrans(ksi_1);
a2 = -ky*ppcTrans(ksi_2);
vds = zeros(size(T));
thetads = zeros(size(T));
ksi_thetas = zeros(size(T));
ksi_vs = zeros(size(T));
e_thetas = zeros(size(T));
e_vs = zeros(size(T));
for j = 1:length(T)
    ksi_1 = (X(j,1)-xd(T(j)))/r1(T(j));
    ksi_2 = (X(j,2)-yd(T(j)))/r2(T(j));
    a1 = -kx*ppcTrans(ksi_1);
    a2 = -ky*ppcTrans(ksi_2);  
    vds(j) = getVd(a1,a2);
    thetads(j) = getThetad(a1,a2,a,gain);
    preva2 = a2;
    ksi_thetas(j) = (X(j,3)-thetads(j))/r3(T(j));
    ksi_vs(j) = (X(j,4)-vds(j))/r4(T(j));
    e_thetas(j) = (X(j,3)-thetads(j));
    e_vs(j) = (X(j,4)-vds(j));
end
%desiredthetas = zeros(size(T));
for j = 1:(length(T)-1)
    desiredthetas(j) = atan(yddot(j)/xddot(j));
end
for j = 1:(length(T)-1)
    desiredveloc(j) = sqrt((xddot(j)/(T(2)-T(1)))^2+(yddot(j)/(T(2)-T(1)))^2);
end
figure
subplot(2,1,1)
plot(T,X(:,3))
hold on
plot(T(1:end-1),desiredthetas)
hold on
plot(T,thetads)
legend("Car θ","Desired Trajectory θ","Θd")
xlabel("Time in sec")
ylabel("Theta position in rad")
title("Orientation in time plot")
subplot(2,1,2)
plot(T,e_thetas)
hold on 
plot(T,r3(T))
hold on 
plot(T,-r3(T))
legend("Error θ","rθ","-rθ")
xlabel("Time in sec")
ylabel("Error θ")
title("eθ plot")
figure
subplot(2,1,1)
plot(T,X(:,4))
hold on
plot(T(1:end-1),desiredveloc)
hold on
plot(T,vds)
legend("Car velocity","Desired Trajectory Vel","Vd")
xlabel("Time in sec")
ylabel("Velocity position in m/s")
title("Velocity in time plot")
subplot(2,1,2)
plot(T,e_vs)
hold on 
plot(T,r4(T))
hold on 
plot(T,-r4(T))
legend("Error V","rV","-rV")
xlabel("Time in sec")
ylabel("Error v")
title("eV plot")
figure
subplot(2,1,1)
plot(T,ksi_thetas)
xlabel("Time in sec")
ylabel("Normalized error θ")
title("ξθ plot")
subplot(2,1,2)
plot(T,ksi_vs)
xlabel("Time in sec")
ylabel("Normalized error V")
title("ξV plot")
us = zeros(size(T));
for j = 1:length(T) 
    us(j) = -ktheta*ppcTrans(ksi_thetas(j));
end
ds = atan(us);
dsdeg = 180*ds/pi;
figure 
subplot(2,1,1)
plot(T,ds)
xlabel("Time in sec")
ylabel("Angle in rad")
title("Steering angle in rad")
subplot(2,1,2)
plot(T,dsdeg)
xlabel("Time in sec")
ylabel("Angle in deg")
title("Steering angle in deg")
%% Animation of movement of cars
n = 1;
N_veh = 1;
vehicle_length = Lcar*ones(1,N_veh); % (m)
%vehicle_length = 0.1; % (m)
vehicle_width  = Wcar*ones(1,N_veh); % (m)
plotAxisLimits = [-200 200 -100 100]*1; % [xmin xmax ymin ymax]
anim_fps=5; % (animation frames / second)
enable_CG_trace=0;       % (0/1) plot animation trace from vehicle CG, or geometric center
enable_rearAxle_trace = 1; % (0/1) enable animation trace from rear axle
axisMode = 0; % 0->auto, 1->fixed, use Axes Limits in 'plotAxisLimits'
save_anim_frames = 0; % (0/1) save animation frames? this slows the simulation considerably
                    %       when writing a .jpeg image to file at each animation interval. 
                    %       see writeVideo() at this link for converting into .avi movies:
                    %       http://www.mathworks.com/help/matlab/examples/convert-be
fname = multiagentFigureInitialize(N_veh,Lcar,Wcar,enable_CG_trace,enable_rearAxle_trace,1,1,save_anim_frames);
tempx = zeros(4*n,1);
for t = 1:20:length(T)
    cntr = t-1;
    tempx(1) = X(t,1)+Lcar/2*cos(X(t,3)); %X
    tempx(2) = X(t,2)+Lcar/2*sin(X(t,3)); %Y
    tempx(3) = X(t,3); %theta
    tempx(4) = atan(us(t));%X(t,4*(i-1)+2); %Y
    oneagentFigureUpdate(tempx,N_veh,cntr,Lcar,Wcar,enable_CG_trace,...
        enable_rearAxle_trace,save_anim_frames,axisMode,plotAxisLimits,maxa,1,1,fname)
  pause(0.05)
end

%nchoice = 2;
%}
