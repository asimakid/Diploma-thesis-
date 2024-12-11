clc
clear all
close all
%Road and car characteristics
aRoad = 7.2; %m
a = 1;%m
x0 = [0,-2,0.1,22,0]; %initial cond
vMax = 30; %m/s
flim = 0.01; %rad
f =  1; %rad
uLim =  vMax*sin(f);
%Desires results
yStar =  1; %m
thetaStar = 0; %rad
vStar =  25; %m/s
yemax  = abs(x0(2)-yStar)+1;
yeinf =  0.01;
yl =  2;
thetaeinf =  0.01;
thetal =  1;
veinf =  0.01;
vl =  1;
ky = 10;
kt = 0.01;
kv = 10;
r_y = @(t)((yemax-yeinf)*exp(-yl*t)+yeinf);
ksi_y0 = (x0(2)-yStar-x0(5))/r_y(0);
u_y0 = -ky*ppcTrans(ksi_y0);
u_y0 = (abs(u_y0) > uLim)*uLim + (u_y0 <= uLim)*u_y0;
[desiredV0,desiredTheta0] = calculateDesired(u_y0,vStar,vMax,flim,f);
thetaemax = abs(x0(3)- desiredTheta0)+0.1;
vemax = abs(x0(4)- desiredV0)+ 0.1; 
r_theta = @(t)((thetaemax-thetaeinf)*exp(-thetal*t)+thetaeinf); 
r_V = @(t)((vemax-veinf)*exp(-vl*t)+veinf); 
Ts = 0:0.001:2;
opts = odeset('MaxStep',10^-4);
[T,X] = ode45(@(t,x)kinematic(t,x,a,yStar,r_y,thetaStar,r_theta,vStar,vMax,f,flim,uLim,r_V,ky,ky,kv),Ts,x0,opts);
dts = zeros(size(T));
dvs = zeros(size(T));
uyuns = zeros(size(T));
for i = 1:length(T)
    ksi_yt = (X(i,2)-yStar-X(i,5))/r_y(T(i));
    u_yt = -ky*ppcTrans(ksi_yt);
    uytuns = u_yt;
    u_yt = (abs(u_yt) > uLim).*uLim.*sign(u_yt) + (abs(u_yt) <= uLim).*u_yt; 
    [desiredV,desiredTheta] = calculateDesired(u_yt,vStar,vMax,flim,f);
    dvs(i) = desiredV;
    dts(i) = desiredTheta;
    uyuns(i) = uytuns;
end
figure
subplot(2,1,1)
plot(T,dts)
subplot(2,1,2)
plot(T,dvs)
figure 
plot(T,X(:,3))
hold on
plot(T,r_theta(T)+dts)%,'color',"r")
hold on
plot(T,-r_theta(T)+dts)%,'color',"r")
legend("Y","R_y","-R_y")
title("Theta")
figure 
plot(T,X(:,4))
hold on
plot(T,r_V(T)+dvs)%,'color',"r")
hold on
plot(T,-r_V(T)+dvs)%,'color',"r")
legend("Y","R_y","-R_y")
title("Velocity of body")
figure 
plot(T,X(:,5))
%hold on
%plot(T,X(:,2))
%legend("s","y")
title("S in comparison with y")
figure 
plot(T,X(:,1))
title("X position")
figure 
plot(T,X(:,2))
hold on
plot(T,r_y(T)+X(:,5)+yStar)%,'color',"r")
hold on
plot(T,-r_y(T)+X(:,5)+yStar)%,'color',"r")
legend("Y","R_y","-R_y")
title("Y position") 
figure
plot(T,X(:,2)-yStar-X(:,5))
hold on
plot(T,r_y(T))%,'color',"r")
hold on
plot(T,-r_y(T))%,'color',"r")
hold on 
plot(T,X(:,2)-yStar)
legend("Y with s","R_y","-R_y","Y without s")
title("Y position Error comparison") 
figure
plot(T,X(:,4)-dvs)
hold on
plot(T,r_V(T))%,'color',"r")
hold on
plot(T,-r_V(T))%,'color',"r")
legend("Error","R_veloc","-R_veloc")
title("Velocity of body error")
figure
plot(T,X(:,3)-dts)
hold on
plot(T,r_theta(T))%,'color',"r")
hold on
plot(T,-r_theta(T))%,'color',"r")
legend("Error","R_theta","-R_theta")
title("Theta of body error")
figure
subplot(2,1,1)
plot(T,uyuns)
subplot(2,1,2)
plot(T,abs(uyuns)>uLim)