clc
clear all
close all 
global preva2
global k 
k = 0;
x0 = [0,0,-0.5,0];
xd = @(t)(1*t); %+ 5*cos(t));
yd = @(t)(1000/(1+exp(-(t-5))));
%xd = @(t)(16*sin(t)^3);
%yd = @(t)(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t));
%x0 = [xd(0),yd(0),0,0];
opts = odeset('MaxStep',10^-4);%,'OutputFcn',@odeplot,'OutputSel',[1]);
Ts = 0:0.01:10;
kx = 100;
ky = 50;
ktheta = 10;
kv = 10000;
xemax = abs(x0(1) - xd(0))+0.1;
yemax = abs(x0(2) - yd(0))+1;
xeinf = 0.01;
yeinf = 0.01;
xl = 0.1;
yl = 0.1;
r1 = @(t)((xemax-xeinf)*exp(-xl*t)+xeinf);
r2 = @(t)((yemax-yeinf)*exp(-yl*t)+yeinf);
ksi_1 = (x0(1)-xd(0))/r1(0);
ksi_2 = (x0(2)-yd(0))/r2(0);
a1 = -kx*ppcTrans(ksi_1);
a2 = -ky*ppcTrans(ksi_2);
preva2 = a2;
a = 0.1;
gain = 1;
vd0 = getVd(a1,a2);
thetad0 = getThetad(a1,a2,a,gain);
%thetad0 = atan(a2/a1);
thetaemax = abs(x0(3) - thetad0) + 0.1;
vemax = abs(x0(4) -vd0) + 5;
thetaeinf = 0.1;
veinf = 0.1;
thetal = 0.1;
vl = 0.1;
r3 = @(t)((thetaemax-thetaeinf)*exp(-thetal*t)+thetaeinf)
r4 = @(t)((vemax-veinf)*exp(-vl*t)+veinf);
e = 1;
[T,X] = ode15s(@(t,x)trajectorysystem(t,x,xd,yd,r1,r2,r3,r4,kx,ky,ktheta,kv,a,gain),Ts,x0,opts);
figure
plot(T,X(:,1))
hold on 
plot(T,xd(T))
figure
plot(T,X(:,2))
hold on
plot(T,yd(T))
figure
plot(T,X(:,3))
figure
plot(T,X(:,4))
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
%Calculation of ksi and 
ksi1 = zeros(size(T));
ksi2 = zeros(size(T));
for i = 1 : length(ksi1)
    ksi1(i) = (X(i,1) - xd(T(i)))/r1(T(i));
    ksi2(i) = (X(i,2) - yd(T(i)))/r2(T(i));    
end
epsilon1 = ppcTrans(ksi1);
epsilon2 = ppcTrans(ksi2);
figure
plot(T,ksi1)
title("ksi1")
figure
plot(T,ksi2)
title("ksi 2")
figure 
plot(T,epsilon1)
title("epsilon 1")
figure
plot(T,epsilon2)
title("epsilon 2")