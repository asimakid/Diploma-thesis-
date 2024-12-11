clc 
clear all
close all
maxstep = 10^-3;
n = 10; % number of cars
Lcar = 4;
Wcar = 1.5;
vMax = 35; %m/s
a = 7.2;   %m
maxa = a;
mina = -a;
f = 0.25;  %angle in rad
thetamax = f;
L = 5.59;  %m
e = 0.2;   %constant
m1 = 0.5;  %constant
m2 = 0.1;  %constant
q = 3 *10^-2; %constant
ap = maxa-Lcar*sin(thetamax)-Wcar*cos(thetamax)/2;% 
c = 3;  %constant
p = 2; %constant
L = Lcar^2*max(4*p^2*sin(thetamax)^2,1+(p-1)*sin(thetamax)^2) ...
    +Wcar^2*p*max(4/p^2*sin(thetamax)^2,1+(1/p-1)*sin(thetamax)^2)...
    +Wcar^2*sin(thetamax)^2+Wcar^2*(1-cos(thetamax))^2/4 ...
    +4*(1+p)*Lcar*Wcar*sin(thetamax)+Wcar^2*sin(thetamax)...
    +Wcar*Lcar*(1-cos(thetamax))+Wcar^2*(1-cos(thetamax))/2; %width in m diameter of cirlce
L = sqrt(L);
lamda = 20; %m
A = 1; %constant
c = 3;  %constant
vStar = [30;25]; %m/s
%Simulation time and initial conditions
tChange = 120;
Ts = 0:0.01:10;
x0 = zeros(4*n,1);
i = 1:n;
x0 = zeros(4*n,1);

x0(4*(i-1)+1) = [1,2,3,7,10,15,18,24,30,32]*10; %init xi
x0(4*(i-1)+2) = [-6.5,+5,+3,-1,-3,4,-2.5,3,+3,-4]*0.8; %init yi
x0(4*(i-1)+3) = [0.01,-0.01,0.02,0.01,-0.01,+0.01,0.005,0.001,-0.01,-0.05]*4; %init thetai
x0(4*(i-1)+4) = [18,28,30,27,25,29,25,34,31,32]*1; %init vi
%{
x0(4*(i-1)+1) = [1,2,3,7,10,15,18,22,30,35]*7; %init xi
x0(4*(i-1)+2) = [-6.5,+6.5,0,-5,-3,+3,-1.5,3,+1,0]*0.8; %init yi
x0(4*(i-1)+3) = [0.01,-0.01,0.02,0.01,-0.01,+0.01,0.005,0.001,-0.01,-0.02]*10; %init thetai
x0(4*(i-1)+4) = [20,28,30,27,25,34,25,32,31,25]*1; %init vi
%}
opts = odeset('Maxstep',maxstep);%,'AbsTol',1,'RelTol',1);
[T,X] = ode15s(@(t,x)carsSystem(t,x,n,A,m1,m2,p,L,lamda,q,c,vMax,vStar,f,e,a,tChange,ap),Ts,x0,opts);
xs = X(:,4*(i-1)+1); 
ys = X(:,4*(i-1)+2);
ts = X(:,4*(i-1)+3);
vs = X(:,4*(i-1)+4);
figure 
for j = 1:n
    plot(T,xs(:,j))
    hold on
end
title("X pos of cars")
xlabel("Time in sec")
ylabel("Position in m")
legend(["1","2","3","4","5","6","7","8","9","10"])
figure 
for j = 1:n
    plot(T,ys(:,j))
    hold on
end
plot(T,ap*ones(size(T)))
hold on
plot(T,-ap*ones(size(T)))
title("Y pos of cars")
xlabel("Time in sec")
ylabel("Position in m")
legend(["1","2","3","4","5","6","7","8","9","10","Uppery","Downy"])
%%Velocities of cars
figure 
for j = 1:n
    plot(T,vs(:,j))
    hold on
end
plot(T,vMax*ones(size(T)))
title("Velocties of cars")
xlabel("Time in sec")
ylabel("Velocities in m/s")
legend(["1","2","3","4","5","6","7","8","9","10","Vmax"])
%%Theta of cars
figure 
for j = 1:n 
    plot(T,ts(:,j))
    hold on
end
plot(T,thetamax*ones(size(T)))
hold on
plot(T,-thetamax*ones(size(T)))
title("Orientation angles of cars")
xlabel("Time in sec")
ylabel("Theta in rad/s")
legend(["1","2","3","4","5","6","7","8","9","10","Θmax","-Θmax"])
%%Acceleration and angular rates of cars
us = zeros(size(X,1),10);
Fs = zeros(size(X,1),10);
for s = 1:length(T)
    if T(s) > tChange
        vStartemp = vStar(2);
    else
        vStartemp = vStar(1);
    end   
    for i = 1 : n 
        xi = X(s,4*(i-1)+1);
        yi = X(s,4*(i-1)+2);
        thetai = X(s,4*(i-1)+3);
        vi = X(s,4*(i-1)+4);
        sumPotx = 0;
        sumPoty = 0;
        for j = 1 : n
            if j ~= i
                d = eclipseDistance(xi,yi,X(s,4*(j-1)+1),X(s,4*(j-1)+2),p); 
                sumPotx = sumPotx + potentialVder(d,q,L,lamda)*((xi-X(s,4*(j-1)+1))/d);
                sumPoty = sumPoty + potentialVder(d,q,L,lamda)*((yi-X(s,4*(j-1)+2))/d);    
            end
        end 
        k = m2 + sumPotx/vStartemp + (vMax*cos(thetai)*ffunc(-sumPotx,e))/(vStartemp*(vMax*cos(thetai)-vStartemp));
        % Inputs provided to the car
        F = -(k/cos(thetai))*(vi*cos(thetai)-vStartemp) - sumPotx/cos(thetai);
        u = -((vStartemp + A/(vi*(cos(thetai)-cos(f))^2))^-1)*(m1*vi*sin(thetai) + potentialUder(yi,ap,c) + p*sumPoty + sin(thetai)*F);
        us(s,i) = u;
        Fs(s,i) = F;
    end 
end
%Plots of accelerations and theta rates
figure
for j = 1:n
    plot(T,us(:,j))
    hold on
end
title("Angular rates of cars")
xlabel("Time in sec")
ylabel("Theta rates in rad/s^2")
legend(["1","2","3","4","5","6","7","8","9","10"])
%Acceleration plots
figure
for j = 1:n
    plot(T,Fs(:,j))
    hold on
end
title("Acceleration of cars")
xlabel("Time in sec")
ylabel("Accel in m/s^2")
legend(["1","2","3","4","5","6","7","8","9","10"])
figure
for j = 1:n
    plot(T,atan(us(:,j)./X(:,4*(i-1)+4)))
    hold on
end
title("Steering angle of cars")
xlabel("Time in sec")
ylabel("Angle in rad")
legend(["1","2","3","4","5","6","7","8","9","10"])
figure
plot(T,max(abs(Fs),[],2))
title("Norm of Acceleration of cars")
xlabel("Time in sec")
ylabel("Accel in m/s^2") 
figure
plot(T,max(abs(us),[],2))
title("Norm of rotation rate of cars")
%Calculating lyap func
lyapValues = zeros(size(T));
for j = 1:size(T,1)
    lyapValues(j) = lyapFunc(X(j,:),n,f,vStar,A,q,L,lamda,p,a,c,tChange,T(j));
end
figure 
plot(T,lyapValues)
title("Lyap func values")
xlabel("Time in sec")
issorted(lyapValues(2:end),'descend') 
%{
figure
subplot(2,1,1)
plot(T,X(:,4*(4-1)+1))
hold on
plot(T,X(:,4*(7-1)+1))
legend("X4","X7")
subplot(2,1,2)
plot(T,X(:,4*(4-1)+2))
hold on
plot(T,X(:,4*(7-1)+2))
%}
%Calculate the eclipse distance between cars
distances = [];
figure
for t = 1:length(T)
    tempmin = 10000;
    for j = 1:n 
        for k = j + 1 : n
            dist = eclipseDistance(X(t,4*(j-1)+1),X(t,4*(j-1)+2),X(t,4*(k-1)+1),X(t,4*(k-1)+2),p);
            if tempmin > dist
                tempmin = dist;
            end
        end
    end    
    distances = [distances;tempmin];
end
plot(T,distances)
title("Minimum eclipse distance between cars")
ylabel("Time in sec")
maxthetas = zeros(size(T));
maxyidot = zeros(size(T));
%Calculating y dot 
for t = 1 : length(T)
    vis = X(t,4*(i-1)+4);
    thetas = X(t,4*(i-1)+3);
    maxthetas(t) = max(abs(thetas));
    maxyidot(t) = max(abs(vis.*sin(thetas)));
end
figure
plot(T,maxthetas)
title("Linf norm of thetas")
xlabel("Time in sec")
figure
plot(T,maxyidot)
title("Linf norm of yi dot")
xlabel("Time in sec")
%{
%Plot of the trajectories of cars
colors = ["r","g","b","c","m","y","k"];
figure
%plot(0:1:10000,a*(sqrt(c-1)/sqrt(c))*ones(size(0:1:1000)))
%hold on
%plot(0:1:10000,-a*(sqrt(c-1)/sqrt(c))*ones(size(0:1:1000)))
%hold on
curves = [];
for j = 1:n 
   curves = [curves,animatedline('Color',colors(mod(j,7)+1))];
end
axis([0,2000,-a,a])
for t = 1:length(T)
    for j = 1:n
        addpoints(curves(j),X(t,4*(j-1)+1),X(t,4*(j-1)+2));
        drawnow
        hold on
    end
end
title("Trajectories of cars")
xlabel("X position")
ylabel("Y position")
legend(["1","2","3","4","5","6","7","8","9","10"])
%}
%% Animation of movement of cars
N_veh = n;
vehicle_length = Lcar*ones(1,N_veh); % (m)
%vehicle_length = 0.1; % (m)
vehicle_width  = Wcar*ones(1,N_veh); % (m)
plotAxisLimits = [0 100 -maxa maxa]*1; % [xmin xmax ymin ymax]
anim_fps=25; % (animation frames / second)
enable_CG_trace=0;       % (0/1) plot animation trace from vehicle CG, or geometric center
enable_rearAxle_trace = 0; % (0/1) enable animation trace from rear axle
axisMode = 0; % 0->auto, 1->fixed, use Axes Limits in 'plotAxisLimits'
save_anim_frames=0; % (0/1) save animation frames? this slows the simulation considerably
                    %       when writing a .jpeg image to file at each animation interval. 
                    %       see writeVideo() at this link for converting into .avi movies:
                    %       http://www.mathworks.com/help/matlab/examples/convert-be
fname = multiagentFigureInitialize(N_veh,Lcar,Wcar,enable_CG_trace,enable_rearAxle_trace,1,1,save_anim_frames)
tempx = zeros(4*n,1);
for t = 1:4:length(T)
    cntr = t-1;
    for i = 1:n 
        tempx(4*(i-1)+1) = X(t,4*(i-1)+1)+Lcar/2*cos(X(t,4*(i-1)+3)); %X
        tempx(4*(i-1)+2) = X(t,4*(i-1)+2)+Lcar/2*sin(X(t,4*(i-1)+3)); %Y
        tempx(4*(i-1)+3) = X(t,4*(i-1)+3); %theta
        tempx(4*(i-1)+4) = atan(us(t,i));%X(t,4*(i-1)+2); %Y
    end
    multiagentFigureUpdate(tempx,N_veh,cntr,Lcar,Wcar,enable_CG_trace,...
        enable_rearAxle_trace,save_anim_frames,axisMode,plotAxisLimits,maxa,1,1,fname)
   %pause(0.05)
end
