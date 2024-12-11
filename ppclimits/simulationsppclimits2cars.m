clear all
close all
clc
Ts = 0:0.01:20;
maxstep = 10^-2;
Lcar = 4;
Wcar = 1.5;
%Characteristics of car 
maxa = 7.2; %m
mina = -7.2; %m
vstar = 30; %m/s
vmax = 35; %m/s
thetamax = 0.25;  %rad
ap = maxa-Lcar*sin(thetamax)-Wcar*cos(thetamax)/2;% 
c = 3;  %constant
q = 3*10^-1; %constant
p = 2; %constant
L = Lcar^2*max(4*p^2*sin(thetamax)^2,1+(p-1)*sin(thetamax)^2) ...
    +Wcar^2*p*max(4/p^2*sin(thetamax)^2,1+(1/p-1)*sin(thetamax)^2)...
    +Wcar^2*sin(thetamax)^2+Wcar^2*(1-cos(thetamax))^2/4 ...
    +4*(1+p)*Lcar*Wcar*sin(thetamax)+Wcar^2*sin(thetamax)...
    +Wcar*Lcar*(1-cos(thetamax))+Wcar^2*(1-cos(thetamax))/2; %width in m diameter of cirlce
L = sqrt(L)
lamda = 10; %radius of circle of dc must bme bigger the w/2
if lamda < L
   error("L must be chosen bigger than lamda") 
end
%Number of cars 
n = 2;
j = 1:n;
x0 = zeros(4*n,1);
x0(4*(j-1)+1) = [8.5,7.5]*2; %init xi
x0(4*(j-1)+2) = [-3,+5]*1; %init yi
x0(4*(j-1)+3) = [0.05,-0.12]*2; %init thetai
x0(4*(j-1)+4) = [26,28]*1; %init vi
ksi_vstar = 2*(vstar -(vmax)/2)/vmax;
ashift =  (1-ksi_vstar)/(1+ksi_vstar); 
kapa = vstar/vmax;
kv = 2;
ktheta = 5;
eclipseDistancePpc(x0(1),x0(5),x0(2),x0(6),p)
%{
%Initial x 
x0(4*(j-1)+1) = [0,5.5,7];
%Inital y
x0(4*(j-1)+2) = [-2,2,-4];
%Inital theta 
x0(4*(j-1)+3) = [0,-0.1,-0.1];
%Initial velocities 
x0(4*(j-1)+4) = [23,22,20];
%}
for i = 1 :n
    for j = i+1:n 
        if  eclipseDistancePpc(x0(4*(i-1)+1),x0(4*(j-1)+1),x0(4*(i-1)+2),x0(4*(j-1)+2),p) < L  
            i 
            j 
            eclipseDistancePpc(x0(4*(i-1)+1),x0(4*(j-1)+1),x0(4*(i-1)+2),x0(4*(j-1)+2),p)
            error("Initial distances error")             
        end
    end 
end
%Determining  initial perfonce functions for t,v
opts = odeset('MaxStep',maxstep);
[T,X] = ode15s(@(t,x)ppclimitssystem(t,x,n,lamda,L,maxa,ap,vmax,thetamax,ashift,kv,ktheta,c,q,p),Ts,x0,opts);
i = 1:n;
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
legend(["1","2"])
figure 
for j = 1:n
    plot(T,ys(:,j))
    hold on
end
plot(T,ap*ones(size(T)))
plot(T,-ap*ones(size(T)))
title("Y pos of cars")
xlabel("Time in sec")
ylabel("Position in m")
legend(["1","2","Uppery","Downy"])
%%Velocities of cars
figure 
for j = 1:n
    plot(T,vs(:,j))
    hold on
end
plot(T,vmax*ones(size(T)))
title("Velocties of cars")
xlabel("Time in sec")
ylabel("Velocities in m/s")
legend(["1","2","Vmax"])
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
legend(["1","2","Θmax","-Θmax"])
%%Acceleration and angular rates of cars
us = zeros(size(X,1),10);
Fs = zeros(size(X,1),10);
for s = 1:length(T)
    for i = 1 : n 
        xi = X(s,4*(i-1)+1);
        yi = X(s,4*(i-1)+2);
        ti = X(s,4*(i-1)+3);
        vi = X(s,4*(i-1)+4);
        vx = vi*cos(ti);
        vy = vi*sin(ti);        
        sfx = 0;
        sfy = potentialUder(yi,ap,c);
        for j = 1:n
           if i == j 
               continue
           end
           xj = X(s,4*(j-1)+1);
           yj = X(s,4*(j-1)+2);
           d = eclipseDistancePpc(xi,xj,yi,yj,p);
           sfx = sfx + potentialVder(d,q,L,lamda)*((xi-xj)/d);
           sfy = sfy + p*potentialVder(d,q,L,lamda)*((yi-yj)/d);
           d = distFunc(xi,xj,yi,yj);
        end
        ksi_v = (vi-vmax/2)/(vmax/2);
        ksi_theta = ti/thetamax;
        epsilon_theta = ppcTrans(ksi_theta);
        epsilon_v = ppcTransShifted(ksi_v,ashift);
        us(s,i) = (sfx*sin(ti)-sfy*cos(ti)...
           +sin(ti)*kv*epsilon_v/(1+cos(ti))-ktheta*epsilon_theta)/vi^2;
        Fs(s,i) = -sfx*cos(ti)-sfy*sin(ti)-kv*epsilon_v;
    end 
end
%Plots of accelerations and theta rates
figure
subplot(2,1,1)
for j = 1:n
    plot(T,atan(us(:,j)))
    hold on
end
title("Steering angle of cars in rad")
xlabel("Time in sec")
ylabel("δ angle in rad")
legend(["1","2"])
subplot(2,1,2)
for j = 1:n
    plot(T,180*atan(us(:,j))/pi)
    hold on
end
title("Steering angle of cars in deg")
xlabel("Time in sec")
ylabel("δ angle in deg")
legend(["1","2"])
%Plots of accelerations and theta rates
figure
for j = 1:n
    plot(T,us(:,j))
    hold on
end
title("Angular rates of cars")
xlabel("Time in sec")
ylabel("Theta rates in rad/s^2")
legend(["1","2"])
%Acceleration plots
figure
for j = 1:n
    plot(T,Fs(:,j))
    hold on
end
title("Acceleration of cars")
xlabel("Time in sec")
ylabel("Accel in m/s^2")
legend(["1","2"])
figure
plot(T,max(abs(Fs),[],2))
title("Norm of Acceleration of cars")
xlabel("Time in sec")
ylabel("Accel in m/s^2") 
figure
plot(T,max(abs(us),[],2))
title("Norm of rotation rate of cars")
distances = [];
figure
for t = 1:length(T)
    tempmin = 10000;
    for j = 1:n 
        for l = j + 1 : n
            dist = eclipseDistancePpc(X(t,4*(j-1)+1),X(t,4*(l-1)+1),X(t,4*(j-1)+2),X(t,4*(l-1)+2),p);
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
%Calculating lyap func
lyapValues = zeros(size(T));
for j = 1:size(T,1)
    lyapValues(j) = lyapFuncPpc(X(j,:),n,kapa,vstar,q,L,lamda,maxa,ap,c,p);
end
issorted(lyapValues(1:end),'descend')
figure 
plot(T,lyapValues)
title("Lyap func values")
xlabel("Time in sec")
%{
colors = ["r","g","b","c","m","y","k"];
figure
curves = [];
for j = 1:n 
   curves = [curves,animatedline('Color',colors(mod(j,7)+1))];
end
axis([0,2500,mina,maxa])
for t = 1:10:length(T)
    for j = 1:n
        addpoints(curves(j),X(t,4*(j-1)+1),X(t,4*(j-1)+2));
        drawnow
        hold on
    end
end
title("Trajectories of cars")
xlabel("X position")
ylabel("Y position")
legend(["1","2"])
 %}
%% Animation of movement of cars
N_veh = n;
vehicle_length = Lcar*ones(1,N_veh); % (m)
%vehicle_length = 0.1; % (m)
vehicle_width  = Wcar*ones(1,N_veh); % (m)
plotAxisLimits = [0 100 -maxa maxa]*1; % [xmin xmax ymin ymax]
anim_fps=5; % (animation frames / second)
enable_CG_trace=0;       % (0/1) plot animation trace from vehicle CG, or geometric center
enable_rearAxle_trace = 0; % (0/1) enable animation trace from rear axle
axisMode = 0; % 0->auto, 1->fixed, use Axes Limits in 'plotAxisLimits'
save_anim_frames= 0; % (0/1) save animation frames? this slows the simulation considerably
                    %       when writing a .jpeg image to file at each animation interval. 
                    %       see writeVideo() at this link for converting into .avi movies:
                    %       http://www.mathworks.com/help/matlab/examples/convert-be
fname = multiagentFigureInitialize(N_veh,Lcar,Wcar,enable_CG_trace,enable_rearAxle_trace,1,1,save_anim_frames)
tempx = zeros(4*n,1);
L = Lcar;
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
  pause(0.05)
end

%nchoice = 2;
%}