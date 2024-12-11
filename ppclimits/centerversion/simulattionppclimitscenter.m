clear all
close all
clc
Ts = 0:0.01:10;
maxstep = 10^-2;
%Characteristics of car 
L = 5; %width in m diameter of cirlce
lamda = 20; %radius of circle of dc must bme bigger the w/2
if lamda < L
   error("L must be chosen bigger than lamda") 
end
maxa = 7.2;
mina = -7.2;
ylim = 0;
vstar = 30;
vmax = 35;
c = 1.5;  %constant
q = 3 *10^-3; %constant
%Number of cars 
n = 10;
j = 1:n;
vstar = 30*ones(n,1);
x0 = zeros(5*n,1);
x0(5*(j-1)+1) = [1,2,3,7,12,15,18,22,30,40]*5; %init xi
x0(5*(j-1)+2) = [-6.5,+6.5,0,-5,-2,+3,-1.5,3,+1,0]*1; %init yi
x0(5*(j-1)+3) = [0.01,-0.01,0.02,0.01,-0.01,+0.01,0.005,0.001,-0.01,-0.01]*1; %init thetai
x0(5*(j-1)+4) = [20,28,30,32,20,31,27,30,31,25]*1; %init vi
x0(5*(j-1)+5) = [0,0,0,0,0,0,0,0,0,0]*1; %init δi
ksi_vstar = 2*(vstar -(vmax)/2)/vmax;
ashift =  (1-ksi_vstar)./(1+ksi_vstar);
kapa = vstar/vmax;
kv = 0.1*ones(n,1);
ktheta = 10*ones(n,1);
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
        if  distFunc(x0(5*(i-1)+1),x0(5*(j-1)+1),x0(5*(i-1)+2),x0(5*(j-1)+2)) < L  
            i 
            j 
            distFunc(x0(5*(i-1)+1),x0(5*(j-1)+1),x0(5*(i-1)+2),x0(5*(j-1)+2))
            error("Initial distances error")             
        end
    end 
end
%{
global preva2;
global k;
preva2 = 0;
k = 0;
%}
%Determining  initial perfonce functions for t,v
opts = odeset('MaxStep',maxstep);
[T,X] = ode15s(@(t,x)ppclimitssystemcenter(t,x,n,lamda,L,maxa,vmax,vstar,ashift,kv,ktheta,c,q),Ts,x0,opts);
i = 1:n;
xs = X(:,5*(i-1)+1); 
ys = X(:,5*(i-1)+2);
ts = X(:,5*(i-1)+3);
vs = X(:,5*(i-1)+4);
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
title("Y pos of cars")
xlabel("Time in sec")
ylabel("Position in m")
legend(["1","2","3","4","5","6","7","8","9","10"])
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
legend(["1","2","3","4","5","6","7","8","9","10","Vmax"])
%%Theta of cars
figure 
for j = 1:n 
    plot(T,ts(:,j))
    hold on
end
title("Orientation angles of cars")
xlabel("Time in sec")
ylabel("Theta in rad/s")
legend(["1","2","3","4","5","6","7","8","9","10"])
%%Acceleration and angular rates of cars
us = zeros(size(X,1),10);
Fs = zeros(size(X,1),10);
for s = 1:length(T)
    for i = 1 : n 
        xi = X(s,5*(i-1)+1);
        yi = X(s,5*(i-1)+2);
        thetai = X(s,5*(i-1)+3);
        vi = X(s,5*(i-1)+4);
        vx = vi*cos(thetai);
        vy = vi*sin(thetai);        
        sfx = 0;
        sfy = potentialUder(yi,maxa,c);
        vmaxx = sqrt(vmax^2-vy^2);
        for j = 1:n
           if i == j 
               continue
           end
           xj = X(s,5*(j-1)+1);
           yj = X(s,5*(j-1)+2);
           d = distFunc(xi,xj,yi,yj);
           sfx = sfx + potentialVder(d,q,L,lamda)*((xi-xj)/d);
           sfy = sfy + potentialVder(d,q,L,lamda)*((yi-yj)/d);
        end
        ksi_y = vy/vmax;
        ksi_x = (vx-vmaxx/2)/(vmaxx/2);
        epsilon_y = ppcTrans(ksi_y);
        epsilon_x = ppcTransShifted(ksi_x,ashift(i));
        fy = -sfy - ktheta(i)*epsilon_y;
        vmaxxdot = - 2*vy*fy/(2*sqrt(vmax^2-vy^2));
        vxstardot = kapa(i)*vmaxxdot;
        fx = -sfx - kv(i)*epsilon_x - vxstardot;
        b = [fx;fy];
        A = [cos(thetai),sin(thetai);-sin(thetai)/vi^2,cos(thetai)/vi^2];
        res = A*b;
        us(s,i) = res(2);
        Fs(s,i) = res(1);
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
    plot(T,atan(X(:,5*(j-1)+5)))
    hold on
end
title("Steering angle of car")
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
distances = [];
figure
for t = 1:length(T)
    tempmin = 10000;
    for j = 1:n 
        for l = j + 1 : n
            dist = distFunc(X(t,5*(j-1)+1),X(t,5*(l-1)+1),X(t,5*(j-1)+2),X(t,5*(l-1)+2));
            if tempmin > dist
                tempmin = dist;
            end
        end
    end    
    distances = [distances;tempmin];
end
plot(T,distances)
title("Minimum circular distance between cars")
ylabel("Time in sec")
%Calculating lyap func
lyapValues = zeros(size(T));
for j = 1:size(T,1)
    lyapValues(j) = lyapFuncPpcCenter(X(j,:),n,vstar,vmax,kapa,q,L,lamda,maxa,c,us(j,:));
end
issorted(lyapValues(10:end),'descend') 
figure 
plot(T,lyapValues)
title("Lyap func values")
xlabel("Time in sec")
%{
figure 
plot(T,distFunc(X(:,1),X(:,5),X(:,2),X(:,6)))
hold on
plot(T,distFunc(X(:,5),X(:,9),X(:,6),X(:,10)))
%}

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
axis([0,2500,mina,maxa])
for t = 1:10:length(T)
    for j = 1:n
        addpoints(curves(j),X(t,5*(j-1)+1),X(t,5*(j-1)+2));
        drawnow
        hold on
    end
end
title("Trajectories of cars")
xlabel("X position")
ylabel("Y position")
legend(["1","2","3","4","5","6","7","8","9","10"])
%}
%nchoice = 2;
