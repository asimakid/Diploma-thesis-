clear all
close all
clc
Ts = 0:0.01:10;
maxstep = 10^-3;
%Characteristics of car 
w = 5; %width in m diameter of cirlce
dc = 10; %radius of circle of dc must bme bigger the w/2
if dc < w/2
   error("dc must be chosen bigger than w/2") 
end
gain = 2;
a = 0.01;
e = 0.5; % angle rad that 
maxa = 7.2;
mina = -7.2;
ylim = 0;
%Number of cars 
n = 10;
j = 1:n;
x0 = zeros(4*n,1);
x0(4*(j-1)+1) = [1,2,3,7,12,15,18,22,30,40]*5; %init xi
x0(4*(j-1)+2) = [-6.5,+6.5,0,-5,-2,+3,-1.5,3,+1,0]*1; %init yi
x0(4*(j-1)+3) = [0.01,+0.01,0.02,0.01,-0.01,+0.01,0.005,0.001,-0.01,-0.05]*2; %init thetai
x0(4*(j-1)+4) = [20,28,30,27,20,34,27,32,31,25]*1; %init vi
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
        if  distFunc(x0(4*(i-1)+1),x0(4*(j-1)+1),x0(4*(i-1)+2),x0(4*(j-1)+2)) < w  
            i 
            j 
            distFunc(x0(4*(i-1)+1),x0(4*(j-1)+1),x0(4*(i-1)+2),x0(4*(j-1)+2))
            error("Initial distances error")             
        end
    end 
end
vstar = 30;
vmax = 35;
global preva2;
global k;
preva2 = 0;
k = 0;
%Determining  initial perfonce functions for t,v
opts = odeset('MaxStep',maxstep);
[T,X] = ode15s(@(t,x)collSystemVeloc(t,x,n,w,dc,e,mina,maxa,vmax,vstar,ylim),Ts,x0,opts);
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
title("Velocties of cars")
xlabel("Time in sec")
ylabel("Velocities in m/s")
legend(["1","2","3","4","5","6","7","8","9","10"])
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
distances = [];
figure
for t = 1:length(T)
    tempmin = 10000;
    for j = 1:n 
        for l = j + 1 : n
            dist = distFunc(X(t,4*(j-1)+1),X(t,4*(l-1)+1),X(t,4*(j-1)+2),X(t,4*(l-1)+2));
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
%nchoice = 2;
sumdistances = zeros([length(T),n]);
for nchoice = 1 : n 
    for t = 1:length(T)
        xi = X(t,4*(nchoice-1)+1);
        yi = X(t,4*(nchoice-1)+2);
        for j = 1 : n 
            if j == nchoice 
                continue
            end
           xj = X(t,4*(j-1)+1);
           yj = X(t,4*(j-1)+2);       
           tempdist = distFunc(xi,xj,yi,yj);        
            sumdistances(t,nchoice) = sumdistances(t,nchoice) + 1/(tempdist-w);
        end
    end
end
%{
total = zeros(size(T));
for i = 1:length(total)
    for j = 1 : n 
       total(i) = sumdistances(i,j) + total(i); 
    end
end
%}
figure
for i = 1:n
    plot(T,sumdistances(:,i))
    hold on
end
legend(["1","2","3","4","5","6","7","8","9","10"])
title("Sum distances function")
