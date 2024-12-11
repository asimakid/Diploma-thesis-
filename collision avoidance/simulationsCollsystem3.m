clear all
close all
clc
Ts = 0:0.01:4;
maxstep = 10^-4;
%Characteristics of car 
w = 5; %width in m diameter of cirlce
dc = 10; %radius of circle of dc must bme bigger the w/2
if dc < w/2
   error("dc must be chosen bigger than w/2") 
end
gain = 2;
a = 0.01;
maxa = 7.2;
mina = -7.2;
ylim = 0;
%Number of cars 
n = 10;
j = 1:n;
x0 = zeros(6*n,1);
x0(6*(j-1)+1) = [1,2,3,7,12,15,18,22,30,40]*1; %init xi
x0(6*(j-1)+2) = [-6.5,+6.5,0,-5,-2,+3,-1.5,3,+1,0]; %init yi
x0(6*(j-1)+3) = [0.01,+0.01,-0.02,+0.01,+0.01,-0.01,0.005,0.001,-0.001,-0.05]*5; %init thetai
x0(6*(j-1)+4) = [32,28,30,27,30,29,27,32,31,25]; %init vi
%{
%Initial x 
x0(6*(j-1)+1) = [0,5.5,7];
%Inital y
x0(6*(j-1)+2) = [-2,2,-4];
%Inital theta 
x0(6*(j-1)+3) = [0,-0.1,-0.1];
%Initial velocities 
x0(6*(j-1)+4) = [23,22,20];
%}
for i = 1 :n
    for j = i+1:n 
        if  distFunc(x0(6*(i-1)+1),x0(6*(j-1)+1),x0(6*(i-1)+2),x0(6*(j-1)+2)) < w  
            i 
            j 
            distFunc(x0(6*(i-1)+1),x0(6*(j-1)+1),x0(6*(i-1)+2),x0(6*(j-1)+2))
            problem = 1            
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
for i = 1:n     
       xi = x0(6*(i-1)+1);
       yi = x0(6*(i-1)+2);
       ti = x0(6*(i-1)+3);
       vi = x0(6*(i-1)+4);
       vx = vi*cos(ti);
       vy = vi*sin(ti);
       tempsy = -vy/vmax;
       vdxmax = sqrt(vmax^2-vy^2)
       vw = min([vdxmax,vstar]); 
       vx
       vy
       vw
       if vx > vw
            tempsx = (vw-vx)/(vdxmax-vw)
       elseif vx < vw 
           tempsx =  (vw - vx)/vw;
       else
           tempsx = 0;
       end
       x0(6*(i-1)+5) = ppcTrans(tempsx);
       x0(6*(i-1)+6) = ppcTrans(tempsy);
end
opts = odeset('MaxStep',maxstep);
[T,X] = ode15s(@(t,x)collSystem3(t,x,n,w,dc,a,mina,maxa,vmax,vstar,ylim),Ts,x0,opts);
i = 1:n;
xs = X(:,6*(i-1)+1); 
ys = X(:,6*(i-1)+2);
ts = X(:,6*(i-1)+3);
vs = X(:,6*(i-1)+4);
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
            dist = distFunc(X(t,6*(j-1)+1),X(t,6*(l-1)+1),X(t,6*(j-1)+2),X(t,6*(l-1)+2));
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
axis([0,250,mina,maxa])
for t = 1:10:length(T)
    for j = 1:n
        addpoints(curves(j),X(t,6*(j-1)+1),X(t,6*(j-1)+2));
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
        xi = X(t,6*(nchoice-1)+1);
        yi = X(t,6*(nchoice-1)+2);
        for j = 1 : n 
            if j == nchoice 
                continue
            end
           xj = X(t,6*(j-1)+1);
           yj = X(t,6*(j-1)+2);       
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
