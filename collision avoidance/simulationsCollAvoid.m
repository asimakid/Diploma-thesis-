clear all
close all
clc
Ts = 0:0.01:10;
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
x0(4*(j-1)+1) = [1,2,3,7,12,15,18,22,30,40]*1; %init xi
x0(4*(j-1)+2) = [-6.5,+6.5,0,-5,-2,+3,-1.5,3,+1,0]; %init yi
x0(4*(j-1)+3) = [-0.01,+0.01,-0.02,+0.01,+0.01,-0.01,0.005,0.001,-0.001,-0.05]*10; %init thetai
x0(4*(j-1)+4) = [32,28,30,27,30,29,27,32,31,25]; %init vi
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
            problem = 1            
        end
    end 
end
kx = 1;
ky = 1;
kt = 10;
kv = 10;
kxs = kx*ones(n,1);
kys = ky*ones(n,1);
kts = kt*ones(n,1);
kvs = kv*ones(n,1);
thetal = 1;
vl = 1;
thetamarg = 3;
vmarg = 3;
thetainf = 2;
vinf = 2;
vstar = 30;
vmax = 35;
ksi0 = (exp(-vstar/kx)-1)/(exp(-vstar/kx)+1);
ksimax = (exp(-vmax/kx)-1)/(exp(-vmax/kx)+1);
global preva2;
global k;
preva2 = 0;
k = 0;
%Determining  initial perfonce functions for t,v
for i = 1:n     
       xi = x0(4*(i-1)+1);
       yi = x0(4*(i-1)+2);
       ti = x0(4*(i-1)+3);
       vi = x0(4*(i-1)+4);
       rdown  = xi-dc; 
       rupper = xi+dc;
       ruppery = yi+dc;
       rdowny  = yi-dc;
       temprdown  = xi-dc; 
       temprupper = xi+dc;
       tempruppery = yi+dc;
       temprdowny  = yi-dc;       
       if maxa  < yi+dc/2
            ruppery = maxa + 2*dc*(maxa-yi);
       end
       if mina  > yi - dc/2
            rdowny = yi - 2*dc*(mina-yi);
       end
       for j = 1:n
           if i == j 
               continue
           end
           if i == j
               fault = 1 
           end
           xj = x0(4*(j-1)+1);
           yj = x0(4*(j-1)+2);
           xn = abs(xi-xj)/distFunc(xi,xj,yi,yj);
           yn = abs(yi-yj)/distFunc(xi,xj,yi,yj);
           d = distFunc(xi,xj,yi,yj);
           if xi <= xj % fordward cars  
               temprupper = xi +dc - dc*(dc-d)/(dc-w);
               if temprupper < rupper 
                   rupper = temprupper;
               end 
               if yi < yj 
                  	tempruppery = yi + dc - dc*(dc-d)/(dc-w)*sigmoidFuncEx(yj-yi,0.001,10);
                    temprdowny  = yi - dc;
               else        
                    tempruppery = yi + dc;
                    temprdowny  = yi - dc + dc*(dc-d)/(dc-w)*sigmoidFuncEx(yi-yj,0.001,10);
               end
               if tempruppery < ruppery 
                   ruppery = tempruppery;
               end
               if temprdowny > rdowny
                   rdowny = temprdowny;
               end
           elseif xi > xj  % backward cars
               %xn = abs(xi-xj)/distFunc(xi,xj,yi,yj);
               %f = variableSigmoid(distFunc(xi,xj,yi,yj),w,w+d,gain);                        
               %temprdown =   (xj+w*xn)*f +(xi-w/2)*(1-f);
               temprdown = xi - dc + dc*xn*(dc-d)/(dc-w);
               if temprdown > rdown 
                   rdown = temprdown;
               end
               if yi <=  yj %back and up neighbours
                   tempruppery = yi + dc - dc*yn*(dc-d)/(dc-w);
               elseif yi > yj
                   temprdowny = yi - dc + dc*yn*(dc-d)/(dc-w);
               end
               if tempruppery <  ruppery
                  ruppery = tempruppery; 
               end
               if temprdowny > rdowny
                   j
                  rdowny = temprdowny
               end
           end
       end    
       ksi_x = 2*(xi -(rupper+rdown)/2)/(rupper-rdown)  
       ksi_y = 2*(yi -(ruppery+rdowny)/2)/(ruppery-rdowny)
       yi 
       rdowny 
       ruppery
       vyd = -vmax*saturationfunc(ksi_y,0.8);
       vdxmax = sqrt(vmax^2-vyd^2);
       vw = min([vdxmax,vstar]); 
       if ksi_x < 0
           vxd = vw-(vdxmax-vw)*saturationfunc(ksi_x,0.8);
       elseif ksi_x == 0
           vxd = vw;
       else
           vxd = vw - vw*saturationfunc(ksi_x,0.8);
       end
       vx = vi*cos(ti);
       vy = vi*sin(ti);
       %xdot(4*(i-1)+1) = vx;
       %xdot(4*(i-1)+2) = vy;
      % a2 = -ky*ppcTrans(ksi_y);
       % .. ppc like  for x 
       % get a1 
       % ppc like for y 
       %thetad0 = getThetad(vx0,vy0,a,gain);
      %vd0 = getVd(vx0,vy0);
       temprt =  @(t)((abs(vx-vxd)+thetamarg)*exp(-thetal*t)+thetainf);
       temprv =  @(t)((abs(vy-vyd)+vmarg)*exp(-vl*t)+vinf);
       rts{i} = temprt;
       rvs{i} = temprv;
end
opts = odeset('MaxStep',maxstep);
[T,X] = ode15s(@(t,x)collSystem2(t,x,n,kxs,kys,kts,kvs,rts,rvs,w,dc,gain,ksi0,ksimax,a,mina,maxa,vmax,vstar,ylim),Ts,x0,opts);
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
%{
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
