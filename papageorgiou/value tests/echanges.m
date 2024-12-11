clc 
clear all
close all
evalues = 0.1:0.1:0.4;
figure(1)
figure(2)
figure(3)
for e = evalues
    n = 10; % number of cars
    vMax = 35; %m/s
    a = 7.2;   %m
    f = 0.25;  %angle in rad
    p = 5.11;  %ecentrocity 
    L = 5.59;  %m
    %e = 0.2;   %constant
    m1 = 0.5;  %constant
    m2 = 0.1;  %constant
    q = 3 *10^-3; %constant
    lamda = 25; %m
    A = 1; %constant
    c = 1.5;  %constant
    vStar = [30;25]; %m/s
    %Simulation time and initial conditions
    tChange = 40;
    Ts = 0:0.1:100;
    x0 = zeros(4*n,1);
    i = 1:n;
    x0(4*(i-1)+1) = [1,2,3,7,12,15,18,22,30,40]*10; %init xi
    x0(4*(i-1)+2) = [-6.5,+6.5,0,+5,-2,+3,-0.5,+5,+1,0]; %init yi
    x0(4*(i-1)+3) = [-0.01,+0.01,-0.05,+0.01,+0.01,-0.01,0.005,0.001,-0.001,0.05]; %init thetai
    x0(4*(i-1)+4) = [32,28,30,27,34,31,27,26,31,25]; %init vi
    opts = odeset('Maxstep',10^-2);%,'AbsTol',1,'RelTol',1);
    [T,X] = ode15s(@(t,x)carsSystem(t,x,n,A,m1,m2,p,L,lamda,q,c,vMax,vStar,f,e,a,tChange),Ts,x0,opts);
    xs = X(:,4*(i-1)+1); 
    ys = X(:,4*(i-1)+2);
    ts = X(:,4*(i-1)+3);
    vs = X(:,4*(i-1)+4);
    %Acceleration and angular rates of cars
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
            u = -((vStartemp + A/(vi*(cos(thetai)-cos(f))^2))^-1)*(m1*vi*sin(thetai) + potentialUder(yi,a,c) + p*sumPoty + sin(thetai)*F);
            us(s,i) = u;
            Fs(s,i) = F;
        end         
    end
     %Calculating lyap func
    lyapValues = zeros(size(T));
    for j = 1:size(T,1)
        lyapValues(j) = lyapFunc(X(j,:),n,f,vStar,A,q,L,lamda,p,a,c,tChange,T(j));
    end
    figure(1)
    plot(T,max(abs(Fs),[],2))
    hold on 
    figure(2)   
    plot(T,lyapValues)
    hold on
    figure(3)
    plot(T,max(abs(us),[],2))
    hold on
end
figure(1)
title("Norm of Acceleration of cars for different e values")
xlabel("Time in sec")
ylabel("Accel in m/s^2")
legend(instr(evalues))
figure(2)
title("Lyap func values for different e values")
xlabel("Time in sec")
legend(instr(evalues))
figure(3)
title("Norm of rotation rates of cars for different e values")
xlabel("Time in sec")
ylabel("Accel in rad/s^2")