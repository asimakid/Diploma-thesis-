function res = lyapFunc(x,n,f,vStar,A,q,L,lamda,p,a,c,tChange,t,ap)
    if t >= tChange
        vStar = vStar(2);
    else
        vStar = vStar(1);
    end
    %For each car there are 4 states
    %For car i there are the states {4(i-1)+1,4*(i-1)+4}
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
    sum4 = 0;
    sum5 = 0;
    for i = 1:n
        xi = x(4*(i-1)+1);
        yi = x(4*(i-1)+2);
        thetai = x(4*(i-1)+3);
        vi = x(4*(i-1)+4);
        %sum1
        sum1 = sum1 + 0.5*(vi*cos(thetai)-vStar)^2;
        %sum2
        sum2 = sum2 + 0.5*(vi*sin(thetai))^2;
        %sum3
        sum3 = sum3 + potentialU(yi,a,c);
        %sum4
        for j = 1:n 
            if j == i
                continue
            else
                d = eclipseDistance(xi,yi,x(4*(j-1)+1),x(4*(j-1)+2),p); 
                sum4 = sum4 + 0.5*potentialV(d,q,L,lamda);
            end
        end
        %sum5
        sum5 = sum5 + A*(1/(cos(thetai)-cos(f))-1/(1-cos(f)));
    end
    res = sum1 + sum2 + sum3 + sum4 + sum5;
end