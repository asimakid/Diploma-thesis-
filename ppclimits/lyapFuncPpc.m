function res = lyapFuncPpc(x,n,kapa,vStar,q,L,lamda,a,ap,c,p)
    %For each car there are 4 states
    %For car i there are the states {4(i-1)+1,4*(i-1)+4}
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
    sum4 = 0;
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
        sum3 = sum3 + potentialU(yi,ap,c);
        %sum4
        for j = 1:n 
            if j == i
                continue
            else
                d = eclipseDistancePpc(xi,x(4*(j-1)+1),yi,x(4*(j-1)+2),p); 
                sum4 = sum4 + 0.5*potentialV(d,q,L,lamda);
            end
        end
    end
    res = sum1 + sum2 + sum3 + sum4;