function res = lyapFuncPpcCenter(x,n,vStar,vmax,kapa,q,L,lamda,a,c,uis)
    %For each car there are 5 states
    %For car i there are the states {5(i-1)+1,5*(i-1)+5}
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
    sum4 = 0;
    for i = 1:n
        xi = x(5*(i-1)+1);
        yi = x(5*(i-1)+2);
        thetai = x(5*(i-1)+3);
        vi = x(5*(i-1)+4);
        vy = vi*sin(thetai);
        vmaxx = sqrt(vmax^2-vy^2);
        %sum1
        sum1 = sum1 + 0.5*(vi*cos(thetai)-0.5*vi*uis(i)*sin(thetai)-vStar(i))^2;
        %sum2
        sum2 = sum2 + 0.5*(vi*sin(thetai)+0.5*vi*uis(i)*cos(thetai))^2;
        %sum3
        sum3 = sum3 + potentialU(yi,a,c);
        %sum4
        for j = 1:n 
            if j == i
                continue
            else
                d = distFunc(xi,x(5*(j-1)+1),yi,x(5*(j-1)+2)); 
                sum4 = sum4 + 0.5*potentialV(d,q,L,lamda);
            end
        end
    end
    res = sum1 + sum2 + sum3 + sum4;