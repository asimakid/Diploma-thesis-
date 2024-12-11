function xdot  = carsSystem(t,x,n,A,m1,m2,p,L,lamda,q,c,vMax,vStar,f,e,a,tChange,ap)
    t
    if t >= tChange
        vStar = vStar(2);
    else
        vStar = vStar(1);
    end
    %For each car there are 4 states
    %For car i there are the states {4(i-1)+1,4*(i-1)+4}
    xdot = zeros(4*n,1);
    for i = 1 : n 
        xi = x(4*(i-1)+1);
        yi = x(4*(i-1)+2);
        thetai = x(4*(i-1)+3); 
        vi = x(4*(i-1)+4);        
        % x_idot
        xdot(4*(i-1)+1) = vi*cos(thetai);
        % y_idot 
        xdot(4*(i-1)+2) = vi*sin(thetai);
        % k gain function
        sumPotx = 0;
        sumPoty = 0;
        for j = 1 : n
            if j ~= i
                d = eclipseDistance(xi,yi,x(4*(j-1)+1),x(4*(j-1)+2),p);
                sumPotx = sumPotx + potentialVder(d,q,L,lamda)*((xi-x(4*(j-1)+1))/d);
                sumPoty = sumPoty + potentialVder(d,q,L,lamda)*((yi-x(4*(j-1)+2))/d);    
            end
        end 
        k = m2 + sumPotx/vStar + (vMax*cos(thetai)*ffunc(-sumPotx,e))/(vStar*(vMax*cos(thetai)-vStar));
        % Inputs provided to the car
        F = -(k/cos(thetai))*(vi*cos(thetai)-vStar) - sumPotx/cos(thetai);
        %{
        if potentialUder(yi,a,c) ~= 0
           yi 
           potentialUder(yi,a,c)
           i
        end
        %}
        u = -((vStar + A/(vi*(cos(thetai)-cos(f))^2))^-1)*(m1*vi*sin(thetai) + potentialUder(yi,ap,c) + p*sumPoty + sin(thetai)*F);
        % theta_idot
        xdot(4*(i-1)+3) =  u;
        % v_idot
        xdot(4*(i-1)+4) =  F;
    end
end

