function xdot = ppclimitssystem(t,x,n,lamda,L,maxa,ap,vmax,thetamax,ashift,kv,ktheta,c,q,p)  
    t
    % n is the number of the cars
    xdot = zeros(4*n,1);
    for i = 1:n
       xi = x(4*(i-1)+1);
       yi = x(4*(i-1)+2);
       ti = x(4*(i-1)+3);
       vi = x(4*(i-1)+4);
       xdot(4*(i-1)+1) = vi*cos(ti);
       xdot(4*(i-1)+2) = vi*sin(ti); 
       sfx = 0;
       sfy = potentialUder(yi,ap,c);
       for j = 1:n
           if i == j 
               continue
           end
           xj = x(4*(j-1)+1);
           yj = x(4*(j-1)+2);
           d = eclipseDistancePpc(xi,xj,yi,yj,p);
           sfx = sfx + potentialVder(d,q,L,lamda)*((xi-xj)/d);
           sfy = sfy + p*potentialVder(d,q,L,lamda)*((yi-yj)/d);
       end  
       ksi_v = (vi-vmax/2)/(vmax/2);
       ksi_theta = ti/thetamax;
       if abs(ksi_v) > 1
           error("Out of bounds velocity")
       end
        if abs(ksi_theta) > 1
           error("Out of bounds theta")
       end
       epsilon_theta = ppcTrans(ksi_theta);
       epsilon_v = ppcTransShifted(ksi_v,ashift);
       xdot(4*(i-1)+3) = (sfx*sin(ti)-sfy*cos(ti)...
           +sin(ti)*kv*epsilon_v/(1+cos(ti))-ktheta*epsilon_theta)/vi;
       %xdot(4*(i-1)+4) = -sfx*cos(ti)-sfy*sin(ti)-kv*epsilon_v;
       
       if epsilon_v < 0 
            xdot(4*(i-1)+4) = -sfx*cos(ti)-sfy*sin(ti)-kv*epsilon_v;
       elseif epsilon_v >= 0 && epsilon_v < 0.9
           xdot(4*(i-1)+4) = -sfx*cos(ti)-sfy*sin(ti)-(kv/4 +3*kv/4*epsilon_v/0.9)*epsilon_v;
       else
           xdot(4*(i-1)+4) = -sfx*cos(ti)-sfy*sin(ti)-kv*epsilon_v;
        end
       
    end
end