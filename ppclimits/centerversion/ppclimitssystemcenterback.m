function xdot = ppclimitssystemcenterback(t,x,n,lamda,L,mina,maxa,vmax,vstar,ylim,ashift,kx,ky,kapa,c,q)  
    t
    % n is the number of the cars
    xdot = zeros(4*n,1);
    for i = 1:n
       xi = x(4*(i-1)+1);
       yi = x(4*(i-1)+2);
       ti = x(4*(i-1)+3);
       vi = x(4*(i-1)+4);
       xci = xi + cos(ti);
       yci = yi + sin(ti);
       vx = vi*cos(ti);
       vy = vi*sin(ti);
       xdot(4*(i-1)+1) = vi*cos(ti);
       xdot(4*(i-1)+2) = vi*sin(ti);   
       sfx = 0;
       sfy = potentialUder(yi,maxa,c);
       for j = 1:n
           if i == j 
               continue
           end
           xj = x(4*(j-1)+1);
           yj = x(4*(j-1)+2);
           tj = x(4*(j-1)+3);
           xcj = xj + cos(tj);
           ycj = yj + sin(tj);
           d = distFunc(xi,xj,yci,ycj);
           sfx = sfx + potentialVder(d,q,L,lamda)*((xci-xcj)/d);
           sfy = sfy + potentialVder(d,q,L,lamda)*((yci-ycj)/d);
       end
       vmaxx = sqrt(vmax^2-vy^2);
       if vy >= vmax
          problem = 1; 
       end
       if isreal(vmaxx)
       else 
           vmaxx 
           vmax
           vy
           s =1;
       end
       ksi_y = vy/vmax;
       ksi_x = (vx-vmaxx/2)/(vmaxx/2);
       ksi_v = (vi-vmax/2)/(vmax/2);
       ksi_theta = ti/(pi/2);
       %epsilon_y = ppcTrans(ksi_y);
       %epsilon_x = ppcTransShifted(ksi_v,ashift);
       epsilon_theta = ppcTrans(ksi_theta);
       epsilon_v = ppcTransShifted(ksi_v,ashift);
       %fy = -sfy - ky*epsilon_y;
       %vmaxxdot = - 2*vy*fy/(2*sqrt(vmax^2-vy^2));
       %vxstardot = kapa*vmaxxdot;
       %fx = -sfx - kx*(vi*cos(ti)+vstar)*epsilon_x; %- vxstardot;
       fx = -sfx;%-(vi*cos(ti)+vstar)*kx*epsilon_v;
       fy = -sfy;%-vi*sin(ti)*kx*epsilon_v - ky*epsilon_theta;
       b = [fx;fy];
       A = [cos(ti),sin(ti);-sin(ti)/vi^2,cos(ti)/vi^2];
       res = A*b;
       xdot(4*(i-1)+3) = vi*res(2)-ky*epsilon_theta;%+sin(ti)*kx*epsilon_v/(vi*(vi*cos(ti)+vstar));
       xdot(4*(i-1)+4) = res(1)-kx*epsilon_v;
    end
end