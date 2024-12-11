function xdot = ppclimitssystemcenter(t,x,n,lamda,L,maxa,vmax,vstar,ashift,kv,ktheta,c,q)  
    t
    % n is the number of the cars
    xdot = zeros(5*n,1);
    for i = 1:n
       xi = x(5*(i-1)+1);
       yi = x(5*(i-1)+2);
       ti = x(5*(i-1)+3);
       vi = x(5*(i-1)+4);
       ui = x(5*(i-1)+5);
       vx = vi*cos(ti);
       vy = vi*sin(ti);
       xdot(5*(i-1)+1) = vi*cos(ti);
       xdot(5*(i-1)+2) = vi*sin(ti);   
       sfx = 0;
       sfy = potentialUder(yi,maxa,c);
       for j = 1:n
           if i == j 
               continue
           end
           xj = x(5*(j-1)+1);
           yj = x(5*(j-1)+2);
           d = distFunc(xi,xj,yi,yj);
           sfx = sfx + potentialVder(d,q,L,lamda)*((xi-xj)/d);
           sfy = sfy + potentialVder(d,q,L,lamda)*((yi-yj)/d);
       end
       if abs(ti) > pi/2 || vi > vmax  
           vmax
           vi
           ti
           i
           ksi_v
           ksi_theta
           epsilon_v
           epsilon_theta
           s =1;
       end
       %{
       vmaxx = sqrt(vmax^2-vy^2);
       if vy >= vmax
          problem = 1; 
       end
       if isreal(vmaxx)
       else 
           vmaxx 
           vmax
           vx
           vy
           i
           s =1;
       end
       %}
       %ksi_y = vy/vmax;
       %ksi_x = (vx-vmaxx/2)/(vmaxx/2);
       ksi_v = (vi-vmax/2)/(vmax/2);
       ksi_theta = ti/(pi/2);
       %epsilon_y = ppcTrans(ksi_y);
       epsilon_v = ppcTransShifted(ksi_v,ashift(i));
       epsilon_theta = ppcTrans(ksi_theta);
       fy = -sfy; %- ky(i)*epsilon_y;
       %vmaxxdot = - vy*fy/sqrt(vmax^2-vy^2);
       %vxstardot = kapa(i)*vmaxxdot;
       fx = -sfx; %- kx(i)*epsilon_x ;%+ vxstardot ;
       b = [fx+vi*sin(ti)*vi*ui+0.5*vi*ui*cos(ti)*vi*ui;...
           fy-vi*cos(ti)*vi*ui+0.5*vi*ui*sin(ti)*vi*ui];
       A = [0.5*vi*cos(ti),-0.5*vi*sin(ti);-(sin(ti)+0.5*ui*sin(ti))/vi,(cos(ti)-0.5*ui*sin(ti))/vi];
       res = A*b;
       xdot(5*(i-1)+5) = res(2)+ kv(i)*epsilon_v*sin(ti)/(vi*cos(ti)+vstar(i))-2*vi*ui...
           -kv(i)*epsilon_v/vi-ktheta(i)*epsilon_theta;
       %{
       epsilon_v
       vi
       vstar(i)
       kv(i)
       s1 = kv(i)*epsilon_v*sin(ti)/(vi*cos(ti)+vstar(i))
       s2 = -ktheta(i)*epsilon_theta
       s3 = -kv(i)*epsilon_v/vi
       s4 = -2*vi*ui
           %}
       xdot(5*(i-1)+4) = res(1) + 0.5*vi^2*ui^2 - kv(i)*epsilon_v;
       xdot(5*(i-1)+3) = vi*ui;              
    end
end