function xdot = collSystem(t,x,n,kxs,kys,kts,kvs,rts,rvs,w,d,gain,ksi0,ksimax,a,mina,maxa)
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
       kx = kxs(i);
       ky = kys(i);
       rdown  = xi-w/2;
       %ksid is given by the cars from behind
       %ksid determines the pushing force from the 
       %cars behind if possible         
       ksid = ksi0;
       for j = 1:n
           xj = x(4*(j-1)+1);
           yj = x(4*(j-1)+2);
           xn = abs(xi-xj)/distFunc(xi,xj,yi,yj);
           %g = 1 - variableSigmoid(abs(xi-xj)/distFunc(xi,xj,yi,yj),0,1,gain);
           f = variableSigmoid(distFunc(xi,xj,yi,yj),w,w+d,gain);
           if xi > xj
               tempvdx  = sqrt((30 + 5*f*xn)^2-vi^2*sin(ti)^2);
               tempksid = (exp(tempvdx/(-kx))-1)/(exp(tempvdx/(-kx))+1)
               %tempksid = ksi0 + (ksimax-ksi0)*f*xn;
               % ksid is the maximum negative between the different
               if tempksid < ksid
                   ksid = tempksid;
               end
           end
       end
       %rupper is the minimum rupper from the front cars
       rupper = xi + w*(1-ksid)/(2+2*ksid);
       for j = 1:n
           if i == j 
               continue
           end
           xj = x(4*(j-1)+1);
           yj = x(4*(j-1)+2);
           if xi <= xj 
               %xi 
               %xj
               %abs(xi-xj)
               %distFunc(xi,xj,yi,yj)
               xn = abs(xi-xj)/distFunc(xi,xj,yi,yj);
               f = variableSigmoid(distFunc(xi,xj,yi,yj),w,w+d,gain);                        
               temprupper = (xi+ w*(1-ksid)/(2+2*ksid))*(1-f) + (xj-w*xn)*f;
               if temprupper < rupper
                   rupper = temprupper;
               end
           end
       end 
       xi;
       rupper;
       rdown;
       ksi_x = 2*(xi -(rupper+rdown)/2)/(rupper-rdown);
       a1 = -kx*ppcTrans(ksi_x);
       %Determining ksi_y
       ruppery = maxa;
       rdowny  = mina;
       for j = 1:n
           if i == j 
               continue
           end
           xj = x(4*(j-1)+1);
           yj = x(4*(j-1)+2);
           %rupper gets smaller only from the upper neighs
           yn = abs(yi-yj)/distFunc(xi,xj,yi,yj);
           %g = 1 - variableSigmoid(abs(yi-yj)/distFunc(xi,xj,yi,yj),0,1,gain);           
           f = variableSigmoid(distFunc(xi,xj,yi,yj),w,w+d,gain);
           if yj >= yi
                tempruppery = (maxa)*(1-f) + (yj-w*yn)*(f);
                if tempruppery < ruppery
                    ruppery = tempruppery;
                end
           else
                temprdowny = (mina)*(1-f) + (yj+w*yn)*(f);
                if temprdowny > rdowny
                    rdowny = temprdowny;
                end
           end
           
       end
       ksi_y = 2*(yi -(ruppery+rdowny)/2)/(ruppery-rdowny);
       a2 = -ky*ppcTrans(ksi_y);
       % .. ppc like  for x 
       % get a1 
       % ppc like for y 
       % get a2 
       thetad = getThetad(a1,a2,a,gain);
       vd = getVd(a1,a2);  
       rt = rts{i};
       rv = rvs{i};
       kt = kts(i);
       kv = kvs(i); 
       ksi_theta = (ti-thetad)/rt(t);
       ksi_v =  (vi-vd)/rv(t);
      % ksi_v
      % kv
       u = -kt*ppcTrans(ksi_theta);
       F = -kv*ppcTrans(ksi_v);       
       xdot(4*(i-1)+3) = vi*u/w;
       xdot(4*(i-1)+4) = F;
    end
end