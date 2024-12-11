function xdot = collSystem2(t,x,n,kxs,kys,kts,kvs,rts,rvs,w,dc,gain,ksi0,ksimax,a,mina,maxa,vmax,vstar,ylim)  
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
            rdowny = yi - 2*dc*(yi-mina);
       end
       for j = 1:n
           if i == j 
               continue
           end
           if i == j
               fault = 1 
           end
           xj = x(4*(j-1)+1);
           yj = x(4*(j-1)+2);
           xn = abs(xi-xj)/distFunc(xi,xj,yi,yj);
           yn = abs(yi-yj)/distFunc(xi,xj,yi,yj);
           d = distFunc(xi,xj,yi,yj);
           if  xi <= xj % fordward cars  
               temprupper = xi + dc - dc*(dc-d)/(dc-w);
               if temprupper < rupper 
                   rupper = temprupper;
               end
               if yi < yj 
                  	tempruppery = yi + dc - dc*(dc-d)/(dc-w)*sigmoidFuncEx(yj-yi,1,10);
                    temprdowny  = yi - dc;
               else        
                    tempruppery = yi + dc;
                    temprdowny  = yi - dc + dc*(dc-d)/(dc-w)*sigmoidFuncEx(yi-yj,1,10);
               end
               if tempruppery < ruppery 
                   ruppery = tempruppery;
               end
               if temprdowny > rdowny
                   rdowny = temprdowny;
               end
           elseif xi > xj  % backward cars
               temprdown = xi - dc + dc*xn*(dc-d)/(dc-w);
               if temprdown > rdown 
                   rdown = temprdown;
               end
               if yi <=  yj %back and up neighbours
                   tempruppery = yi + dc - dc*yn*(dc-d)/(dc-w);
                   temprdowny  = yi - dc;
               elseif yi > yj
                   temprdowny = yi - dc + dc*yn*(dc-d)/(dc-w);
                   tempruppery = yi + dc;
               end
               if tempruppery <  ruppery
                  ruppery = tempruppery; 
               end
               if temprdowny > rdowny
                  rdowny = temprdowny; 
               end
           end
       end 

       ksi_x = 2*(xi -(rupper+rdown)/2)/(rupper-rdown);
       ksi_y = 2*(yi -(ruppery+rdowny)/2)/(ruppery-rdowny);
       %{ 
       if abs(ksi_y)>0.9
           ksi_y 
           yi
           ruppery
           rdowny
            yi - w/2
           for k = 1:n
                if i == k 
               continue
                end
                distFunc(xi,xj,yi,yj)
                f = variableSigmoid(distFunc(xi,xj,yi,yj),w,w+d,gain)
           end
   
         end
       %}  
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
       

       %}
       %xdot(4*(i-1)+1) = vxd;
       %xdot(4*(i-1)+2) = vyd;
       % .. ppc like  for x 
       % get a1 
       % ppc like for y 
       % get a2 
       
       %thetad = getThetad(vx,vy,a,gain);
       %vd = getVd(vx,vy);  
      rt = rts{i};
      rv = rvs{i};      
       kt = kts(i);
       kv = kvs(i); 
       vx = vi*cos(ti);
       vy = vi*sin(ti);
       ksi_vx = (vx-vxd)/(rt(t));
       ksi_vy = (vy-vyd)/(rv(t));
       avx = - kt*ppcTrans(ksi_vx);
       avy = - kv*ppcTrans(ksi_vy);
       b = [avx;avy];
       A = [vx/sqrt(vx^2+vy^2) -vy*sqrt(vx^2+vy^2)/w;vy/sqrt(vx^2+vy^2) vx*sqrt(vx^2+vy^2)/w];
       res = inv(A)*b;
       F = res(1);
       u = res(2);
      % ksi_theta = (ti-thetad)/rt(t);
       %ksi_v =  (vi-vd)/rv(t);
      % ksi_v
      % kv
       %u = -kt*ppcTrans(ksi_theta);
       %F = -kv*ppcTrans(ksi_v);       
       xdot(4*(i-1)+3) = vi*u/w;
       %xdot(4*(i-1)+3) =-1000*(x(4*(i-1)+3)-atan2(vyd,vxd));% ;
       xdot(4*(i-1)+4) = F;
       %xdot(4*(i-1)+4) =-1000*(x(4*(i-1)+4)-sqrt(vxd^2+vyd^2));%F ;
      
           yi
           ksi_y
           ruppery
           rdowny
           vyd
       %{     
       if i ==  6
           vyd
           ksi_vy
           %ruppery
        %rdowny
           %   rdown > xi -dc
        %rupper
       % rdown
           % yi
          % ksi_x = 2*(xi -(rupper+rdown)/2)/(rupper-rdown)
           %ksi_y = 2*(yi -(ruppery+rdowny)/2)/(ruppery-rdowny)
          %vxd
        %   vyd
              end
       %}
       if isreal(ksi_vx) && isreal(ksi_vy) && isreal(ksi_x) && isreal(ksi_y)
            continue
       else
             i
              ksi_x
              ksi_y
              ksi_vx
              ksi_vy
              problem  = 1;
          end
       if abs(ksi_x) >= 0.9 || abs(ksi_y) >= 0.9 ||  abs(ksi_vx) >= 0.9 || abs(ksi_vy) >= 0.9
          t;
          i 
          ksi_x
          ksi_y
          ksi_vx
          ksi_vy
          if t > 1  
              problem  = 1
          end
          
          yi;
          ruppery ;
          rdowny;
          %ksi_theta;
          %ksi_v;
          rupper;
          rdown;
          xi;
          malakia = 1;
       end
       %}
    end
end