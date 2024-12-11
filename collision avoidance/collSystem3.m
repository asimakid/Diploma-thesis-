function xdot = collSystem3(t,x,n,w,dc,a,mina,maxa,vmax,vstar,ylim)  
    t
    % n is the number of the cars
    xdot = zeros(6*n,1);
    for i = 1:n
       xi = x(6*(i-1)+1);
       yi = x(6*(i-1)+2);
       ti = x(6*(i-1)+3);
       vi = x(6*(i-1)+4);
       vx = vi*cos(ti);
       vy = vi*sin(ti);
       xdot(6*(i-1)+1) = vi*cos(ti);
       xdot(6*(i-1)+2) = vi*sin(ti);               
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
           xj = x(6*(j-1)+1);
           yj = x(6*(j-1)+2);
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
       if abs(ksi_x) > 0.9 || abs(ksi_y) > 0.9 
           i
           ksi_x
           ksi_y
       end
       vyd = -vmax*invPpctrans(x(6*(i-1)+6));
       vdxmax = sqrt(vmax^2-vyd^2);
       vw = min([vdxmax,vstar]);
       if x(6*(i-1)+5)< 0
           vxd = vw-(vdxmax-vw)*invPpctrans(x(6*(i-1)+5));
       elseif x(6*(i-1)+5) == 0
           vxd = vw;
       else
           vxd = vw - vw*invPpctrans(x(6*(i-1)+5));
       end
       xdot(6*(i-1)+1) = vxd;
       xdot(6*(i-1)+2) = vyd;
      % i
     %  vxd
       %vyd
       % .. ppc like  for x 
       % get a1 
       % ppc like for y 
       % get a2 
       
       %thetad = getThetad(vx,vy,a,gain);
        
       %xdot(6*(i-1)+3) = vi*u/w;
       xdot(6*(i-1)+3) = -1000*(x(6*(i-1)+3)-atan2(vyd,vxd));% ;
       %xdot(6*(i-1)+4) = F;
       xdot(6*(i-1)+4) = -1000*(x(6*(i-1)+4)-sqrt(vxd^2+vyd^2));%F ;
       xdot(6*(i-1)+5) = -100*x(6*(i-1)+5) + 100*ppcTrans(ksi_x);
       xdot(6*(i-1)+6) = -100*x(6*(i-1)+6) + 100*ppcTrans(ksi_y);
  
       if  isreal(ksi_x) && isreal(ksi_y)
            continue
       else
             i
              ksi_x
              ksi_y
              problem  = 1;
          end
       if abs(ksi_x) >= 0.9 || abs(ksi_y) >= 0.9 
          t;
          i 
          ksi_x
          ksi_y
  
          if t > 1  
              problem  = 1
          end
          
          yi;
          ruppery ;
          rdowny;
          rupper;
          rdown;
          xi;
          malakia = 1;
       end
       %}
    end
end