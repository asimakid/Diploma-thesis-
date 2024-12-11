function xdot = collSystemVeloc(t,x,n,w,dc,e,mina,maxa,vmax,vstar,ylim)  
    t
    ksi_vstar = 2*(vstar -(vmax)/2)/vmax;
    ashift =  (1-ksi_vstar)/(1+ksi_vstar);
    % n is the number of the cars
    xdot = zeros(4*n,1);
    for i = 1:n
       xi = x(4*(i-1)+1);
       yi = x(4*(i-1)+2);
       ti = x(4*(i-1)+3);
       vi = x(4*(i-1)+4);
       xdot(4*(i-1)+1) = vi*cos(ti);
       xdot(4*(i-1)+2) = vi*sin(ti); 
       vdown =  0;
       vup = vmax;
       tup = pi/2;
       tdown = -pi/2; 
       yl =  0.1;
       temptup = pi/2;
       temptdown = -pi/2;
       tempvup = vmax;
       if yi > maxa - yl - 2
         temptup = pi/2 - pi/2*variableSigmoid(maxa-yi,0,yl+2,10);
       elseif yi  < mina + yl + 2
         temptdown = -pi/2 + pi/2*variableSigmoid(yi-mina,0,yl+2,10);          
       end
       if yi > maxa - yl
         tempvup = vmax*(1-variableSigmoid(maxa-yi,0,yl,0.5));
       elseif yi  < mina + yl
         tempvup =  vmax*(1-variableSigmoid(yi-mina,0,yl,0.5));   
       end
       if temptup <  tup  
               if temptup > ti + e
                   tup = temptup; 
               else
                   tup = ti + e; 
               end                  
       end
       if temptdown > tdown 
              if temptdown < ti - e
                   tdown = temptdown; 
               else
                   tdown = ti - e; 
               end    
       end     
      if tempvup < vup 
          vup = tempvup;
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
           d = distFunc(xi,xj,yi,yj);
           yn = abs(yi-yj)/d;
           if  xi <= xj % fordward cars
               thetamin = abs(yi-(yj-w))/distFunc(xi,xj,yi,yj-w);
               thetamax = abs(yi-(yj+w))/distFunc(xi,xj,yi,yj+w);
               trange = 0.1;
               if d <= dc && ti <=  thetamax + trange && ti >= thetamin-trange
                   tempvup = vmax*(1-(dc-d)/(dc-w));                  
                   
                   if ti >= thetamax 
                       tempvup = tempvup*(variableSigmoid(ti-thetamax,0,trange,4)) + vmax*(1-variableSigmoid(ti-thetamax,0,trange,4));
                   elseif ti <= thetamin
                       tempvup = tempvup*(variableSigmoid(thetamin-ti,0,trange,4)) + vmax*(1-variableSigmoid(thetamin-ti,0,trange,4));
                   else %inside thetamin,thetamax
                   end  
                    
               else
                   tempvup = vmax;
               end                
               if tempvup < vup 
                   vup = tempvup;
               end
           elseif xi > xj  % backward cars
               if d <= dc
                    tempvdown = vi*(variableSigmoid(d,w,dc,0.5));%(dc-d)/(dc-w);
               else
                    tempvdown = 0;
               end
               if tempvdown > vdown 
                   vdown = tempvdown;
               end               
           end
           if d < dc
               if yi <=  yj 
                   temptup = pi/2 - yn*(1-variableSigmoid(d,w,dc,0.2));
                   temptdown  = -pi/2;
               elseif yi > yj
                   temptup = pi/2;
                   temptdown  = -pi/2 + yn*(1-variableSigmoid(d,w,dc,0.2));        
               end
               
               if temptup <  tup  
                   if temptup > ti + e
                       tup = temptup; 
                   else
                       tup = temptup;
                       %tup = ti + e; 
                   end                  
               end
               if temptdown > tdown 
                  if temptdown < ti - e
                       tdown = temptdown; 
                  else
                       tdown = temptdown; 
                       %tdown = ti - e; 
                   end    
               end
              
           end
       end
       ksi_t = 2*(ti -(tup+tdown)/2)/(tup-tdown);
       ksi_v = 2*(vi -(vup+vdown)/2)/(vup-vdown);    
       if t == 0 && abs(ksi_v) > 1 && abs(ksi_t) > 1
           i
           vi
           vup
           vdown
           ksi_t 
           ksi_v
           error("Wrong initial condition")
       end
       if abs(ksi_t) > 0.99 || abs(ksi_v)> 0.99
          i 
          ksi_t
          ti
          tup
          tdown
          ksi_v
          vi
          vup
          vdown   
          yi
          s = 1
       end
       xdot(4*(i-1)+3) = -0.05*x(4*(i-1)+4)*(variableGainSigmoid(ksi_t,-1,1,50)+1)*ppcTrans(ksi_t)/w;
       %xdot(4*(i-1)+3) = -10*ppcTrans(ksi_t);
       xdot(4*(i-1)+4) = -10*ppcTransShifted(ksi_v,ashift); 
    end
end