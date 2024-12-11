function xdot = kinematic(t,x,a,yStar,r_y,thetaStar,r_theta,vStar,vMax,f,flim,uLim,r_V,ky,kt,kv)
    ksi_y = (x(2)-yStar-x(5))/r_y(t);
    u_y = -ky*ppcTrans(ksi_y);
    uyus = u_y;
    u_y = (abs(u_y) > uLim)*uLim*sign(u_y) + (abs(u_y) <= uLim)*u_y;    
    [desiredV,desiredTheta] = calculateDesired(u_y,vStar,vMax,flim,f);
    ksi_theta = (x(3)-desiredTheta)/r_theta(t);
    u = -kt*ppcTrans(ksi_theta);
    ksi_V = (x(4)-desiredV)/r_V(t);
    F = -kv*ppcTrans(ksi_V);
    if isreal(ksi_V) && isreal(ksi_theta) && isreal(ksi_y)
        ;
    else
        malakia = 1;
    end
    xdot = zeros(5,1);
    xdot(1) = x(4)*cos(x(3));
    xdot(2) = x(4)*sin(x(3));
    xdot(3) = u;
    xdot(4) = F;
    b = 1000;
    %uyus 
    %uLim
    %(uyus-uLim)*(uyus > uLim)
    xdot(5) = -b*x(5) + (uyus-uLim)*(uyus > uLim);
end