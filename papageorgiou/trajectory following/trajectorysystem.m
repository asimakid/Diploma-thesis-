function xdot = trajectorysystem(t,x,xd,yd,r1,r2,r3,r4,kx,ky,ktheta,kv,a,gain)
   	%sprintf('%.32f',t);
    global preva2;
    t;
    xdot = zeros(4,1);
    xdot(1) = x(4)*cos(x(3));
    xdot(2) = x(4)*sin(x(3));
    ksi_1 = (x(1)-xd(t))/r1(t);
    ksi_2 = (x(2)-yd(t))/r2(t);
    a1 = -kx*ppcTrans(ksi_1);
    a2 = -ky*ppcTrans(ksi_2);   
    vd = getVd(a1,a2);
    thetad = getThetad(a1,a2,a,gain);
    preva2 = a2;
    vd;
    thetad;
    %a1
    %a2
    %sign(a1)/(2/(1+exp(-a1/e))-1);
    ksi_theta = (x(3)-thetad)/r3(t);
    ksi_v = (x(4)-vd)/r4(t);
    xdot(3) = -ktheta*ppcTrans(ksi_theta)*x(4);
    xdot(4) = -kv*ppcTrans(ksi_v);
    if isreal(ksi_theta) && isreal(ksi_v);
    else
       malakia = 1;
    end   
end