function [vd,thetad] = calculateDesired(u,vStar,vMax,flim,f)
    if abs(u) < vStar*sin(flim)
       vd =  vStar;
       thetad = asin(u/vd);
    else
        if u > 0
            a = (vMax - vStar)/(sin(f)-sin(flim));
            b = -(vMax-vStar)*sin(flim)/(sin(f)-sin(flim)) + vStar;
            c = -u;
            diak = b^2-4*a*c;
            sol1 = (-b + sqrt(diak))/(2*a);
            sol2 = (-b - sqrt(diak))/(2*a);
            thetad = asin(sol1);
            vd = u/sin(thetad);
        else
            
            a = (vMax - vStar)/(sin(flim)-sin(f));
            b = (vMax-vStar)*sin(flim)/(sin(flim)-sin(f)) + vStar;
            c = -u;
            diak = b^2-4*a*c;
            sol1 = (-b + sqrt(diak))/(2*a);
            sol2 = (-b - sqrt(diak))/(2*a);
            thetad = asin(sol1);
            vd = u/sin(thetad);
        end
    end
end