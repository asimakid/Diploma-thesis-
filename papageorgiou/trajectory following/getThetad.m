function thetad = getThetad(a1,a2,a,gain)
    global preva2;
    global k;
    if a1 < 0 && sign(a2*preva2) == -1 && a2 < 0       
        k = k + 1;
    elseif a1 < 0 && sign(a2*preva2) == -1 && a2 > 0
        k = k - 1;
    end
    if a1^2 + a2^2 >= a
        thetad = atan2(a2,a1) + 2*k*pi;
    else
        thetad = (atan2(a2,a1) + 2*k*pi)*sigmoidFunc(a1^2+a2^2,a,gain);
    end
end