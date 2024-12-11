function res = saturationfunc(x,satpoint)
    if abs(x) <= satpoint
       res = x/satpoint;
    else
        res = sign(x);
    end
end