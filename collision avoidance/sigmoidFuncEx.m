function res = sigmoidFuncEx(x,e,k) 
    if x > e 
        res  = 1;
    elseif x < 0
        res = 0;
    else
        res = 1/(1 + exp(k*(x-e/2)/((x-e/2)^2-(e^2)/4)));
    end
end