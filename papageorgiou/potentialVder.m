function res = potentialVder(d,q,L,lamda)
    % The potential function for keeping cars distance
    if L >lamda
        error("Bad limits")
    end
    if  d >= lamda
        res = 0;
    elseif d >= L
        res = q*(((lamda-d)^2)*(3*L-2*d-lamda))/(d-L)^2;
        %res = q^((lamda-d)^3/(d-L))*log(q)*(3*L-2*d-lamda)/(d-L)^2;
    else 
        d
        error("Out of range")
    end
end