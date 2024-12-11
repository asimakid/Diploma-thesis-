function res = potentialV(d,q,L,lamda)
    if d < L
       error("Out of Region")
    end
    % The potential function for keeping cars distance
    if  d >= lamda
        res = 0;
    else
        res = q*((lamda-d)^3/(d-L));
        %res = q^((lamda-d)^3/(d-L));
    end
end