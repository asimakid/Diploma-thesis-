function res = potentialUder(y,a,c)
    % The potential function for staying inside road
    if  y >= -a*(sqrt(c-1)/sqrt(c)) &&  y <= a*(sqrt(c-1)/sqrt(c))
        res = 0;
    else        
        res = 8*y*(1/(a^2-y^2)-c/a^2)^3/(a^2-y^2);
    end
end
