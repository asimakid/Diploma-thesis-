function res = potentialUderinsert(x,y,a,ax,c)
    % The potential function for staying inside road
    if  y >= -(a+ax(x))*sqrt(c-1)/sqrt(c) &&  y <= (a+ax(x))*sqrt(c-1)/sqrt(c)
        res = 0;
    else        
        res = 8*y*(1/(a^2-y^2)-c/a^2)^3/(a^2-y^2);
    end
end
