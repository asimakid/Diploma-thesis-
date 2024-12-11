function res = variableGainSigmoid(x,rangemin,rangemax,gain)
    % a sigmoid function that gets 1 value on range max 
    % and 1 value on renge min 
    % gain is a parameter to choose the changes the slope of the function
    % 0 in the middle
    if x < rangemin 
        res = 1;
    elseif x > rangemax 
        res = 0;
    else
        center = (rangemin+rangemax)/2; 
        dist = rangemax/2 - rangemin/2;
        res = -1 + 2/(1+exp(gain*(x-center)^2/((x-center)^2-dist^2)));
    end
end