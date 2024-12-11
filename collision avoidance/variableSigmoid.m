function res = variableSigmoid(x,rangemin,rangemax,gain)
    % a sigmoid function that gets 0 value on range max 
    % and 1 value on renge min 
    % gain is a parameter to choose the changes the slope of the function
    if x < rangemin 
        res = 1;
    elseif x > rangemax 
        res = 0;
    else
        center = (rangemin+rangemax)/2; 
        dist = rangemax/2 - rangemin/2;
        res = 1 - 1/(1+exp(gain*(x-center)/((x-center)^2-dist^2)));
    end
end