function resultf =  ffunc(x,e)

    if x <= -e
            resultf = 0;
    elseif x >= 0
        resultf = (e^2 + 2*e*x)/(2*e);
    else
        resultf = ((x+e)^2)/(2*e);
    end
    %}
    %resultf. = e/2 + x^2/(2*e);
end