function res = sigmoidFunc(x,e,k)
    res = 2/(1 + exp(k*x/(x^2-e^2)))-1;
end