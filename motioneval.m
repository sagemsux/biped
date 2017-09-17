function [th,thd,thdd] = motioneval(coeff,t)

    th = coeff(1) + coeff(2).*t + coeff(3).*t.^2 + coeff(4).*t.^3;
    thd = coeff(2) + 2*coeff(3).*t + 3*coeff(4).*t.^2;
    thdd = 2*coeff(3) + 6*coeff(4).*t;
    
end