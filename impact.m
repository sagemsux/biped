function [w1,w2,w3] = impact(th1,th2,th3,th1d,th2d,th3d)

    delta_q = -95 + 10*cos(2*th1-2*th2) + 20*cos(2*th2-2*th3);
    
    w1 = 5*(th1d - 20*th1d*cos(2*th1-2*th2) + 4*th1d*cos(2*th3-2*th1) + 2*th2d*cos(2*th1 - 2*th2)) / delta_q;
    
    w2 = 10*(2*th1d*cos(2*th3-th1-th2) - 9*th1d*cos(th1-th2) + th2d) / delta_q;
    
    w3 = 5*(12*th1d*cos(th1+th3-2*th2) - 12*th1d*cos(th1-th3) + th1d*cos(3*th1-2*th2-th3) - th2d*cos(th2-th3) - 9.5*th3d ...
        + th3d*cos(2*th1-2*th2) + 2*th3d*cos(2*th2-2*th3)) / delta_q;



end
