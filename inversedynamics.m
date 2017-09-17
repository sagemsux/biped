function [u1,u2] = inversedynamics(l,m,Mh,Mt,r,th1,th2,th3,th1d,th2d,th3d,th1dd,th2dd,th3dd)

    g = 9.81;
    
    qdd = [th1dd;th2dd;th3dd];
    qd = [th1d;th2d;th3d];
    
    D = [(1.25*m + Mh + Mt)*r^2, -0.5*m*r^2*cos(th1-th2), Mt*r*l*cos(th1-th3);
        -0.5*m*r^2*cos(th1-th2), 0.25*m*r^2,              0;
        Mt*r*l*cos(th1-th3),    0,                      Mt*l^2];
    
    C = [0, -0.5*m*r^2*sin(th1+th2), Mt*r*l*sin(th1+th3);
        0.5*m*r^2*sin(th1+th2),  0,  0;
        -Mt*r*l*cos(th1-th3),    0,  0;];
    
    G = [-0.5*g*(2*Mh + 3*m + 2*Mt)*r*sin(th1);
        0.5*g*m*r*sin(th2);
        -g*Mt*l*sin(th3);];
    
    B = [-1 0;
        0 -1;
        1 1];
    
    u = B\(D*qdd + C*qd + G);
    
    u1 = u(1); u2 = u(2);
    
    
end