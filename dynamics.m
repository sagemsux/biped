function dy = dynamics(t,y,m,r,l,Mh,Mt,u)

    g = 9.81;
    
    dy = zeros(6,1);
    
    th1 = y(1);
    th2 = y(2);
    th3 = y(3);
    
    w1 = y(4);
    w2 = y(5);
    w3 = y(6);
    
    w = [w1;w2;w3];
    
    D = [(1.25*m + Mh + Mt)*r^2, -0.5*m*r^2*cos(th1-th2), Mt*r*l*cos(th1-th3);
        -0.5*m*r^2*cos(th1-th2), 0.25*m*r^2,              0;
        Mt*r*l*cos(th1-th3),    0,                      Mt*l^2];
    
    C = [0, -0.5*m*r^2*sin(th1+th2)*w2, Mt*r*l*sin(th1+th3)*w3;
        0.5*m*r^2*sin(th1+th2)*w1,  0,  0;
        -Mt*r*l*cos(th1-th3)*w1,    0,  0;];
    
    G = [-0.5*g*(2*Mh + 3*m + 2*Mt)*r*sin(th1);
        0.5*g*m*r*sin(th2);
        -g*Mt*l*sin(th3);];
    
    B = [-1 0;
        0 -1;
        1 1];
    
    dy(1) = w1;
    dy(2) = w2;
    dy(3) = w3;
    
    w_dot = D\(-C*w - G + B*u);
    dy(4) = w_dot(1);
    dy(5) = w_dot(2);
    dy(6) = w_dot(3);


end